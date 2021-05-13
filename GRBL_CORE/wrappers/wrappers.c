#include "wrappers.h"
#include "stdio.h"
#include "usart.h"
#include "stm32f10x.h"
#include "grbl.h"
#include "stdarg.h"
#include "stdint.h"

char uart_tx_buff[1024]={0};
int HAL_printf(const char *fmt, ...)
{
    
    uint16_t count = 0;
    va_list ap;

    memset(uart_tx_buff,0,sizeof(uart_tx_buff));
    va_start(ap,fmt); 
    count=vsprintf((char*)uart_tx_buff,fmt,ap);
    //user printf

    return count;
}

void HAL_delay_ms(int ms)
{

}

void HAL_delay_us(int us)
{

}

int HAL_serial_init(int baud_rate)
{
    uart1_init(baud_rate);
    return 0;
}

int HAL_serial_write(char *data,int len)
{
    return uart1_send_data(data,len);
}

extern uint8_t serial_rx_buffer[RX_BUFFER_SIZE];
extern uint8_t serial_rx_buffer_head;
extern volatile uint8_t serial_rx_buffer_tail;


extern volatile uint8_t serial_tx_buffer_tail;
extern uint8_t serial_tx_buffer[TX_BUFFER_SIZE];
extern uint8_t serial_tx_buffer_head;


int HAL_serial_idle_isr(void)
{
    uint8_t tail = serial_tx_buffer_tail; // Temporary serial_tx_buffer_tail (to optimize for volatile)
  #ifdef ENABLE_XONXOFF
    if (flow_ctrl == SEND_XOFF) { 
      UDR0 = XOFF_CHAR; 
      flow_ctrl = XOFF_SENT; 
    } else if (flow_ctrl == SEND_XON) { 
      UDR0 = XON_CHAR; 
      flow_ctrl = XON_SENT; 
    } else
  #endif
  { 
    // Send a byte from the buffer	
    HAL_serial_write(&serial_tx_buffer[tail],1);
    // Update tail position
    tail++;
    if (tail == TX_BUFFER_SIZE) { tail = 0; }
    serial_tx_buffer_tail = tail;
  }
  
  // Turn off Data Register Empty Interrupt to stop tx-streaming if this concludes the transfer
    if (tail == serial_tx_buffer_head) 
    { 
        return 0;
    }

    return 1;
}

//串口中断接收 需要在对应接收中断调用
int HAL_serial_tx_isr(char *tx_data,int len)
{
    int i =0;
    uint8_t data = 0;
    uint8_t next_head =0;
    
    for(i=0;i<len;i++)
    {
        data=tx_data[i];
  // Pick off realtime command characters directly from the serial stream. These characters are
  // not passed into the buffer, but these set system state flag bits for realtime execution.
        switch (data) {
            case CMD_STATUS_REPORT: bit_true_atomic(sys_rt_exec_state, EXEC_STATUS_REPORT); break; // Set as true
            case CMD_CYCLE_START:   bit_true_atomic(sys_rt_exec_state, EXEC_CYCLE_START); break; // Set as true
            case CMD_FEED_HOLD:     bit_true_atomic(sys_rt_exec_state, EXEC_FEED_HOLD); break; // Set as true
            case CMD_SAFETY_DOOR:   bit_true_atomic(sys_rt_exec_state, EXEC_SAFETY_DOOR); break; // Set as true
            case CMD_RESET:         mc_reset(); break; // Call motion control reset routine.
            default: // Write character to buffer    
            next_head = serial_rx_buffer_head + 1;
            if (next_head == RX_BUFFER_SIZE) { next_head = 0; }
            
            // Write data to buffer unless it is full.
            if (next_head != serial_rx_buffer_tail) {
                serial_rx_buffer[serial_rx_buffer_head] = data;
                serial_rx_buffer_head = next_head;    
                
                #ifdef ENABLE_XONXOFF
                if ((serial_get_rx_buffer_count() >= RX_BUFFER_FULL) && flow_ctrl == XON_SENT) {
                    flow_ctrl = SEND_XOFF;
                    UCSR0B |=  (1 << UDRIE0); // Force TX
                } 
                #endif
                
            }
            //TODO: else alarm on overflow?
        }
    }
    return i;
}

//gpio output
int HAL_X_axis_init(void *port,int pin)
{

//USART1_TX   GPIOA.9
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = pin; //PA.9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
 
    GPIO_Init((GPIO_TypeDef *)port, &GPIO_InitStructure);//初始化GPIOA.9
    
    return 0;
}

int HAL_Y_axis_init(void)
{
    return 0;
}

int HAL_Z_axis_init(void)
{
    return 0;
}


//gpio output set status
int HAL_X_axis_output(char status)
{

    return 0;
}

int HAL_Y_axis_output(char status)
{
    return 0;
}

int HAL_Z_axis_output(char status)
{
    return 0;
}


int HAL_create_timer(int ms)
{
    return 0;
}

int HAL_timee1_init(void)
{

}

int HAL_timee1_deinit(void)
{
  
}

extern volatile uint8_t busy;  
void HAL_timer1_ISR(void)
{    
  #if 0    
// SPINDLE_ENABLE_PORT ^= 1<<SPINDLE_ENABLE_BIT; // Debug: Used to time ISR
  if (busy) { return; } // The busy-flag is used to avoid reentering this interrupt
  
  // Set the direction pins a couple of nanoseconds before we step the steppers
  DIRECTION_PORT = (DIRECTION_PORT & ~DIRECTION_MASK) | (st.dir_outbits & DIRECTION_MASK);

  // Then pulse the stepping pins
  #ifdef STEP_PULSE_DELAY
    st.step_bits = (STEP_PORT & ~STEP_MASK) | st.step_outbits; // Store out_bits to prevent overwriting.
  #else  // Normal operation
    STEP_PORT = (STEP_PORT & ~STEP_MASK) | st.step_outbits;
  #endif  

  // Enable step pulse reset timer so that The Stepper Port Reset Interrupt can reset the signal after
  // exactly settings.pulse_microseconds microseconds, independent of the main Timer1 prescaler.
  TCNT0 = st.step_pulse_time; // Reload Timer0 counter
  TCCR0B = (1<<CS01); // Begin Timer0. Full speed, 1/8 prescaler

  busy = true;
  sei(); // Re-enable interrupts to allow Stepper Port Reset Interrupt to fire on-time. 
         // NOTE: The remaining code in this ISR will finish before returning to main program.
    
  // If there is no step segment, attempt to pop one from the stepper buffer
  if (st.exec_segment == NULL) {
    // Anything in the buffer? If so, load and initialize next step segment.
    if (segment_buffer_head != segment_buffer_tail) {
      // Initialize new step segment and load number of steps to execute
      st.exec_segment = &segment_buffer[segment_buffer_tail];

      #ifndef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
        // With AMASS is disabled, set timer prescaler for segments with slow step frequencies (< 250Hz).
        TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (st.exec_segment->prescaler<<CS10);
      #endif

      // Initialize step segment timing per step and load number of steps to execute.
      OCR1A = st.exec_segment->cycles_per_tick;
      st.step_count = st.exec_segment->n_step; // NOTE: Can sometimes be zero when moving slow.
      // If the new segment starts a new planner block, initialize stepper variables and counters.
      // NOTE: When the segment data index changes, this indicates a new planner block.
      if ( st.exec_block_index != st.exec_segment->st_block_index ) {
        st.exec_block_index = st.exec_segment->st_block_index;
        st.exec_block = &st_block_buffer[st.exec_block_index];
        
        // Initialize Bresenham line and distance counters
        st.counter_x = st.counter_y = st.counter_z = (st.exec_block->step_event_count >> 1);
      }
      st.dir_outbits = st.exec_block->direction_bits ^ dir_port_invert_mask; 

      #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
        // With AMASS enabled, adjust Bresenham axis increment counters according to AMASS level.
        st.steps[X_AXIS] = st.exec_block->steps[X_AXIS] >> st.exec_segment->amass_level;
        st.steps[Y_AXIS] = st.exec_block->steps[Y_AXIS] >> st.exec_segment->amass_level;
        st.steps[Z_AXIS] = st.exec_block->steps[Z_AXIS] >> st.exec_segment->amass_level;
      #endif
      
    } else {
      // Segment buffer empty. Shutdown.
      st_go_idle();
      bit_true_atomic(sys_rt_exec_state,EXEC_CYCLE_STOP); // Flag main program for cycle end
      return; // Nothing to do but exit.
    }  
  }
  
  
  // Check probing state.
  probe_state_monitor();
   
  // Reset step out bits.
  st.step_outbits = 0; 

  // Execute step displacement profile by Bresenham line algorithm
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    st.counter_x += st.steps[X_AXIS];
  #else
    st.counter_x += st.exec_block->steps[X_AXIS];
  #endif  
  if (st.counter_x > st.exec_block->step_event_count) {
    st.step_outbits |= (1<<X_STEP_BIT);
    st.counter_x -= st.exec_block->step_event_count;
    if (st.exec_block->direction_bits & (1<<X_DIRECTION_BIT)) { sys.position[X_AXIS]--; }
    else { sys.position[X_AXIS]++; }
  }
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    st.counter_y += st.steps[Y_AXIS];
  #else
    st.counter_y += st.exec_block->steps[Y_AXIS];
  #endif    
  if (st.counter_y > st.exec_block->step_event_count) {
    st.step_outbits |= (1<<Y_STEP_BIT);
    st.counter_y -= st.exec_block->step_event_count;
    if (st.exec_block->direction_bits & (1<<Y_DIRECTION_BIT)) { sys.position[Y_AXIS]--; }
    else { sys.position[Y_AXIS]++; }
  }
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    st.counter_z += st.steps[Z_AXIS];
  #else
    st.counter_z += st.exec_block->steps[Z_AXIS];
  #endif  
  if (st.counter_z > st.exec_block->step_event_count) {
    st.step_outbits |= (1<<Z_STEP_BIT);
    st.counter_z -= st.exec_block->step_event_count;
    if (st.exec_block->direction_bits & (1<<Z_DIRECTION_BIT)) { sys.position[Z_AXIS]--; }
    else { sys.position[Z_AXIS]++; }
  }  

  // During a homing cycle, lock out and prevent desired axes from moving.
  if (sys.state == STATE_HOMING) { st.step_outbits &= sys.homing_axis_lock; }   

  st.step_count--; // Decrement step events count 
  if (st.step_count == 0) {
    // Segment is complete. Discard current segment and advance segment indexing.
    st.exec_segment = NULL;
    if ( ++segment_buffer_tail == SEGMENT_BUFFER_SIZE) { segment_buffer_tail = 0; }
  }

  st.step_outbits ^= step_port_invert_mask;  // Apply step port invert mask    
  busy = false;
// SPINDLE_ENABLE_PORT ^= 1<<SPINDLE_ENABLE_BIT; // Debug: Used to time ISR
#endif
}