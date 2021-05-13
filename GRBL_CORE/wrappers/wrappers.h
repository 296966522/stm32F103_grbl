#ifndef _WRAPPERS_H_
#define _WRAPPERS_H_


int HAL_printf(const char *fmt, ...);

void HAL_delay_ms(int ms);

void HAL_delay_us(int us);

int HAL_X_axis_init(void *port,int pin);

int HAL_serial_init(int baud_rate);

int HAL_serial_idle_isr(void);

int HAL_serial_tx_isr(char *tx_data,int len);

#endif // !_WRAPPERS_H_
