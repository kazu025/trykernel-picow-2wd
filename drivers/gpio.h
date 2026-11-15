#ifndef RP2040_GPIO_H
#define RP2040_GPIO_H
/*
 * RP2040 GPIOドライバー
 * RP2040のレジスタを直接操作する最小GPIOドライバー
 */
/* --- 汎用GPIO API --- */
void gpio_init_out(unsigned int pin);
void gpio_set(unsigned int pin);
void gpio_clear(unsigned int pin);
void gpio_toggle(unsigned int pin);
void gpio_put(unsigned int pin, int value);

typedef void (*GPIO_IRQ_NOTIFY_FUNC)(void);
BOOL gpio_irq_init_rising(unsigned int pin, GPIO_IRQ_NOTIFY_FUNC notify);
UW gpio_irq_count_get(void);
void io_irq_bank0_handler(void);

/* --- Pico Onboard LED --- */
#define STATUS_LED_PIN 15
void status_led_init(void);
void status_led_on(void);
void status_led_off(void);
void status_led_toggle(void);
/* --- I2C --- */
void gpio_init_i2c(unsigned int pin);
void gpio_init_spi(unsigned int pin);
/* -------------------- */
#endif // RP2040_GPIO_H
