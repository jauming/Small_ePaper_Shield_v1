#if !defined(DELAY_H)
#define DELAY_H 1

extern volatile uint32_t lptmrCounterMs;
extern void delay_init(void);
extern void Delay_ms(int ms);
extern void Delayus(volatile uint32_t count);
extern unsigned long millis(void);         
#endif
