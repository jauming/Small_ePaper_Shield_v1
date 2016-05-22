#if !defined(GPIO_H)
#define GPIO_H 1

#define HIGH 1
#define LOW  0

#define INPUT  0
#define OUTPUT 1

#define SCK  0x10
#define MOSI 0x11
#define MISO 0x12


typedef unsigned char uint8_t;

extern void digitalWrite(uint8_t idx, uint8_t output);
extern uint8_t digitalRead(uint8_t pin);
extern void pinMode(uint8_t idx, uint8_t pinDirection);
	
#endif
