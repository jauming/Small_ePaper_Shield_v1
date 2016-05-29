#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "clock_config.h"
#include "pin_mux.h"

#include "fsl_port.h" //kPORT_MuxAsGpio
//#include "pin_mux.h"
#include "fsl_common.h" //kPORT_MuxAsGpio

#include "gpio.h"

/*@}*/

/*! @name GPIO Output Operations */
/*@{*/

/*!
 * @brief Sets the output level of the multiple GPIO pins to the logic 1 or 0.
 *
 * @param base    GPIO peripheral base pointer(GPIOA, GPIOB, GPIOC, and so on.)
 * @param pin     GPIO pin's number
 * @param output  GPIO pin output logic level.
 *        - 0: corresponding pin output low logic level.
 *        - 1: corresponding pin output high logic level.
 */
/*static inline void GPIO_WritePinOutput(GPIO_Type *base, uint32_t pin, uint8_t output)
{
    if (output == 0U)
    {
        base->PCOR = 1 << pin;
    }
    else
    {
        base->PSOR = 1 << pin;
    }
}*/
/*@}*/

/*! @name GPIO Input Operations */
/*@{*/

/*!
 * @brief Reads the current input value of the whole GPIO port.
 *
 * @param base GPIO peripheral base pointer(GPIOA, GPIOB, GPIOC, and so on.)
 * @param pin     GPIO pin's number
 * @retval GPIO port input value
 *        - 0: corresponding pin input low logic level.
 *        - 1: corresponding pin input high logic level.
 */
/*static inline uint32_t GPIO_ReadPinInput(GPIO_Type *base, uint32_t pin)
{
    return (((base->PDIR) >> pin) & 0x01U);
}*/
/*@}*/
/*
typedef struct _gpio_pin_config
{
    gpio_pin_direction_t pinDirection; //!< gpio direction, input or output 
    // Output configurations, please ignore if configured as a input one 
    uint8_t outputLogic; //!< Set default output logic, no use in input 
} gpio_pin_config_t;
*/
/*
typedef enum _gpio_pin_direction
{
    kGPIO_DigitalInput = 0U,  //!< Set current pin as digital input
    kGPIO_DigitalOutput = 1U, //!< Set current pin as digital output
} gpio_pin_direction_t;
*/
PORT_Type *idx2port[11]=
{
 0, 0, 
 PORTD, 
 PORTD, 
 PORTD,
 PORTC, //pwm 
 PORTD, 
 PORTC, //[7]=input
 PORTC, 
 PORTC, 
 PORTC,  
};
GPIO_Type *idx2gpio[11]=
{
 0, 0, 
 GPIOD, 
 GPIOD, 
 GPIOD,
 GPIOC, //pwm 
 GPIOD, 
 GPIOC, //[7]=input
 GPIOC, 
 GPIOC, 
 GPIOC,  
};
uint32_t idx2pin[11]=
{
  0, 0, 
  1,  //d1 
  3,  //d3
  2,  //d2
  8,  //c8,c9, //pwm
  4,  //d4
  5,  //c5 //[7]=input
  3,  //c3
  10, //c10
  11, //c11
};
/*
#define SCK  0
#define MOSI 1
#define MISO 2
spi0
a14,cs
a15,sck
a16,sout
a17,sin

*/
PORT_Type *spi2port[3]=
{
  PORTA,
  PORTA,
  PORTA,
};
GPIO_Type *spi2gpio[3]=
{
  GPIOA,
  GPIOA,
  GPIOA,
};
uint32_t spi2pin[3]=
{
  15,
  16,
  17,
};
/*
static inline void PORT_SetPinMux(PORT_Type *base, uint32_t pin, port_mux_t mux)
{
    base->PCR[pin] = (base->PCR[pin] & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(mux);
}
*/
//static void GPIO_PinInit(GPIO_Type *base, uint32_t pin, const gpio_pin_config_t *config)
static void GPIO_PinInit2(GPIO_Type *base, uint32_t pin, gpio_pin_direction_t pinDirection, uint8_t outputLogic)
{
    
    //if (config->pinDirection == kGPIO_DigitalInput)
		if (pinDirection == kGPIO_DigitalInput)
    {
        base->PDDR &= ~(1U << pin);
        


    }
    else
    {
        //GPIO_WritePinOutput(base, pin, config->outputLogic);
			  GPIO_WritePinOutput(base, pin, outputLogic);
        base->PDDR |= (1U << pin);
    }
}
void digitalWrite(uint8_t idx, uint8_t output)
{
  if (idx < 0x10)
       GPIO_WritePinOutput(idx2gpio[idx], idx2pin[idx], output);
  else if ((idx >= 0x10) && (idx < 0xa0)) 
       GPIO_WritePinOutput(spi2gpio[idx-0x10], spi2pin[idx-0x10], output);
}
void pinMode(uint8_t idx, uint8_t pinDirection)
{
  CLOCK_EnableClock(kCLOCK_PortA);
  CLOCK_EnableClock(kCLOCK_PortC);
  CLOCK_EnableClock(kCLOCK_PortD); 
  
  if (idx < 0x10)
  {
    PORT_SetPinMux(idx2port[idx], idx2pin[idx], kPORT_MuxAsGpio); //<chk
    GPIO_PinInit2(idx2gpio[idx], idx2pin[idx],(gpio_pin_direction_t) pinDirection, 0);
    if (pinDirection == kGPIO_DigitalInput)
    {
      /* Input pin PORT configuration */
      port_pin_config_t config = {
          kPORT_PullUp, //kPORT_PullDown
          kPORT_FastSlewRate,
          kPORT_PassiveFilterDisable,
          kPORT_OpenDrainDisable,
          kPORT_LowDriveStrength,
          kPORT_MuxAsGpio,
         
          0,//kPORT_UnLockRegister,
      
      };
      /*  Sets the configuration */
      PORT_SetPinConfig(idx2port[idx]/*PORTC*/, idx2pin[idx]/*5*/, &config);    
    }
  }
  else if ((idx >= 0x10) && (idx < 0xa0)) 
  {
    PORT_SetPinMux(spi2port[idx-0x10], spi2pin[idx-0x10], kPORT_MuxAsGpio); //<chk
    GPIO_PinInit2(spi2gpio[idx-0x10], spi2pin[idx-0x10],(gpio_pin_direction_t) pinDirection, 0);
  }
}
uint8_t digitalRead(uint8_t idx)
{
	uint8_t ret = 0;
  ret = GPIO_ReadPinInput(idx2gpio[idx], idx2pin[idx]);
  
  if (ret) ret = HIGH;
  else ret = LOW;
  
	return ret;
}
