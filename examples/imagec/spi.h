#if !defined(SPI_H)
#define SPI_H 1

typedef unsigned int   uint32_t;
typedef unsigned short uint16_t;
typedef unsigned char  uint8_t;
typedef unsigned char  _bool;

#define SPI_MODE0 0
#define SPI_MODE1 1
#define SPI_MODE2 2
#define SPI_MODE3 3

#define MSBFIRST 0
#define LSBFIRST 1

 /*
    SPI.end();   //Disables the SPI bus (leaving pin modes unchanged). 
    SPI.begin(); //Initializes the SPI bus by setting SCK, MOSI, and SS to outputs, pulling SCK and MOSI low, and SS high. 
    SPI.transfer(c);
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);

    receivedVal   = SPI.transfer(val)
    receivedVal16 = SPI.transfer16(val16)
    SPI.transfer(buffer, size) 
*/
typedef struct _spi_t
{
  void (*begin)(void);
  void (*end)(void);
  uint8_t (*transfer)(uint8_t c);
  void (*setBitOrder)(uint8_t BitOrder);
  void (*setDataMode)(uint8_t DataMode); 
} spi_t;

extern spi_t SPI;

#endif
