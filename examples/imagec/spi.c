#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_dspi.h"
#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"

//pinmux:
#include "fsl_common.h"
#include "fsl_port.h"
//#include "pin_mux.h"

#include "spi.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define EXAMPLE_DSPI_MASTER_BASEADDR         SPI0
#define DSPI_MASTER_CLK_SRC                  DSPI0_CLK_SRC
#define EXAMPLE_DSPI_MASTER_PCS_FOR_INIT     kDSPI_Pcs0
#define EXAMPLE_DSPI_MASTER_PCS_FOR_TRANSFER kDSPI_MasterPcs0

#define EXAMPLE_DSPI_SLAVE_BASEADDR SPI1
#define TRANSFER_SIZE               1U//256U    /*! Transfer dataSize */
//epd:4M~12M
//#define TRANSFER_BAUDRATE           500000U /*! Transfer baudrate - 500k */
#define TRANSFER_BAUDRATE           4000000U /*! Transfer baudrate - 4M */

/*******************************************************************************
 * Variables
 ******************************************************************************/
static uint8_t masterRxData[TRANSFER_SIZE] = {0U};
static uint8_t masterTxData[TRANSFER_SIZE] = {0U};

static uint32_t srcClock_Hz;
//static uint32_t errorCount;
//static uint32_t i;
static dspi_master_config_t masterConfig;
static dspi_transfer_t      masterXfer;
    
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
/*
typedef struct _spi_t
{
  void (*begin)(void);
  void (*end)(void);
  uint8_t (*transfer)(transfer c);
  void (*setBitOrder)(uint8_t BitOrder);
  void (*setDataMode)(uint8_t DataMode); 
} spi_t;
*/

spi_t SPI;

static void SPI_begin(void) //Initializes the SPI bus by setting SCK, MOSI, and SS to outputs, pulling SCK and MOSI low, and SS high. 
{
    //BOARD_InitPins();
    //BOARD_BootClockRUN();
    //BOARD_InitDebugConsole();
    
    //pinmux:
    //spi0: a14,15,16,17
    //spi1: d4,5,6,7
    CLOCK_EnableClock(kCLOCK_PortA);
    /* SPI0 */
    PORT_SetPinMux(PORTA, 14U, kPORT_MuxAlt2);
    PORT_SetPinMux(PORTA, 15U, kPORT_MuxAlt2);
    PORT_SetPinMux(PORTA, 16U, kPORT_MuxAlt2);
    PORT_SetPinMux(PORTA, 17U, kPORT_MuxAlt2);

    /*CLOCK_EnableClock(kCLOCK_PortD);
    // SPI1 
    PORT_SetPinMux(PORTD, 4U, kPORT_MuxAlt7); //reset
    PORT_SetPinMux(PORTD, 5U, kPORT_MuxAlt7);
    PORT_SetPinMux(PORTD, 6U, kPORT_MuxAlt7);
    PORT_SetPinMux(PORTD, 7U, kPORT_MuxAlt7);
    */
    
    /* Master config */
    masterConfig.whichCtar               = kDSPI_Ctar0;
    masterConfig.ctarConfig.baudRate     = TRANSFER_BAUDRATE;
    masterConfig.ctarConfig.bitsPerFrame = 8U;
    
    masterConfig.ctarConfig.cpol         = kDSPI_ClockPolarityActiveHigh;
    masterConfig.ctarConfig.cpha         = kDSPI_ClockPhaseFirstEdge;
    masterConfig.ctarConfig.direction    = kDSPI_MsbFirst;
    
    masterConfig.ctarConfig.pcsToSckDelayInNanoSec        = 1000000000U / TRANSFER_BAUDRATE;
    masterConfig.ctarConfig.lastSckToPcsDelayInNanoSec    = 1000000000U / TRANSFER_BAUDRATE;
    masterConfig.ctarConfig.betweenTransferDelayInNanoSec = 1000000000U / TRANSFER_BAUDRATE;

    masterConfig.whichPcs           = EXAMPLE_DSPI_MASTER_PCS_FOR_INIT;
    masterConfig.pcsActiveHighOrLow = kDSPI_PcsActiveLow;

    masterConfig.enableContinuousSCK        = false;
    masterConfig.enableRxFifoOverWrite      = false;
    masterConfig.enableModifiedTimingFormat = false;
    masterConfig.samplePoint                = kDSPI_SckToSin0Clock;

    srcClock_Hz = CLOCK_GetFreq(DSPI_MASTER_CLK_SRC);
    DSPI_MasterInit(EXAMPLE_DSPI_MASTER_BASEADDR, &masterConfig, srcClock_Hz);
}
static void SPI_end(void) //Disables the SPI bus (leaving pin modes unchanged). 
{
    //DSPI_Deinit(EXAMPLE_DSPI_MASTER_BASEADDR);
}

static uint8_t SPI_transfer(uint8_t c)
{
    masterTxData[0]=c;
    /* Start master transfer */    
    masterXfer.txData      = masterTxData;
    masterXfer.rxData      = masterRxData;
    masterXfer.dataSize    = TRANSFER_SIZE;
    masterXfer.configFlags = kDSPI_MasterCtar0 | EXAMPLE_DSPI_MASTER_PCS_FOR_TRANSFER | kDSPI_MasterPcsContinuous;

    DSPI_MasterTransferBlocking(EXAMPLE_DSPI_MASTER_BASEADDR, &masterXfer);
    return masterRxData[0];
}

static void SPI_setBitOrder(uint8_t BitOrder)
{
  switch(BitOrder)
  {
  case MSBFIRST:
    masterConfig.ctarConfig.direction    = kDSPI_MsbFirst;
    DSPI_MasterInit(EXAMPLE_DSPI_MASTER_BASEADDR, &masterConfig, srcClock_Hz);
    break;
  }
}
static void SPI_setDataMode(uint8_t DataMode)
{
  switch(DataMode)
  {
  case SPI_MODE0:
    masterConfig.ctarConfig.cpol         = kDSPI_ClockPolarityActiveHigh;
    masterConfig.ctarConfig.cpha         = kDSPI_ClockPhaseFirstEdge;
    DSPI_MasterInit(EXAMPLE_DSPI_MASTER_BASEADDR, &masterConfig, srcClock_Hz);
    break;
  }
}

void spi_init(void)
{       
  SPI.begin       = SPI_begin;
  SPI.end         = SPI_end;
  SPI.transfer    = SPI_transfer;
  SPI.setBitOrder = SPI_setBitOrder;
  SPI.setDataMode = SPI_setDataMode;
}

