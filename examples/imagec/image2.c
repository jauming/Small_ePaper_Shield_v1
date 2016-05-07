/*-------------------------------------------------------------------------------------------
  demo of ePaper Shield for display image in flash
  
  loovee
  2013-7-10
-------------------------------------------------------------------------------------------*/ 

#include <ePaper.h>
//#include <SPI.h>
//#include <SD.h>

//#include "GT20L16_drive.h"

#define SCREEN_SIZE 270             // choose screen size here: 144, 200, 270
#include "picture2.h"

#if (SCREEN_SIZE == 144)
#define EPD_SIZE    EPD_1_44
#define IMAGEFILE   image_144

#elif (SCREEN_SIZE == 200)
#define EPD_SIZE    EPD_2_0
#define IMAGEFILE   image_200

#elif (SCREEN_SIZE == 270)
#define EPD_SIZE    EPD_2_7
#define IMAGEFILE   image_270

#else
#error "Unknown EPB size: Change the #define SCREEN_SIZE to a supported value"
#endif

void setup()
{
	extern void EPAPER_init(void);
	EPAPER_init();
	//ePaper_begin->EPD.begin->ePaper_init_io
    EPAPER.begin(EPD_SIZE);                             // setup epaper, size
	//ePaper_start(EPD.start->EPD.setFactor)
	//->EPD.image(frame_fixed_repeat(frame_fixed)->frame_data_repeat(frame_data))->ePaper_end
    EPAPER.image_flash(IMAGEFILE);
 
}

void loop()
{
    // add code here
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
