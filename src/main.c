/* Includes */
#include "stm32f4xx.h"
#include "stdio.h"
#include "macros.h"
#include "bitband.h"
#include "hardware.h"
#include "video.h"
#include "system.h"

/* Pin Usage
 * PA0-7 	- Pixel bus
 * PA9		- CSYNC input
 * PC6  	- Pixel clock output
 *
 */

/* Private macro */
/* Private variables */
/* Private function prototypes */
/* Private functions */


/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
int main(void)
{
  /**
  *  IMPORTANT NOTE!
  *  The symbol VECT_TAB_SRAM needs to be defined when building the project
  *  if code has been located to RAM and interrupts are used. 
  *  Otherwise the interrupt table located in flash will be used.
  *  See also the <system_*.c> file and how the SystemInit() function updates 
  *  SCB->VTOR register.  
  *  E.g.  SCB->VTOR = 0x20000000;  
  */

  /* TODO - Add your application code here */
  //SystemCoreClockUpdate();
  printf("CVBS 3.00\n");

  initDebug();
  initLeds();
  initSystem();
  initVideo();

  /* Infinite loop */
  while (1)
  {

  }
}
