#ifndef _GPIO_H_
#define _GPIO_H_

#ifdef __cplusplus								// If we are including to a C++
extern "C" {									// Put extern C directive wrapper around
#endif

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{																			}
{       Filename: gpio.h													}
{       Copyright(c): Leon de Boer(LdB) 2020								}
{       Version: 1.00														}
{																			}
{***************************************************************************}
{                                                                           }
{     Defines an API interface for the GPIO acces on Linux					}
{																            }
{++++++++++++++++++++++++[ REVISIONS ]++++++++++++++++++++++++++++++++++++++}
{  1.00 Initial version														}
{++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include <stdbool.h>							// C standard unit for bool, true, false
#include <stdint.h>								// C standard unit for uint32_t etc

#define GPIO_DRIVER_VERSION 1000				// Version number 1.00 build 0

typedef struct gpio_device* GPIO_HANDLE;		// Define a GPIO_HANDLE pointer to opaque internal struct

#define NGPIO 4									// 4 GPIO devices supported

/*--------------------------------------------------------------------------}
;{	      ENUMERATED FSEL REGISTERS ... BCM2835.PDF MANUAL see page 92		}
;{-------------------------------------------------------------------------*/
/* In binary so any error is obvious */
typedef enum {
	GPIO_INPUT = 0b000,							// 0
	GPIO_OUTPUT = 0b001,						// 1
	GPIO_ALTFUNC5 = 0b010,						// 2
	GPIO_ALTFUNC4 = 0b011,						// 3
	GPIO_ALTFUNC0 = 0b100,						// 4
	GPIO_ALTFUNC1 = 0b101,						// 5
	GPIO_ALTFUNC2 = 0b110,						// 6
	GPIO_ALTFUNC3 = 0b111,						// 7
} GPIOMODE;


/*-[GPIO_Open]--------------------------------------------------------------}
. Creates a GPIO handle which provides access to the GPIO at given address.
. RETURN: GPIO_HANDLE id for success, INVALID_GPIO_HANDLE for any failure
.--------------------------------------------------------------------------*/
GPIO_HANDLE GPIO_Open (uint32_t gpio_base, uint32_t gpio_size);

/*-[GPIO_Close]-------------------------------------------------------------}
. Given a valid GPIO handle the access is released and the handle freed.
. RETURN: true for success, false for any failure
.--------------------------------------------------------------------------*/
bool GPIO_Close (GPIO_HANDLE gpioHandle);

/*-[GPIO_Setup]-------------------------------------------------------------}
. Given a valid GPIO handle and mode sets GPIO to given mode
. RETURN: true for success, false for any failure
.--------------------------------------------------------------------------*/
bool GPIO_Setup (GPIO_HANDLE gpioHandle, uint8_t gpio, GPIOMODE mode);

/*-[GPIO_output]------------------------------------------------------------}
. Given a valid GPIO port number the output is set high(true) or Low (false)
. RETURN: true for success, false for any failure
.--------------------------------------------------------------------------*/
bool GPIO_Output (GPIO_HANDLE gpioHandle, uint8_t gpio, bool on);

/*-[GPIO_input]-------------------------------------------------------------}
. Reads the actual level of the GPIO port number
. RETURN: true = GPIO input high, false = GPIO input low
.--------------------------------------------------------------------------*/
bool GPIO_Input (GPIO_HANDLE gpioHandle, uint8_t gpio);

/*-[GPIO_checkEvent]-------------------------------------------------------}
. Checks the given GPIO port number for an event/irq flag.
. RETURN: true for event occured, false for no event
.-------------------------------------------------------------------------*/
bool GPIO_CheckEvent (GPIO_HANDLE gpioHandle, uint8_t gpio);

/*-[GPIO_clearEvent]-------------------------------------------------------}
. Clears the given GPIO port number event/irq flag.
. RETURN: true for success, false for any failure
.-------------------------------------------------------------------------*/
bool GPIO_ClearEvent (GPIO_HANDLE gpioHandle, uint8_t gpio);

/*-[GPIO_edgeDetect]-------------------------------------------------------}
. Sets GPIO port number edge detection to lifting/falling in Async/Sync mode
. RETURN: true for success, false for any failure
.-------------------------------------------------------------------------*/
bool GPIO_EdgeDetect (GPIO_HANDLE gpioHandle, uint8_t gpio, bool lifting, bool Async);

#ifdef __cplusplus								// If we are including to a C++ file
}												// Close the extern C directive wrapper
#endif

#endif