/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{																			}
{       Filename: gpio.c													}
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

#include <stdbool.h>			// C standard unit for bool, true, false
#include <stdint.h>				// C standard unit for uint32_t etc
#include <fcntl.h>				// Needed for linux file access			
#include <sys/mman.h>			// Needed for linux memory access
#include <unistd.h>				// Need for file definitions
#include "gpio.h"				// This units header

#if GPIO_DRIVER_VERSION != 1000
#error "Header does not match this version of file"
#endif

#define GPFSEL   0		// 0x00  GPFSEL0 - GPFSEL5
#define GPSET    7		// 0x1C  GPSET0 - GPSET1
#define GPCLR    10		// 0x28  GPCLR0 - GPCLR1
#define GPLEV    13		// 0x34  GPLEV0 - GPLEV1  ** READ ONLY
#define GPEDS    16		// 0x40  GPEDS0 - GPEDS1
#define GPREN    19		// 0x4C  GPREN0 - GPREN1
#define GPFEN    22		// 0x58  GPFEN0 - GPFEN1
#define GPHEN    25		// 0x64  GPHEN0 - GPHEN1
#define GPLEN    28		// 0x70  GPLEN0 - GPLEN1
#define GPAREN   31		// 0x7C  GPAREN0 - GPAREN1
#define GPAFEN   34		// 0x88  GPAFEN0 - GPAFEN1
#define GPPUD	 37		// 0x94  GPPUD
#define GPPUDCLK 38		// 0x98  GPPUDCLK0 - GPPUDCLK1

struct gpio_device
{
	uint8_t gpio_num;							// GPIO device table number
	uint32_t gpio_size;							// GPIO size
	volatile uint32_t* gpio_map;				// Address GPIO is mapped at
};

/* Global table of GPIO devices.  */
static uint16_t gpio_cnt = 0;					// We start with zero GPIO handles in use
static struct gpio_device gpiotab[NGPIO] = { 0 };

/*-[GPIO_Open]--------------------------------------------------------------}
. Creates a GPIO handle which provides access to the GPIO at given address.
. RETURN: GPIO_HANDLE id for success, INVALID_GPIO_HANDLE for any failure
.--------------------------------------------------------------------------*/
GPIO_HANDLE GPIO_Open (uint32_t gpio_base, uint32_t gpio_size)
{
	GPIO_HANDLE gpio = 0;											// Preset null handle
	if (gpio_cnt < NGPIO)											// Check there is a spare GPIO handle
	{
		uint16_t gpio_num;
		for (gpio_num = 0; gpio_num < NGPIO; gpio_num++)			// Search gpio table
			if (gpiotab[gpio_num].gpio_size == 0) break;			// Found empty gpio handle
		if (gpio_size < 4096) gpio_size = 4096;						// Size can't be smaller than 4K
		int mem_fd = open("/dev/gpiomem", O_RDWR | O_SYNC);
		if (mem_fd >= 0)											// GPIO device opened correctly
		{
			struct gpio_device* gpio_ptr = &gpiotab[gpio_num];		// GPIO device pointer 
			/* mmap GPIO */
			void* gpio_map = mmap(
				NULL,												// Any adddress in our space will do
				gpio_size,											// Map length
				PROT_READ | PROT_WRITE,								// Enable reading & writting to mapped memory
				MAP_SHARED,											// Shared with other processes
				mem_fd,												// File to map
				gpio_base											// Physical address to GPIO peripheral
			);
			close(mem_fd);											// No need to keep mem_fd open after mmap

			if (gpio_map != MAP_FAILED)								// Mapping did not fail
			{
				gpio_ptr->gpio_num = gpio_num;						// Hold gpio device number
				gpio_ptr->gpio_map = (volatile uint32_t*)gpio_map;	// Hold the GPIO map
				gpio_ptr->gpio_size = gpio_size;					// Hold GPIO size
				gpio = gpio_ptr;									// Return the handle
				gpio_cnt++;											// Increment gpio handles used count
			}
		}
	}
	return(gpio);													// Return the handle
}

/*-[GPIO_Close]-------------------------------------------------------------}
. Given a valid GPIO handle the access is released and the handle freed.
. RETURN: true for success, false for any failure
.--------------------------------------------------------------------------*/
bool GPIO_Close (GPIO_HANDLE gpioHandle)
{
	int status_value;
	if (gpioHandle && gpioHandle->gpio_size)						// Gpio handle valid and entry in use
	{
		status_value = munmap((void*)gpioHandle->gpio_map, gpioHandle->gpio_size);
		gpioHandle->gpio_map = 0;									// Clear map
		gpioHandle->gpio_num = 0;									// Clear number
		gpioHandle->gpio_size = 0;									// Clear size
		gpio_cnt--;													// Decrement gpio handles used count
		if (status_value == 0) return true;							// Unmap success return true
	}
	return false;													// Something falied return false
}

/*-[GPIO_Setup]-------------------------------------------------------------}
. Given a valid GPIO handle and mode sets GPIO to given mode
. RETURN: true for success, false for any failure
.--------------------------------------------------------------------------*/
bool GPIO_Setup (GPIO_HANDLE gpioHandle, uint8_t gpio, GPIOMODE mode)
{
	if (gpioHandle && gpioHandle->gpio_size && gpio < 54)			// Check GPIO handle and pin number valid
	{
		if (mode < 0 || mode > GPIO_ALTFUNC3) return false;			// Check requested mode is valid, return false if invalid
		uint32_t bit = ((gpio % 10) * 3);							// Create bit mask
		uint32_t regnum = gpio / 10;								// Register number
		uint32_t mem = gpioHandle->gpio_map[GPFSEL + regnum];		// Read register
		mem &= ~(7 << bit);											// Clear GPIO mode bits for that port
		mem |= (mode << bit);										// Logical OR GPIO mode bits
		gpioHandle->gpio_map[GPFSEL + regnum] = mem;				// Write value to register
		return true;												// Return true
	}
	return false;													// Return false
}

/*-[GPIO_Output]------------------------------------------------------------}
. Given a valid GPIO port number the output is set high(true) or Low (false)
. RETURN: true for success, false for any failure
.--------------------------------------------------------------------------*/
bool GPIO_Output (GPIO_HANDLE gpioHandle, uint8_t gpio, bool on)
{
	if (gpioHandle && gpioHandle->gpio_size && gpio < 54)			// Check GPIO handle and pin number valid
	{
		uint32_t regnum = gpio / 32;								// Register number
		uint32_t bit = 1 << (gpio % 32);							// Create mask bit
		uint8_t offset = (on == true) ? GPSET : GPCLR;				// Either set or clear offset
		gpioHandle->gpio_map[offset + regnum] = bit;				// Write value to register
		return true;												// Return true
	}
	return false;													// Return false
}

/*-[GPIO_Input]-------------------------------------------------------------}
. Reads the actual level of the GPIO port number
. RETURN: true = GPIO input high, false = GPIO input low
.--------------------------------------------------------------------------*/
bool GPIO_Input (GPIO_HANDLE gpioHandle, uint8_t gpio)
{
	if (gpioHandle && gpioHandle->gpio_size && gpio < 54)			// Check GPIO handle and pin number valid
	{
		uint32_t bit = 1 << (gpio % 32);							// Create mask bit
		uint32_t mem = gpioHandle->gpio_map[GPLEV + (gpio / 32)];	// Read port level
		if (mem & bit) return true;									// Return true if bit set
	}
	return false;													// Return false
}

/*-[GPIO_CheckEvent]-------------------------------------------------------}
. Checks the given GPIO port number for an event/irq flag.
. RETURN: true for event occured, false for no event
.-------------------------------------------------------------------------*/
bool GPIO_CheckEvent (GPIO_HANDLE gpioHandle, uint8_t gpio)
{
	if (gpioHandle && gpioHandle->gpio_size && gpio < 54)			// Check GPIO handle and pin number valid
	{
		uint32_t bit = 1 << (gpio % 32);							// Create mask bit
		uint32_t mem = gpioHandle->gpio_map[GPEDS + (gpio / 32)];	// Read event detect status register
		if (mem & bit) return true;									// Return true if bit set
	}
	return false;													// Return false
}

/*-[GPIO_ClearEvent]-------------------------------------------------------}
. Clears the given GPIO port number event/irq flag.
. RETURN: true for success, false for any failure
.-------------------------------------------------------------------------*/
bool GPIO_ClearEvent (GPIO_HANDLE gpioHandle, uint8_t gpio)
{
	if (gpioHandle && gpioHandle->gpio_size && gpio < 54)			// Check GPIO handle and pin number valid
	{
		uint32_t bit = 1 << (gpio % 32);							// Create mask bit
		gpioHandle->gpio_map[GPEDS + (gpio / 32)] = bit;			// Clear the event from GPIO register
		return true;												// Return true
	}
	return false;													// Return false
}

/*-[GPIO_EdgeDetect]-------------------------------------------------------}
. Sets GPIO port number edge detection to lifting/falling in Async/Sync mode
. RETURN: true for success, false for any failure
.-------------------------------------------------------------------------*/
bool GPIO_EdgeDetect (GPIO_HANDLE gpioHandle, uint8_t gpio, bool lifting, bool Async)
{
	if (gpioHandle && gpioHandle->gpio_size && gpio < 54)			// Check GPIO handle and pin number valid
	{
		uint32_t bit = 1 << (gpio % 32);							// Create mask bit
		uint32_t regnum = gpio / 32;								// Register number
		if (lifting) {												// Lifting edge detect
			if (Async) gpioHandle->gpio_map[GPAREN + regnum] = bit;	// Asynchronous lifting edge detect register bit set
			else gpioHandle->gpio_map[GPREN + regnum] = bit;		// Synchronous lifting edge detect register bit set
		}
		else {														// Falling edge detect
			if (Async) gpioHandle->gpio_map[GPAFEN + regnum] = bit;	// Asynchronous falling edge detect register bit set
			else gpioHandle->gpio_map[GPFEN + regnum] = bit;		// Synchronous falling edge detect register bit set
		}
		return true;												// Return true
	}
	return false;													// Return false
}