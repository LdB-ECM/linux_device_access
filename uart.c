/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{																			}
{       Filename: uart.c													}
{       Copyright(c): Leon de Boer(LdB) 2020								}
{       Version: 1.00														}
{																			}
{***************************************************************************}
{                                                                           }
{     Defines an API interface for the UART devices on Linux				}
{																            }
{++++++++++++++++++++++++[ REVISIONS ]++++++++++++++++++++++++++++++++++++++}
{  1.00 Initial version														}
{++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#include <stdbool.h>			// C standard unit for bool, true, false
#include <stdint.h>				// C standard unit for uint32_t etc
#include <stdio.h>
#include <unistd.h>				// Used for UART
#include <fcntl.h>				// Used for UART
#include <termios.h>			// Used for UART
#include <semaphore.h>			// Linux Semaphore unit
#include "uart.h"				// Inlcude this units header

#if UART_DRIVER_VERSION != 1000
#error "Header does not match this version of file"
#endif

struct uart_device
{
	int uart_fd;								// File descriptor for the UART device
	uint32_t uart_speed;						// UART speed
	sem_t write_lock;							// Semaphore write lock
	sem_t read_lock;							// Semaphore read lock
	struct {
		uint16_t uart_datalen : 4;				// Uart data length
		uint16_t uart_num : 4;					// Uart device table number
		uint16_t uart_partity : 2;				// Uart parity  0 = none, 1= odd, 2 = even .. 3 invalid
		uint16_t uart_twostopbits : 1;			// Uart stop bits 0 = 1 stop bit , 1 = 2 stop bits
		uint16_t _reserved : 3;        			// reserved
		uint16_t uselocks : 1;					// Locks to be used for access
		uint16_t inuse : 1;						// In use flag
	};
};

/* Global table of UART devices.  */
static struct uart_device uarttab[NUART] = { 0 };

/*-[ UartOpenPort ]---------------------------------------------------------}
. Creates a UART handle which provides access to the UART device number.
. The UART device is setup to the bits, speed and mode provided.
. RETURN: valid UART_HANDLE for success, NULL for any failure
.--------------------------------------------------------------------------*/
UART_HANDLE UartOpenPort (uint8_t uart_devicenum, uint16_t baud, uint8_t datalen, uint8_t parity, uint8_t stop, bool useLock)
{
	UART_HANDLE uart = NULL;
	struct uart_device* uart_ptr = &uarttab[uart_devicenum];		// UART device pointer 
	if (uart_devicenum < NUART && uart_ptr->inuse == 0 && 
		baud > 0 && datalen >=5 && datalen <= 8 && parity <= 2 && 
		stop >= 1 && stop <= 2)										// Check UART parameters valid
	{
		char buf[256] = { 0 };
		sprintf(&buf[0], "/dev/ttyS%c", (char)(0x30 + uart_devicenum));
		int fd = open(&buf[0], O_RDWR | O_NOCTTY | O_NDELAY);
		if (fd >= 0)												// UART device opened correctly
		{
			uart_ptr->uart_fd = fd;									// Hold the file device to UART
			uart_ptr->uart_num = uart_devicenum;					// Hold uart device number
			uart_ptr->uart_speed = baud;							// Hold uart baud setting
			uart_ptr->uart_datalen = datalen;						// Hold uart data length
			uart_ptr->uart_partity = parity;						// Hold uart parity
			uart_ptr->uart_twostopbits = stop;						// Hold uart stop bit settings
			uart_ptr->uselocks = (useLock == true) ? 1 : 0;			// Set use lock
			if (useLock)                                            // Using locks
			{
				sem_init(&uart_ptr->write_lock, 0, 1);				// Initialize write mutex to 1
				sem_init(&uart_ptr->read_lock, 0, 1);				// Initialize read mutex to 1
			}
			//CONFIGURE THE UART
			//The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
			struct termios options;
			tcgetattr(uart_ptr->uart_fd, &options);
			switch (baud)
			{
				case 19200:
					cfsetospeed(&options, B19200);					// Set output baud
					cfsetispeed(&options, B19200);					// Set input baud
					break;
				case 38400:
					cfsetospeed(&options, B38400);					// Set output baud
					cfsetispeed(&options, B38400);					// Set input baud
					break;
				default:
					cfsetospeed(&options, B9600);					// Set output baud
					cfsetispeed(&options, B9600);					// Set input baud
					break;
			};
			options.c_cflag = (options.c_cflag & ~CSIZE);	// Clear data length flags
			switch (datalen)
			{
				case 5:
					options.c_cflag |= CS5;
					break;
				case 6:
					options.c_cflag |= CS6;
					break;
				case 7:
					options.c_cflag |= CS7;
					break;
				default:
					options.c_cflag |= CS8;
					break;
			};
			options.c_cflag |= (CLOCAL | CREAD);
			options.c_cflag &= ~(PARENB | PARODD);      // shut off parity
			switch (parity)
			{
			case 1:
				options.c_cflag |= (PARENB | PARODD);
				break;
			case 2:
				options.c_cflag |= PARENB;
				break;
			default:
				break;
			};
			options.c_cflag &= ~(CSTOPB);				// 1 stop bit
			if (stop == 2) options.c_cflag |= CSTOPB;
			options.c_iflag = IGNPAR;
			options.c_oflag = 0;
			options.c_lflag = 0;
			tcflush(uart_ptr->uart_fd, TCIFLUSH);
			tcsetattr(uart_ptr->uart_fd, TCSANOW, &options);
			uart_ptr->inuse = 1;
			uart = uart_ptr;
		}
	}
	return(uart);
}


/*-[ UartClosePort ]--------------------------------------------------------}
. Given a valid UART handle the access is released and the handle freed.
. RETURN: true for success, false for any failure
.--------------------------------------------------------------------------*/
bool UartClosePort (UART_HANDLE uartHandle)
{
	if (uartHandle && uartHandle->inuse)							// UART handle valid and UART device is in use
	{
		if (uartHandle->uselocks)									// Using locks
		{
			sem_destroy(&uartHandle->write_lock);					// Destroy write lock mutex
			sem_destroy(&uartHandle->read_lock);					// Destroy read lock mutex
		}
		close(uartHandle->uart_fd);									// Close the uart handle
		uartHandle->uart_fd = 0;									// Zero the uart file handle
		uartHandle->uart_num = 0;									// Zero uart handle number
		uartHandle->inuse = 0;										// The uart handle is now free
		return true;												// Return success
	}
	return false;
}

/*-[ UartWrite ]------------------------------------------------------------}
. Given a valid uart handle and valid data pointers the call will send data 
. length bytes from the buffer to UART.
. RETURN: >= 0 transfer count for success, < 0 for any error
.--------------------------------------------------------------------------*/
int UartWrite (UART_HANDLE uartHandle, uint8_t* TxData, uint16_t Length)
{
	int retVal = -1;												// Preset -1
	if (uartHandle && uartHandle->inuse)							// uart handle valid and uart device is in use
	{
		if (uartHandle->uselocks)									// Using locks
		{
			sem_wait(&uartHandle->write_lock);						// Take write semaphore
		}
		retVal = write(uartHandle->uart_fd, TxData, Length);		// Write the data to uart
		if (uartHandle->uselocks)									// Using locks
		{
			sem_post(&uartHandle->write_lock);					    // Give write semaphore
		}
	}
	return retVal;													// Return result
}

/*-[ UartRead ]-------------------------------------------------------------}
. Given a valid uart handle and valid data pointers the call will read data
. of length bytes from UART and place in buffer.
. RETURN: >= 0 transfer count for success, < 0 for any error
.--------------------------------------------------------------------------*/
int UartRead (UART_HANDLE uartHandle, uint8_t* RxData, uint16_t Length)
{
	int retVal = -1;												// Preset -1
	if (uartHandle && uartHandle->inuse)							// uart handle valid and uart device is in use
	{
		if (uartHandle->uselocks)									// Using locks
		{
			sem_wait(&uartHandle->read_lock);						// Take read semaphore
		}
		retVal = read(uartHandle->uart_fd, RxData, Length);			// Read data from uart
		if (uartHandle->uselocks)									// Using locks
		{
			sem_post(&uartHandle->read_lock);					    // Give read semaphore
		}
	}
	return retVal;
}
