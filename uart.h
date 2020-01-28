#ifndef _UART_H_
#define _UART_H_

#ifdef __cplusplus								// If we are including to a C++
extern "C" {									// Put extern C directive wrapper around
#endif

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{																			}
{       Filename: uart.h													}
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

#include <stdbool.h>							// C standard unit for bool, true, false
#include <stdint.h>								// C standard unit for uint32_t etc

#define UART_DRIVER_VERSION 1000						// Version number 1.00 build 0

typedef struct uart_device* UART_HANDLE;				// Define an UART_HANDLE

#define NUART 2			

/*-[ UartOpenPort ]---------------------------------------------------------}
. Creates a UART handle which provides access to the UART device number.
. The UART device is setup to the bits, speed and mode provided.
. RETURN: valid UART_HANDLE for success, NULL for any failure
.--------------------------------------------------------------------------*/
UART_HANDLE UartOpenPort (uint8_t uart_devicenum, uint16_t baud, uint8_t datalen, uint8_t parity, uint8_t stop, bool useLock);

/*-[ UartClosePort ]--------------------------------------------------------}
. Given a valid UART handle the access is released and the handle freed.
. RETURN: true for success, false for any failure
.--------------------------------------------------------------------------*/
bool UartClosePort (UART_HANDLE uartHandle);

/*-[ UartWrite ]------------------------------------------------------------}
. Given a valid uart handle and valid data pointers the call will send data
. length bytes from the buffer to UART.
. RETURN: >= 0 transfer count for success, < 0 for any error
.--------------------------------------------------------------------------*/
int UartWrite (UART_HANDLE uartHandle, uint8_t* TxData, uint16_t Length);

/*-[ UartRead ]-------------------------------------------------------------}
. Given a valid uart handle and valid data pointers the call will read data
. of length bytes from UART and place in buffer.
. RETURN: >= 0 transfer count for success, < 0 for any error
.--------------------------------------------------------------------------*/
int UartRead (UART_HANDLE uartHandle, uint8_t* RxData, uint16_t Length);

#ifdef __cplusplus								// If we are including to a C++ file
}												// Close the extern C directive wrapper
#endif

#endif