/**
  UART2 Generated Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    uart2.c

  @Summary
    This is the generated driver implementation file for the UART2 driver using PIC24 / dsPIC33 / PIC32MM MCUs

  @Description
    This header file provides implementations for driver APIs for UART2.
    Generation Information :
        Product Revision  :  PIC24 / dsPIC33 / PIC32MM MCUs - 1.75.1
        Device            :  PIC32MM0256GPM064
    The generated drivers are tested against the following:
        Compiler          :  XC32 v2.10
        MPLAB             :  MPLAB X v5.05
*/

/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/

/**
  Section: Included Files
*/

#include "uart2.h"
#include "usb/usb.h"
#include "pin_manager.h"
#include "../GenericTypeDefs.h"
#include "../Framework.h"
#include "../BootLoader.h"
/**
  Section: UART2 APIs
*/

static UINT8 TxBuff[255];
#ifdef __cplusplus
extern "C" {
#endif
BOOL GetChar(UINT8 *byte);
void PutChar(UINT8 txChar);
#ifdef __cplusplus
}
#endif

void UART2_Initialize(void)
{
    // Set the UART2 module to the options selected in the user interface.

    // STSEL 1; PDSEL 8N; RTSMD disabled; OVFDIS disabled; ACTIVE disabled; RXINV disabled; WAKE disabled; BRGH enabled; IREN disabled; ON enabled; SLPEN disabled; SIDL disabled; ABAUD disabled; LPBACK disabled; UEN TX_RX; CLKSEL PBCLK; 
    // Data Bits = 8; Parity = None; Stop Bits = 1;
    U2MODE = (0x8008 & ~(1<<15));  // disabling UART ON bit
    // UTXISEL TX_ONE_CHAR; UTXINV disabled; ADDR 0; MASK 0; URXEN disabled; OERR disabled; URXISEL RX_ONE_CHAR; UTXBRK disabled; UTXEN disabled; ADDEN disabled; 
    U2STA = 0x00;
    // BaudRate = 19200; Frequency = 24000000 Hz; BRG 312; 
    //57600
    U2BRG = 0x67;
     
    //Make sure to set LAT bit corresponding to TxPin as high before UART initialization
    U2STASET = _U2STA_UTXEN_MASK;
    U2MODESET = _U2MODE_ON_MASK;  // enabling UART ON bit
    U2STASET = _U2STA_URXEN_MASK;
}

static UINT8 readBuffer[64];
static UINT8 writeBuffer[64];
BOOL flagUART=FALSE;
BOOL flagUSB=FALSE;

void UartTask(void)
{
	UINT8 TxLen=0;
	UINT8 Rx;
	UINT8 *ptr;
	// Check any character is received.
	if(GetChar(&Rx))//if(GetChar(&Rx))
	{
		// Pass the bytes to frame work.
        flagUART=TRUE;
		FRAMEWORK_BuildRxFrame(&Rx, 1);
        
	}	
	
	ptr = TxBuff;
	// Get transmit frame from frame work.
	if(flagUART) {
        TxLen = FRAMEWORK_GetTransmitFrame(ptr);
    }
	
	if(TxLen)
	{
		// There is something to transmit.
	   	while(TxLen--)
	   	{
		   PutChar(*(ptr++));//PutChar(*(ptr++));
		} 
	} 
    
    
    
    if( USBGetDeviceState() < CONFIGURED_STATE )
    {
        return;
    }

    if( USBIsDeviceSuspended()== true )
    {
        return;
    }

    if( USBUSARTIsTxTrfReady() == true)
    {
        UINT8 i;
        UINT16 numBytesRead=0;
        
        numBytesRead = getsUSBUSART(readBuffer, sizeof(readBuffer));
        
        if(numBytesRead)  { 
        FRAMEWORK_BuildRxFrame(readBuffer, numBytesRead);  flagUSB=TRUE;  }
        
        numBytesRead=0;
        if(flagUSB) {numBytesRead = FRAMEWORK_GetTransmitFrame(writeBuffer);}      
        if(numBytesRead > 0)
           {
               putUSBUSART(writeBuffer,numBytesRead);
           }
    }

    CDCTxService();
	
	
}

BOOL GetChar(UINT8 *byte)
{
	BYTE dummy;
    
    RS485_SetLow();
    
	if(U2STA & 0x000E)              // receive errors?
	{
		dummy = U2RXREG; 			// dummy read to clear FERR/PERR
		U2STAbits.OERR = 0;			// clear OERR to keep receiving
	}

	if(U2STAbits.URXDA)
	{
		*byte = U2RXREG;		        // get data from UART RX FIFO
		//delay_us(400);
        return TRUE;
	}
	
	return FALSE;

}

void PutChar(UINT8 txChar)
{
    RS485_SetHigh();
    while(U2STAbits.UTXBF); // wait for TX buffer to be empty
    U2TXREG = txChar;
    delay_us(200);
}

