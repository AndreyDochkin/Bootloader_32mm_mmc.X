/**
  System Interrupts Generated Driver File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.h

  @Summary:
    This is the generated manager file for the MPLAB(c) Code Configurator device.  This manager
    configures the pins direction, initial state, analog setting.
    The peripheral pin select, PPS, configuration is also handled by this manager.

  @Description:
    This source file provides implementations for MPLAB(c) Code Configurator interrupts.
    Generation Information :
        Product Revision  :  MPLAB(c) Code Configurator - pic24-dspic-pic32mm : 1.75.1
        Device            :  PIC32MM0256GPM064
    The generated drivers are tested against the following:
        Compiler          :  XC32 v2.10
        MPLAB             :  MPLAB X v5.05

    Copyright (c) 2013 - 2015 released Microchip Technology Inc.  All rights reserved.

    Microchip licenses to you the right to use, modify, copy and distribute
    Software only when embedded on a Microchip microcontroller or digital signal
    controller that is integrated into your product or third party product
    (pursuant to the sublicense terms in the accompanying license agreement).

    You should refer to the license agreement accompanying this Software for
    additional information regarding your rights and obligations.

    SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
    EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
    MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
    IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
    CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
    OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
    INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
    CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
    SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
    (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

*/

#ifndef _PIN_MANAGER_H
#define _PIN_MANAGER_H
/**
    Section: Includes
*/
#include <xc.h>
#include <stdbool.h>
/**
    Section: Device Pin Macros
*/
/**
  @Summary
    Sets the GPIO pin, RA0, high using LATAbits.LATA0.

  @Description
    Sets the GPIO pin, RA0, high using LATAbits.LATA0.

  @Preconditions
    The RA0 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RA0 high (1)
    IO_RA0_SetHigh();
    </code>

*/
#define IO_RA0_SetHigh()          ( LATASET = (1 << 0) )
/**
  @Summary
    Sets the GPIO pin, RA0, low using LATAbits.LATA0.

  @Description
    Sets the GPIO pin, RA0, low using LATAbits.LATA0.

  @Preconditions
    The RA0 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RA0 low (0)
    IO_RA0_SetLow();
    </code>

*/
#define IO_RA0_SetLow()           ( LATACLR = (1 << 0) )

/**
  @Summary
    Sets a value to the GPIO pin.

  @Description
    Sets or Resets the GPIO pin, RA0, low or high using LATAbits.LATA0.

  @Preconditions
    The RA0 must be set to an output.

  @Returns
    None.

  @Param
    bool value; : value to be set to the GPIO pin.

  @Example
    <code>
    // Set RA0 to low.
    IO_RA0_SetValue(false);
    </code>

*/
inline static void IO_RA0_SetValue(bool value)
{
  if(value)
  {
    IO_RA0_SetHigh();
  }
  else
  {
    IO_RA0_SetLow();
  }
}

/**
  @Summary
    Toggles the GPIO pin, RA0, using LATAbits.LATA0.

  @Description
    Toggles the GPIO pin, RA0, using LATAbits.LATA0.

  @Preconditions
    The RA0 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RA0
    IO_RA0_Toggle();
    </code>

*/
#define IO_RA0_Toggle()           ( LATAINV = (1 << 0) )
/**
  @Summary
    Reads the value of the GPIO pin, RA0.

  @Description
    Reads the value of the GPIO pin, RA0.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RA0
    postValue = IO_RA0_GetValue();
    </code>

*/
#define IO_RA0_GetValue()         PORTAbits.RA0
/**
  @Summary
    Configures the GPIO pin, RA0, as an input.

  @Description
    Configures the GPIO pin, RA0, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RA0 as an input
    IO_RA0_SetDigitalInput();
    </code>

*/
#define IO_RA0_SetDigitalInput()   ( TRISASET = (1 << 0) )
/**
  @Summary
    Configures the GPIO pin, RA0, as an output.

  @Description
    Configures the GPIO pin, RA0, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RA0 as an output
    IO_RA0_SetDigitalOutput();
    </code>

*/
#define IO_RA0_SetDigitalOutput()   ( TRISACLR = (1 << 0) )
/**
  @Summary
    Sets the GPIO pin, RC0, high using LATCbits.LATC0.

  @Description
    Sets the GPIO pin, RC0, high using LATCbits.LATC0.

  @Preconditions
    The RC0 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC0 high (1)
    RS485_SetHigh();
    </code>

*/
#define RS485_SetHigh()          ( LATCSET = (1 << 0) )
/**
  @Summary
    Sets the GPIO pin, RC0, low using LATCbits.LATC0.

  @Description
    Sets the GPIO pin, RC0, low using LATCbits.LATC0.

  @Preconditions
    The RC0 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC0 low (0)
    RS485_SetLow();
    </code>

*/
#define RS485_SetLow()           ( LATCCLR = (1 << 0) )

/**
  @Summary
    Sets a value to the GPIO pin.

  @Description
    Sets or Resets the GPIO pin, RC0, low or high using LATCbits.LATC0.

  @Preconditions
    The RC0 must be set to an output.

  @Returns
    None.

  @Param
    bool value; : value to be set to the GPIO pin.

  @Example
    <code>
    // Set RC0 to low.
    RS485_SetValue(false);
    </code>

*/
inline static void RS485_SetValue(bool value)
{
  if(value)
  {
    RS485_SetHigh();
  }
  else
  {
    RS485_SetLow();
  }
}

/**
  @Summary
    Toggles the GPIO pin, RC0, using LATCbits.LATC0.

  @Description
    Toggles the GPIO pin, RC0, using LATCbits.LATC0.

  @Preconditions
    The RC0 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RC0
    RS485_Toggle();
    </code>

*/
#define RS485_Toggle()           ( LATCINV = (1 << 0) )
/**
  @Summary
    Reads the value of the GPIO pin, RC0.

  @Description
    Reads the value of the GPIO pin, RC0.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RC0
    postValue = RS485_GetValue();
    </code>

*/
#define RS485_GetValue()         PORTCbits.RC0
/**
  @Summary
    Configures the GPIO pin, RC0, as an input.

  @Description
    Configures the GPIO pin, RC0, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC0 as an input
    RS485_SetDigitalInput();
    </code>

*/
#define RS485_SetDigitalInput()   ( TRISCSET = (1 << 0) )
/**
  @Summary
    Configures the GPIO pin, RC0, as an output.

  @Description
    Configures the GPIO pin, RC0, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC0 as an output
    RS485_SetDigitalOutput();
    </code>

*/
#define RS485_SetDigitalOutput()   ( TRISCCLR = (1 << 0) )
/**
  @Summary
    Sets the GPIO pin, RC1, high using LATCbits.LATC1.

  @Description
    Sets the GPIO pin, RC1, high using LATCbits.LATC1.

  @Preconditions
    The RC1 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC1 high (1)
    SA1_SetHigh();
    </code>

*/
#define SA1_SetHigh()          ( LATCSET = (1 << 1) )
/**
  @Summary
    Sets the GPIO pin, RC1, low using LATCbits.LATC1.

  @Description
    Sets the GPIO pin, RC1, low using LATCbits.LATC1.

  @Preconditions
    The RC1 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC1 low (0)
    SA1_SetLow();
    </code>

*/
#define SA1_SetLow()           ( LATCCLR = (1 << 1) )

/**
  @Summary
    Sets a value to the GPIO pin.

  @Description
    Sets or Resets the GPIO pin, RC1, low or high using LATCbits.LATC1.

  @Preconditions
    The RC1 must be set to an output.

  @Returns
    None.

  @Param
    bool value; : value to be set to the GPIO pin.

  @Example
    <code>
    // Set RC1 to low.
    SA1_SetValue(false);
    </code>

*/
inline static void SA1_SetValue(bool value)
{
  if(value)
  {
    SA1_SetHigh();
  }
  else
  {
    SA1_SetLow();
  }
}

/**
  @Summary
    Toggles the GPIO pin, RC1, using LATCbits.LATC1.

  @Description
    Toggles the GPIO pin, RC1, using LATCbits.LATC1.

  @Preconditions
    The RC1 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RC1
    SA1_Toggle();
    </code>

*/
#define SA1_Toggle()           ( LATCINV = (1 << 1) )
/**
  @Summary
    Reads the value of the GPIO pin, RC1.

  @Description
    Reads the value of the GPIO pin, RC1.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RC1
    postValue = SA1_GetValue();
    </code>

*/
#define SA1_GetValue()         PORTCbits.RC1
/**
  @Summary
    Configures the GPIO pin, RC1, as an input.

  @Description
    Configures the GPIO pin, RC1, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC1 as an input
    SA1_SetDigitalInput();
    </code>

*/
#define SA1_SetDigitalInput()   ( TRISCSET = (1 << 1) )
/**
  @Summary
    Configures the GPIO pin, RC1, as an output.

  @Description
    Configures the GPIO pin, RC1, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC1 as an output
    SA1_SetDigitalOutput();
    </code>

*/
#define SA1_SetDigitalOutput()   ( TRISCCLR = (1 << 1) )
/**
  @Summary
    Sets the GPIO pin, RC3, high using LATCbits.LATC3.

  @Description
    Sets the GPIO pin, RC3, high using LATCbits.LATC3.

  @Preconditions
    The RC3 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC3 high (1)
    LED1_SetHigh();
    </code>

*/
#define LED1_SetHigh()          ( LATCSET = (1 << 3) )
/**
  @Summary
    Sets the GPIO pin, RC3, low using LATCbits.LATC3.

  @Description
    Sets the GPIO pin, RC3, low using LATCbits.LATC3.

  @Preconditions
    The RC3 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC3 low (0)
    LED1_SetLow();
    </code>

*/
#define LED1_SetLow()           ( LATCCLR = (1 << 3) )

/**
  @Summary
    Sets a value to the GPIO pin.

  @Description
    Sets or Resets the GPIO pin, RC3, low or high using LATCbits.LATC3.

  @Preconditions
    The RC3 must be set to an output.

  @Returns
    None.

  @Param
    bool value; : value to be set to the GPIO pin.

  @Example
    <code>
    // Set RC3 to low.
    LED1_SetValue(false);
    </code>

*/
inline static void LED1_SetValue(bool value)
{
  if(value)
  {
    LED1_SetHigh();
  }
  else
  {
    LED1_SetLow();
  }
}

/**
  @Summary
    Toggles the GPIO pin, RC3, using LATCbits.LATC3.

  @Description
    Toggles the GPIO pin, RC3, using LATCbits.LATC3.

  @Preconditions
    The RC3 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RC3
    LED1_Toggle();
    </code>

*/
#define LED1_Toggle()           ( LATCINV = (1 << 3) )
/**
  @Summary
    Reads the value of the GPIO pin, RC3.

  @Description
    Reads the value of the GPIO pin, RC3.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RC3
    postValue = LED1_GetValue();
    </code>

*/
#define LED1_GetValue()         PORTCbits.RC3
/**
  @Summary
    Configures the GPIO pin, RC3, as an input.

  @Description
    Configures the GPIO pin, RC3, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC3 as an input
    LED1_SetDigitalInput();
    </code>

*/
#define LED1_SetDigitalInput()   ( TRISCSET = (1 << 3) )
/**
  @Summary
    Configures the GPIO pin, RC3, as an output.

  @Description
    Configures the GPIO pin, RC3, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC3 as an output
    LED1_SetDigitalOutput();
    </code>

*/
#define LED1_SetDigitalOutput()   ( TRISCCLR = (1 << 3) )
/**
  @Summary
    Sets the GPIO pin, RC4, high using LATCbits.LATC4.

  @Description
    Sets the GPIO pin, RC4, high using LATCbits.LATC4.

  @Preconditions
    The RC4 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC4 high (1)
    LED2_SetHigh();
    </code>

*/
#define LED2_SetHigh()          ( LATCSET = (1 << 4) )
/**
  @Summary
    Sets the GPIO pin, RC4, low using LATCbits.LATC4.

  @Description
    Sets the GPIO pin, RC4, low using LATCbits.LATC4.

  @Preconditions
    The RC4 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC4 low (0)
    LED2_SetLow();
    </code>

*/
#define LED2_SetLow()           ( LATCCLR = (1 << 4) )

/**
  @Summary
    Sets a value to the GPIO pin.

  @Description
    Sets or Resets the GPIO pin, RC4, low or high using LATCbits.LATC4.

  @Preconditions
    The RC4 must be set to an output.

  @Returns
    None.

  @Param
    bool value; : value to be set to the GPIO pin.

  @Example
    <code>
    // Set RC4 to low.
    LED2_SetValue(false);
    </code>

*/
inline static void LED2_SetValue(bool value)
{
  if(value)
  {
    LED2_SetHigh();
  }
  else
  {
    LED2_SetLow();
  }
}

/**
  @Summary
    Toggles the GPIO pin, RC4, using LATCbits.LATC4.

  @Description
    Toggles the GPIO pin, RC4, using LATCbits.LATC4.

  @Preconditions
    The RC4 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RC4
    LED2_Toggle();
    </code>

*/
#define LED2_Toggle()           ( LATCINV = (1 << 4) )
/**
  @Summary
    Reads the value of the GPIO pin, RC4.

  @Description
    Reads the value of the GPIO pin, RC4.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RC4
    postValue = LED2_GetValue();
    </code>

*/
#define LED2_GetValue()         PORTCbits.RC4
/**
  @Summary
    Configures the GPIO pin, RC4, as an input.

  @Description
    Configures the GPIO pin, RC4, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC4 as an input
    LED2_SetDigitalInput();
    </code>

*/
#define LED2_SetDigitalInput()   ( TRISCSET = (1 << 4) )
/**
  @Summary
    Configures the GPIO pin, RC4, as an output.

  @Description
    Configures the GPIO pin, RC4, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC4 as an output
    LED2_SetDigitalOutput();
    </code>

*/
#define LED2_SetDigitalOutput()   ( TRISCCLR = (1 << 4) )
/**
  @Summary
    Sets the GPIO pin, RC5, high using LATCbits.LATC5.

  @Description
    Sets the GPIO pin, RC5, high using LATCbits.LATC5.

  @Preconditions
    The RC5 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC5 high (1)
    LED3_SetHigh();
    </code>

*/
#define LED3_SetHigh()          ( LATCSET = (1 << 5) )
/**
  @Summary
    Sets the GPIO pin, RC5, low using LATCbits.LATC5.

  @Description
    Sets the GPIO pin, RC5, low using LATCbits.LATC5.

  @Preconditions
    The RC5 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC5 low (0)
    LED3_SetLow();
    </code>

*/
#define LED3_SetLow()           ( LATCCLR = (1 << 5) )

/**
  @Summary
    Sets a value to the GPIO pin.

  @Description
    Sets or Resets the GPIO pin, RC5, low or high using LATCbits.LATC5.

  @Preconditions
    The RC5 must be set to an output.

  @Returns
    None.

  @Param
    bool value; : value to be set to the GPIO pin.

  @Example
    <code>
    // Set RC5 to low.
    LED3_SetValue(false);
    </code>

*/
inline static void LED3_SetValue(bool value)
{
  if(value)
  {
    LED3_SetHigh();
  }
  else
  {
    LED3_SetLow();
  }
}

/**
  @Summary
    Toggles the GPIO pin, RC5, using LATCbits.LATC5.

  @Description
    Toggles the GPIO pin, RC5, using LATCbits.LATC5.

  @Preconditions
    The RC5 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RC5
    LED3_Toggle();
    </code>

*/
#define LED3_Toggle()           ( LATCINV = (1 << 5) )
/**
  @Summary
    Reads the value of the GPIO pin, RC5.

  @Description
    Reads the value of the GPIO pin, RC5.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RC5
    postValue = LED3_GetValue();
    </code>

*/
#define LED3_GetValue()         PORTCbits.RC5
/**
  @Summary
    Configures the GPIO pin, RC5, as an input.

  @Description
    Configures the GPIO pin, RC5, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC5 as an input
    LED3_SetDigitalInput();
    </code>

*/
#define LED3_SetDigitalInput()   ( TRISCSET = (1 << 5) )
/**
  @Summary
    Configures the GPIO pin, RC5, as an output.

  @Description
    Configures the GPIO pin, RC5, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC5 as an output
    LED3_SetDigitalOutput();
    </code>

*/
#define LED3_SetDigitalOutput()   ( TRISCCLR = (1 << 5) )

/**
    Section: Function Prototypes
*/
/**
  @Summary
    Configures the pin settings of the PIC32MM0256GPM064
    The peripheral pin select, PPS, configuration is also handled by this manager.

  @Description
    This is the generated manager file for the MPLAB(c) Code Configurator device.  This manager
    configures the pins direction, initial state, analog setting.
    The peripheral pin select, PPS, configuration is also handled by this manager.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    void SYSTEM_Initialize(void)
    {
        // Other initializers are called from this function
        PIN_MANAGER_Initialize();
    }
    </code>

*/
void PIN_MANAGER_Initialize(void);

#endif
