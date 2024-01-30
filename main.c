#include "mcc_generated_files/system.h"
#include "mcc_generated_files/pin_manager.h"
#include "mcc_generated_files/uart2.h"
#include "mcc_generated_files/usb/usb.h"
#include "GenericTypeDefs.h"
#include "Framework.h"
#include "BootLoader.h"
#include "mcc_generated_files/interrupt_manager.h"
#include <string.h>

// Macros
#define F_CPU 24000000UL
#define BlinkLED() LED3_SetValue((_CP0_GET_COUNT() & 0x00110000) == 0)
#define TIMER_3S 3
#define TIMER_30S 30

// Function prototypes
BOOL CheckTrigger(void);
void JumpToApp(void);
BOOL ValidAppEmpty(void);
BOOL CheckValidTags(void);
BOOL CheckValidApp(void);

// Global variables
BOOL FlagTMR = FALSE; // Flag for 3s pass
UINT8 SecCnt = 0;     // Second count

// Timer1 callback function
void __attribute__((weak)) TMR1_CallBack(void)
{
    static int cnt = 0;
    cnt++;
    if (cnt > SecCnt)
    {
        FlagTMR = (CheckValidTags() && CheckValidApp());
    }
}

int main(void)
{
    // Initialize the device
    SYSTEM_Initialize();
    SA1_SetDigitalInput();
    RS485_SetDigitalOutput();
    TMR1_Start();

    SecCnt = CheckTrigger() ? TIMER_30S : TIMER_3S;

    while (!FlagTMR && !FRAMEWORK_ExitFirmwareUpgradeMode())
    {
        TRANS_LAYER_Task();      // Run Transport layer tasks
        FRAMEWORK_FrameWorkTask(); // Run framework-related tasks
        BlinkLED();               // Blink LED to indicate the bootloader is running
    }

    USBDeviceDetach();
    TMR1_Stop();
    INTERRUPT_GlobalDisable();
    __asm__("J 0x9D005000"); // Jump to the application code
    return 0;
}

BOOL CheckTrigger(void)
{
    return !SA1_GetValue();
}

void JumpToApp(void)
{
    void (*fptr)(void);
    fptr = (void (*)(void))(0x1D005000 + 1); // Add 1 to the address (MM devices)
    fptr();
}

BOOL ValidAppEmpty(void)
{
    UINT32 *AppPtr;
    AppPtr = (UINT32 *)USER_APP_RESET_ADDRESS;
    return *AppPtr == 0xFFFFFFFF;
}

BOOL CheckValidTags(void)
{
    UINT32 *BWEPtr;
    UINT32 *BJAPtr;
    BWEPtr = (UINT32 *)VALID_BOOT_WRITE_END_ADRESSS;
    BJAPtr = (UINT32 *)VALID_BOOT_JUMP_APP_ADRESSS;
    return (*BWEPtr == VALID_BOOT_WRITE_END_TAG) && (*BJAPtr == VALID_BOOT_JUMP_APP_TAG);
}

BOOL CheckValidApp(void)
{
    UINT32 *ASVPtr;
    ASVPtr = (UINT32 *)VALID_APP_START_ADRESSS;
    return *ASVPtr == VALID_APP_START_TAG;
}
