/*********************************************************************
 *
 *                  PIC32 Boot Loader
 *
 **********************************************************************/

#include "GenericTypeDefs.h"
#include "BootLoader.h"
#include "Framework.h"
#include "NVMem.h"
#include "mcc_generated_files/uart2.h"
#include  <string.h>
#include "mcc_generated_files/pin_manager.h"
#include "mcc_generated_files/tmr1.h"

#include <xc.h>

#define PA_TO_KVA1(pa)    ((uint32_t)(pa) | 0xA0000000)
#define PA_TO_KVA0(pa)    ((uint32_t)(pa) | 0x80000000)

#define DATA_RECORD 		0
#define END_OF_FILE_RECORD 	1
#define EXT_SEG_ADRS_RECORD 2
#define EXT_LIN_ADRS_RECORD 4

    #define FLASH_PAGE_SIZE		 		2048
    #define FLASH_ROW_SIZE		 		256
    #define DEV_CONFIG_REG_BASE_ADDRESS 0x9FC01700
    #define DEV_CONFIG_REG_END_ADDRESS  0x9FC017FF
    //#warning VERIFICAR ESTES DEFINES 

typedef enum
{	ERROR_ACK 	  = 0x00,
	ERROR_ERR 	  = 0x01,
	ERROR_WRITEOK = 0x02
}error_t;
typedef enum
{	CMD_RESET	 	= 0x01,
	CMD_READ_ID1 	= 0x02,
	CMD_READ_ID2 	= 0x03,
	CMD_WRITE_INIT	= 0x04,
	CMD_WRITE_DATA	= 0x05
}cmd_t;
typedef enum
{	FIELD_STA	= 0,
	FIELD_LEN	= 1,
	FIELD_CMD	= 2,
	FIELD_ERR	= 2,
	FIELD_NUM	= 3,
	FIELD_INIT	= 3,
	FIELD_DATAW	= 3,
	FIELD_DATAR	= 4
}field_t;

typedef struct
{
	UINT Len;
	UINT8 Data[FRAMEWORK_BUFF_SIZE];
	
}T_FRAME;

typedef struct 
{
	UINT8 RecDataLen;
	DWORD_VAL Address;
	UINT8 RecType;
	UINT8* Data;
	UINT8 CheckSum;	
	DWORD_VAL ExtSegAddress;
	DWORD_VAL ExtLinAddress;
}T_HEX_RECORD;	


static const UINT8 BootInfo[32] =
{
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20	
};


static T_FRAME RxBuff;
static T_FRAME TxBuff;
static BOOL RxFrameValid;
static BOOL TriggerBaudChange;
static DWORD_VAL NewBaud;
static BOOL RunApplication = FALSE;

void HandleCommand(void);
void BuildRxFrame(UINT8 *RxData, INT16 RxLen);
UINT GetTransmitFrame(UINT8* Buff);
void WriteHexRecord2Flash(UINT8* HexRecord, UINT totalRecLen);
void WriteData2Flash(UINT8* DataPack, UINT lenDataPack);
void EraseToEnd(void); 
BOOL CheckValidTags(void);

BOOL BaudRateChangeRequested(void);
UINT16 CalculateCrc(UINT8 *data, UINT32 len);
	
extern void appFunction(void);

/********************************************************************
* Function: 	FrameWorkTask()
*
* Precondition: 
*
* Input: 		None.
*
* Output:		None.
*
* Side Effects:	None.
*
* Overview: 	Process the command if there is a valid fame.
*
*			
* Note:		 	None.
********************************************************************/
void FrameWorkTask(void)
{

	if(RxFrameValid)
	{
		// Valid frame received, process the command.
		HandleCommand();	
		// Reset the flag.
		RxFrameValid = FALSE;			
	}        
}


/********************************************************************
* Function: 	HandleCommand()
*
* Precondition: 
*
* Input: 		None.
*
* Output:		None.
*
* Side Effects:	None.
*
* Overview: 	Process the received frame and take action depending on
				the command received.
*
*			
* Note:		 	None.
********************************************************************/

int myvar;
int sendTXUart;
extern unsigned int addRead;

static BOOL flagFirst = TRUE; 
static BOOL flagWriteLest = FALSE;  //flag show is it lest data was writen

void HandleCommand(void)
{
	UINT8 Cmd;
	UINT8 i;
	UINT Result;
	
    // Reset the response length to 0.
	TxBuff.Len = 0;
	// First byte of the data field is command.
	Cmd = RxBuff.Data[1];
	// Partially build response frame. First byte in the data field carries command.
    TxBuff.Data[0] = SYM_STA; //
    TxBuff.Data[1] = 0x03; //len of response always 0x03
	TxBuff.Len += 2;  
  	
	// Process the command.		
	switch(Cmd)
	{
		case CMD_READ_ID1: // Read boot loader version info.
			memcpy(&TxBuff.Data[2], BootInfo, 32);
			TxBuff.Len += 32; // Boot Info Fields
            TMR1_Stop();
			break;
         	   
		case CMD_READ_ID2:
            memcpy(&TxBuff.Data[2], BootInfo, 32);
			TxBuff.Len += 32; // Boot Info Fields
			break;
            
		case CMD_WRITE_INIT:
            //BootWriteDone_Erase
            //BootJumpDone_Erase
            Result = NVMemErasePage((void *)VALID_APP_START_ADRESSS );
            
            WriteData2Flash(&RxBuff.Data[2],RxBuff.Len-4); //-len -cmd -crch -crcl
            TxBuff.Data[2] = ERROR_ACK;
            TxBuff.Len +=1 ;  
            break;
		
		case CMD_WRITE_DATA:
		    WriteData2Flash(&RxBuff.Data[3],RxBuff.Len-5); //-len -cmd -nPack -crch -crcl
            if(flagWriteLest) {
                  TxBuff.Data[2] = ERROR_WRITEOK;
                  flagWriteLest = FALSE; 
                  EraseToEnd(); 
                  
                  //BootWriteDone
                  Result=NVMemWriteWord((void *)VALID_BOOT_WRITE_END_ADRESSS, VALID_BOOT_WRITE_END_TAG, 0xffffffff);
            }
            else {
                 TxBuff.Data[2] = ERROR_ACK; //len of response always 0x03                 
            }
            TxBuff.Len +=1 ;   
		   	break;
    
	    case CMD_RESET:
	    	// Exit firmware upgrade mode.
            
            //BootJumpDone
            Result=NVMemWriteWord((void *)VALID_BOOT_JUMP_APP_ADRESSS, VALID_BOOT_JUMP_APP_TAG, 0xffffffff);
           
            RunApplication = CheckValidTags();
           	break;
	    default:
	    	// Nothing to do.
	    	break;
	}   
}


/********************************************************************
* Function: 	BuildRxFrame()
*
* Precondition: 
*
* Input: 		Pointer to Rx Data and Rx byte length.
*
* Output:		None.
*
* Side Effects:	None.
*
* Overview: 	Builds rx frame and checks CRC.
*
*			
* Note:		 	None.
********************************************************************/
void BuildRxFrame(UINT8 *RxData, INT16 RxLen)
{
	static UINT8 state = 0;
    static uint32_t cnt =0;
    int i=0;
	WORD_VAL crcRec;
    WORD_VAL crcCalc;

	while((RxLen > 0) && (!RxFrameValid)) // Loop till len = 0 or till frame is valid
	{
		RxLen--;  
        switch(state)
		{
			case 0: //Start of header
				if(*RxData==SYM_STA)
				{ state++; }			
				break;                
            case 1:
                cnt = *RxData;
                RxBuff.Len=0;
                RxBuff.Data[RxBuff.Len++] = cnt;
                state++;
            	break;
            case 2:  
                RxBuff.Data[RxBuff.Len++] = *RxData;
                if(RxBuff.Len>cnt) { 
                    state=0; 
                    cnt=0;
                    crcRec.byte.HB=RxBuff.Data[(RxBuff.Len-1)];
                    crcRec.byte.LB=RxBuff.Data[(RxBuff.Len-2)];
                    crcCalc.Val = CalculateCrc(RxBuff.Data, (UINT32)RxBuff.Len-2);
                        if(crcRec.Val==crcCalc.Val) {
                            state=0;
                            RxFrameValid = TRUE;
                            break;
                        }
                        else {
                            TxBuff.Data[0] = SYM_STA; //
                            TxBuff.Data[1] = 0x03;
                            TxBuff.Data[2] = ERROR_ERR; //len of response always 0x03
                            TxBuff.Len = 3;                                 
                         }
                }   
                break;
			default:                  
				break;				
		}
		//Increment the pointer.
		RxData++;		
	}		
}	

/********************************************************************
* Function: 	GetTransmitFrame()
*
* Precondition: 
*
* Input: 		Buffer pointer.
*
* Output:		Length of the buffer.
*
* Side Effects:	None.
*
* Overview: 	Gets the complete transmit frame into the "Buff".
*
*			
* Note:		 	None.
********************************************************************/
UINT GetTransmitFrame(UINT8* Buff)
{
	INT BuffLen = 0;
	WORD_VAL crc;
	UINT8 i;
	
	if(TxBuff.Len) 
	{
		//There is something to transmit.
		// Calculate CRC of the frame.
		crc.Val = CalculateCrc(TxBuff.Data, (UINT32)TxBuff.Len);
		TxBuff.Data[TxBuff.Len++] = crc.byte.LB;
		TxBuff.Data[TxBuff.Len++] = crc.byte.HB; 	
		
		// Insert Data Link Escape Character.
		for(i = 0; i < TxBuff.Len; i++)
		{		
			Buff[BuffLen++] = TxBuff.Data[i];
		} 
	
		TxBuff.Len = 0; // Purge this buffer, no more required.
	}	
	return(BuffLen); // Return buffer length.
}	

/********************************************************************
* Function: 	WriteHexRecord2Flash()
*
* Precondition: 
*
* Input: 		HexRecord buffer.
*
* Output:		None.
*
* Side Effects:	None.
*
* Overview:     Writes hex record to flash.
*
*			
* Note:		 	None.
********************************************************************/
unsigned int WrData;
static unsigned char __attribute__((address(0xA0003800), persistent)) rowSizeProg[256];
static unsigned int flagFirsOnce = 0;
static unsigned int flagProg = 0;
static unsigned int addressProgInit = 0x9D000000;//USER_APP_RESET_ADDRESS;
unsigned int * pAdMemRam;
static unsigned char countDataProg=0;
static unsigned int flagDataNotProg = 0;


void WriteData2Flash(UINT8* DataPack, UINT lenDataPack) //lenDataPack = 128 0x80
{
    UINT i=0;
    UINT y=0;
    UINT FF=0xFF;
	void* ProgAddress;
	UINT Result;
	static UINT numBytesLeft = 0; 
    static UINT countData=0; 
    static UINT8 rowCnt=8;
    
    numBytesLeft=lenDataPack+countData; 
    
    ProgAddress = (void *)addressProgInit;
    
    //if(((ProgAddress >= (void *)APP_FLASH_BASE_ADDRESS) && (ProgAddress <= (void *)APP_FLASH_END_ADDRESS))
    if(((ProgAddress >= 0x9D000000) && (ProgAddress <= (void *)APP_FLASH_END_ADDRESS))
	&& ((ProgAddress < (void*)DEV_CONFIG_REG_BASE_ADDRESS) || (ProgAddress > (void*)DEV_CONFIG_REG_END_ADDRESS)))
    
    {
        do{
         memcpy(&WrData, &DataPack[i++], 1); 
         rowSizeProg[countData++] = WrData;
         
              if(rowCnt==(FLASH_PAGE_SIZE/FLASH_ROW_SIZE)){
                   rowCnt=0;
                   Result = NVMemErasePage((void *)addressProgInit);
                   ASSERT(Result==0);
                 }

             if (countData==FLASH_ROW_SIZE){
                 Result = NVMemWriteRow((void *)addressProgInit,(void*)rowSizeProg);
                 ASSERT(Result==0);

                 countData=0; //cnt rowBuff 
                 addressProgInit+=FLASH_ROW_SIZE; //row addres +
                 numBytesLeft -=FLASH_ROW_SIZE; // rec data left => rowBuff
                     for(y=0;y<FLASH_ROW_SIZE;y++) { //buff=0xff
                         memcpy(&WrData, &FF, 1);
                         rowSizeProg[y] = WrData;
                         }
                 //flag about first write data
                 flagFirst=FALSE;   
                 rowCnt++;
                   
                 }     

         }while(countData<numBytesLeft);

             if((lenDataPack<128)&&!flagFirst){

                 Result = NVMemWriteRow((void *)addressProgInit,(void*)rowSizeProg);
                 ASSERT(Result==0);

                 countData=0; //cnt rowBuff 
                 //addressProgInit=USER_APP_RESET_ADDRESS; //row addres +
                 numBytesLeft = 0; // rec data left => rowBuff

                 flagWriteLest=TRUE;                
             } 
   }        
}   

void EraseToEnd(void){
    UINT Result;
    UINT i;									
			for( i = addressProgInit/FLASH_PAGE_SIZE; i < (0x30000/FLASH_PAGE_SIZE); i++ ) ///0x2F800
			{
				Result = NVMemErasePage((void *)0x9D000000 + (i*FLASH_PAGE_SIZE) );
				// Assert on NV error. This must be caught during debug phase.
				ASSERT(Result==0);
			}	
}

void WriteFlag(){
    UINT Result;
   // Result = NVMemWriteWord((void*)VALID_BOOT_WRITE_APP_ADRESSS, 0x1234, 0x8888);
    ASSERT(Result==0);
}

/**
 * Static table used for the table_driven implementation.
 *****************************************************************************/
static const UINT16 crc_table[16] = 
{
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef
};

/********************************************************************
* Function: 	CalculateCrc()
*
* Precondition: 
*
* Input: 		Data pointer and data length
*
* Output:		CRC.
*
* Side Effects:	None.
*
* Overview:     Calculates CRC for the given data and len
*
*			
* Note:		 	None.
********************************************************************/	
//UINT16 CalculateCrc(UINT8 *data, UINT32 len)
//{
//    UINT i;
//    UINT16 crc = 0;
//    
//    while(len--)
//    {
//        i = (crc >> 12) ^ (*data >> 4);
//	    crc = crc_table[i & 0x0F] ^ (crc << 4);
//	    i = (crc >> 12) ^ (*data >> 0);
//	    crc = crc_table[i & 0x0F] ^ (crc << 4);
//	    data++;
//	} 
//
//    return (crc & 0xFFFF);
//}

UINT16 CalculateCrc(UINT8 *data, UINT32 len)
{
    UINT16 crc=0xffff;
    int i;
    if (!data || len < 0)
        return crc;

    while (len--) {
        crc ^= *data++;
        for (i=0; i<8; i++) {
            if (crc & 1)  crc = (crc >> 1) ^ 0x8408;
            else          crc = (crc >> 1);
        }
    }
    return (crc & 0xFFFF);
}

/********************************************************************
* Function: 	ExitFirmwareUpgradeMode()
*
* Precondition: 
*
* Input: 		Void
*
* Output:		True if firmware upgrade mode has to be exited.
*
* Side Effects:	None.
*
* Overview:     This function returns true if firmware mode has to be exited.
*
*			
* Note:		 	None.
********************************************************************/
BOOL ExitFirmwareUpgradeMode(void)
{
	return RunApplication;
}	




/**************************End of file**************************************************/

