//Can Bus Simulator
#include "c8051F040.h"
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
#include "main.h"
#include "spi.h"

extern CANFRAME xdata OneTime[TXBUFFS];
extern CANFRAME xdata rxframe0[RX0BUFFERS];
extern U8 xdata MaxMesgs,MaxRxTraps;

extern MSGTABLE xdata *Mesgs;
extern U8 xdata MesgCnt;
extern BUTTABLE xdata *Buttons;
extern U8 xdata ButtCnt;
extern RxMESG xdata *RxMesg;
extern U8 xdata RxCnt;
extern U16 xdata RelayCnf[4];
extern U8 xdata RelayChange;
extern U8 xdata RxBuffCnt[2];

extern bit IGN_OFF_FLG;

extern U32 keys,keyChange;
extern bit RxCAN0,RxCAN1;
extern U32 msticks;

void User1()	// program select 1 - CUSW
{	
	U32 StartTicks = 0;
	U8  index,index1,ErrCnt,counter,vin,error, cnt;
	bit IGN_OFF=1;

	char VIN[17] = "1C3ADECZ7GV1002UC";

	error = 0;	ErrCnt=0;



/***************************************************************************************
****************************************************************************************
	CAUTION WATCH XDATA BOUNDARIES.  NOT AUTOMATIC ITS UP TO YOU!!!!!!
****************************************************************************************
	// CODE CURRENTLY USES 809 xdata bytes.  THEREFORE STARTING CAN BE AT 0x32A
	// watch at compile the xdata bytes used. if xdata > 810d need to change below
	// xdata space ends at 0xFFF.
***************************************************************************************/
//	Buttons = (BUTTABLE xdata *)0x32A;	// 20 buttons takes 32Abytes => 0x32A-0x4E1
	ButtCnt = 0;						// each Button = 20 bytes

//	Mesgs = (MSGTABLE xdata *)0x4E2;	// 0x4E2-0x93F -> space for 43 Tx messages 
	MesgCnt = 0;						// each Mesgs = 26 bytes

//	RxMesg = (RxMESG xdata *)0x940;		// 0x940-0xFF8 -> space for 43 Rx messages 
	RxCnt = 0;							// each RxMesg = 40 bytes

// AUTOMATED above some,
// Make Sure XDATASPACE define follows compile output value  
#define XDATASPACE 810		// starting free address

/***************************************************************************************
// Currently (809 xdata start) you get 43 Tx messages and 43 Rx message traps.  If you 
// need more of one and less of the other you can manipulate the starting numbers below
// to get it to work.
// also note RX0 buff size is 32 messages on CAN0, and RX1 buff size is 16 for CAN1
***************************************************************************************/
	MaxMesgs = 45;			// number of Tx message configs
	MaxRxTraps = 45;		// number of Rx message configs
	while(!MemManage(MAXBUTTONS,MaxMesgs,MaxRxTraps,XDATASPACE))	// 20 button configs  
	{
		MaxRxTraps--;		// reduce Message buffers till it fits.
		if (!MemManage(MAXBUTTONS,MaxMesgs,MaxRxTraps,XDATASPACE))
			MaxMesgs--;		// reduce Message buffers till it fits
	}
/**************************************************************************************/


/*************************************************************************
	Repeating message setups
	example below-
	message ID 0x6E2 is created at rate of 500mSec on port 1 (C). With 8 bytes 
	of data, shown are it's defaults.
**************************************************************************/
	// build repetative message 6E2 on CAN1
	OneTime[0].cdat[0] = 0x08;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
//U8 AddMessage(U8 port,U32 ID,U16 Rate,U8 cnt,U8 dat[8])
	AddMessage(0,0x339,250,8,OneTime[0].cdat); //STATUS_BCM

	// build repetative message 3E0 on CAN0 
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	AddMessage(0,0x33B,250,8,OneTime[0].cdat); //STATUS_BCM2

	// build repetative message 46C on CAN0 
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	AddMessage(0,0x3CB,100,8,OneTime[0].cdat); //DYNAMIC_VEHICLE_INFO2
	
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	AddMessage(0,0x3D1,100,4,OneTime[0].cdat); //VEHICLE_SPEED_ODOMETER
	
	//vin = 0;
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	AddMessage(0,0x3D3,1000,8,OneTime[0].cdat); //VIN
	
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0x05;
	OneTime[0].cdat[4] = 0;
	AddMessage(0,0x499,500,5,OneTime[0].cdat); //PAM_B
	
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	AddMessage(0,0x4A3,450,8,OneTime[0].cdat); //SWS_8
	
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	AddMessage(0,0x548,250,4,OneTime[0].cdat); //STATUS_C_CAN
	
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	AddMessage(0,0x5C8,1000,3,OneTime[0].cdat); //CBC_I1
	
	OneTime[0].cdat[0] = 0xC8;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0xFF;
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0xFF;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	AddMessage(0,0x5CA,1000,8,OneTime[0].cdat); //CBC_I2
	
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	AddMessage(0,0x5CC,1000,4,OneTime[0].cdat); //DAS_B
	
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0x20;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	AddMessage(0,0x5DC,1000,5,OneTime[0].cdat); //GW_C_I2
	
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	AddMessage(0,0x6A4,500,8,OneTime[0].cdat); //ENVIRONMENTAL_CONDITIONS
	
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	AddMessage(0,0x75A,1000,2,OneTime[0].cdat); //AMB_TEMP_DISP
	
	OneTime[0].cdat[0] = 0x0F;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	AddMessage(0,0x762,1000,3,OneTime[0].cdat); //COMPASS_A1
	
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	AddMessage(0,0x772,1000,4,OneTime[0].cdat); //IBS_2

		// build repetative message 6EA on CAN0
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0x22;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	AddMessage(0,0x190,100,5,OneTime[0].cdat);  //CBC_I4

		// build repetative message 6EA on CAN0
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	AddMessage(0,0x098,1000,8,OneTime[0].cdat); //STATUS_B_BSM
	
	
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0x02;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	AddMessage(0,0x0E094000,1000,6,OneTime[0].cdat);
	
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	AddMessage(0,0x5E0,1000,8,OneTime[0].cdat); //ICS_KNOBS
	
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	AddMessage(0,0x5E2,1000,8,OneTime[0].cdat); //ICS_MSG

/*************************************************************************
 	Button setup
	example below -
	pressing button 1 causes byte 5 of message 0x6E2 to be altered to 0xAA 
	for the next 5 outputs. The mask clears byte 5 leaving the rest alone.  
	The value (0xAA) is ORed into byte 5.
**************************************************************************/
	#define BUTTONMASK 0xF	// only button 1 defined (bit 1) 

	// Volume Up
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0;		
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0x08;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
//U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
	SimpleButtonConfig(19,0,0x4A3,OneTime[0].cdat,OneTime[1].cdat,1);

	// Volume Down
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0;		
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0x20;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
//U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
	SimpleButtonConfig(9,0,0x4A3,OneTime[0].cdat,OneTime[1].cdat,1);

	// Mode
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0;		
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0x80;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
//U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
	SimpleButtonConfig(18,0,0x4A3,OneTime[0].cdat,OneTime[1].cdat,1);

	// Preset
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0xFF;		
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0x08;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
//U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
	SimpleButtonConfig(8,0,0x4A3,OneTime[0].cdat,OneTime[1].cdat,1);

	// Seek Up
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0xFF;		
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0x02;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
//U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
	SimpleButtonConfig(17,0,0x4A3,OneTime[0].cdat,OneTime[1].cdat,1);

	// Seek Down
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0xFF;		
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0x08;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
//U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
	SimpleButtonConfig(7,0,0x4A3,OneTime[0].cdat,OneTime[1].cdat,1);

	// Phone Pickup
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0xFF;		
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0x80;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
//U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
	SimpleButtonConfig(16,0,0x4A3,OneTime[0].cdat,OneTime[1].cdat,1);

	// Phone Hangup
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0xFF;		
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0x02;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
//U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
	SimpleButtonConfig(6,0,0x4A3,OneTime[0].cdat,OneTime[1].cdat,1);

	// VR
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0xFF;		
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0x20;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
//U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
	SimpleButtonConfig(15,0,0x4A3,OneTime[0].cdat,OneTime[1].cdat,1);

	// Screen Off
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0xFF;		
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0x10;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
//U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
	SimpleButtonConfig(5,0,0x5E2,OneTime[0].cdat,OneTime[1].cdat,1);

	// Mute
	OneTime[0].cdat[0] = 0;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0xFF;		
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0x02;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
//U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
	SimpleButtonConfig(14,0,0x5E2,OneTime[0].cdat,OneTime[1].cdat,1);

	// Back
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0xFF;		
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0x01;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
//U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
	SimpleButtonConfig(4,0,0x5E2,OneTime[0].cdat,OneTime[1].cdat,1);

	// Browse/Enter
	OneTime[0].cdat[0] = 0;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0xFF;		
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0x01;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
//U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
	SimpleButtonConfig(13,0,0x5E2,OneTime[0].cdat,OneTime[1].cdat,1);

	// Screenshot
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0xFF;		
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0xC8;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
//U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
	SimpleButtonConfig(3,0,0x5E2,OneTime[0].cdat,OneTime[1].cdat,12);


	// ENG. Mode
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0xFF;		
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0xC0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
//U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
	SimpleButtonConfig(12,0,0x5E2,OneTime[0].cdat,OneTime[1].cdat,12);

	// Dealer Mode
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0xFF;		
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0xC4;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
//U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
	SimpleButtonConfig(2,0,0x5E2,OneTime[0].cdat,OneTime[1].cdat,12);

	// Night Mode
	OneTime[0].cdat[0] = 0;		// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0xFF;		
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0x00;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
//U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
	SimpleButtonConfig(11,0,0x339,OneTime[0].cdat,OneTime[1].cdat,0);

	// Day Mode
	OneTime[0].cdat[0] = 0;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0xFF;		
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0x08;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
//U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
	SimpleButtonConfig(1,0,0x339,OneTime[0].cdat,OneTime[1].cdat,0);

	// Phone Pickup Long Press
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0xFF;		
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0x80;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
//U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
	SimpleButtonConfig(10,0,0x4A3,OneTime[0].cdat,OneTime[1].cdat,10);

	// VR Long Press
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0xFF;		
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0x20;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
//U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
	SimpleButtonConfig(0,0,0x4A3,OneTime[0].cdat,OneTime[1].cdat,10);

/*************************************************************************
 	RxMessage filters / traps
	Below example:
	First half, Watches for ID 0x6E8 on can port 1 (C).  Only byte 3 matters 
	as the mask is set to 0's for all else.  The matching value required in  
	byte 3 is 0x58. 8 bytes expected.
	Second half, configures output message 0x6E3 on port 1 (C). 8 bytes in 
	count.  mask is ANDed with cyclic message and ORed with value, but in 
	this example there is no cyclic message, therefor mask is ignored and 
	value is just output as the data 1 time. Take care to use the index value
	returned from the first function as input parameter to 2nd function
**************************************************************************/
	OneTime[0].cdat[0] = 0;		// mask bits (0's don't cares, 1's are matches)
	OneTime[0].cdat[1] = 0;
	OneTime[0].cdat[2] = 0;		// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;	// this byte matters
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;	 
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0x58;	// selector switches to x58 causes triggers
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
// configure Rx filter, look for this message match
//U8 RxMessageConfig1(U8 port,U32 ID,U8 mask[8],U8 value[8],U8 count);
	index = RxMessageConfig1(1,0x6E8,OneTime[0].cdat,OneTime[1].cdat,8);
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0xFF;
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0x88;	// values (ORed)
	OneTime[1].cdat[1] = 0x99;
	OneTime[1].cdat[2] = 0xAA;
	OneTime[1].cdat[3] = 0xBB;
	OneTime[1].cdat[4] = 0xCC;
	OneTime[1].cdat[5] = 0xDD;
	OneTime[1].cdat[6] = 0xEE;
	OneTime[1].cdat[7] = 0xFF;
// configure Tx output caused by above Recieved message
//void RxMessageConfig2(U8 index,U8 outport,U32 outID,U8 outmask[8],U8 outvalue[8],U8 cnt,U8 altercnt);
	RxMessageConfig2(index,1,0x6E3,OneTime[0].cdat,OneTime[1].cdat,8,1);


// Detect and set for English
	OneTime[0].cdat[0] = 0xFF;	// mask bits (0's don't cares, 1's are matches)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	
	OneTime[1].cdat[0] = 0xA;	// values (ORed)
	OneTime[1].cdat[1] = 3;
	OneTime[1].cdat[2] = 1;		// this byte must match
	index = RxMessageConfig1(0,0x8A,OneTime[0].cdat,OneTime[1].cdat,3);
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0xFF;
	OneTime[0].cdat[6] = 0x0;	// clear
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 8;		// write
	OneTime[1].cdat[7] = 0;		// note altercount = 0 means permanant
	RxMessageConfig2(index,0,0x49D,OneTime[0].cdat,OneTime[1].cdat,8,0);

// Detect and set for spa
	OneTime[0].cdat[0] = 0xFF;	// mask bits (0's don't cares, 1's are matches)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	
	OneTime[1].cdat[0] = 0xA;	// values (ORed)
	OneTime[1].cdat[1] = 3;
	OneTime[1].cdat[2] = 4;		// this byte must match
	index = RxMessageConfig1(0,0x8A,OneTime[0].cdat,OneTime[1].cdat,3);
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0xFF;
	OneTime[0].cdat[6] = 0x0;	// clear
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0x20;	// write
	OneTime[1].cdat[7] = 0;
	RxMessageConfig2(index,0,0x49D,OneTime[0].cdat,OneTime[1].cdat,8,0);

// Detect and set for ger
	OneTime[0].cdat[0] = 0xFF;	// mask bits (0's don't cares, 1's are matches)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	
	OneTime[1].cdat[0] = 0xA;	// values (ORed)
	OneTime[1].cdat[1] = 3;
	OneTime[1].cdat[2] = 0;		// this byte must match
	index = RxMessageConfig1(0,0x8A,OneTime[0].cdat,OneTime[1].cdat,3);
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0xFF;
	OneTime[0].cdat[6] = 0x0;	// clear
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;		// write
	OneTime[1].cdat[7] = 0;
	RxMessageConfig2(index,0,0x49D,OneTime[0].cdat,OneTime[1].cdat,8,0);

// Detect and set for fre
	OneTime[0].cdat[0] = 0xFF;	// mask bits (0's don't cares, 1's are matches)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	
	OneTime[1].cdat[0] = 0xA;	// values (ORed)
	OneTime[1].cdat[1] = 3;
	OneTime[1].cdat[2] = 2;		// this byte must match
	index = RxMessageConfig1(0,0x8A,OneTime[0].cdat,OneTime[1].cdat,3);
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0xFF;
	OneTime[0].cdat[6] = 0x0;	// clear
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0x10;	// write
	OneTime[1].cdat[7] = 0;
	RxMessageConfig2(index,0,0x49D,OneTime[0].cdat,OneTime[1].cdat,8,0);

// Detect and set for ita
	OneTime[0].cdat[0] = 0xFF;	// mask bits (0's don't cares, 1's are matches)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	
	OneTime[1].cdat[0] = 0xA;	// values (ORed)
	OneTime[1].cdat[1] = 3;
	OneTime[1].cdat[2] = 3;		// this byte must match
	index = RxMessageConfig1(0,0x8A,OneTime[0].cdat,OneTime[1].cdat,3);
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0xFF;
	OneTime[0].cdat[6] = 0x0;	// clear
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0x60;	// write
	OneTime[1].cdat[7] = 0;
	RxMessageConfig2(index,0,0x49D,OneTime[0].cdat,OneTime[1].cdat,8,0);

// Detect and set for jap
	OneTime[0].cdat[0] = 0xFF;	// mask bits (0's don't cares, 1's are matches)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	
	OneTime[1].cdat[0] = 0xA;	// values (ORed)
	OneTime[1].cdat[1] = 3;
	OneTime[1].cdat[2] = 5;		// this byte must match
	index = RxMessageConfig1(0,0x8A,OneTime[0].cdat,OneTime[1].cdat,3);
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0xFF;
	OneTime[0].cdat[6] = 0x0;	// clear
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0x50;	// write
	OneTime[1].cdat[7] = 0;
	RxMessageConfig2(index,0,0x49D,OneTime[0].cdat,OneTime[1].cdat,8,0);

// Detect and set for chs
	OneTime[0].cdat[0] = 0xFF;	// mask bits (0's don't cares, 1's are matches)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	
	OneTime[1].cdat[0] = 0xA;	// values (ORed)
	OneTime[1].cdat[1] = 3;
	OneTime[1].cdat[2] = 9;		// this byte must match
	index = RxMessageConfig1(0,0x8A,OneTime[0].cdat,OneTime[1].cdat,3);
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0xFF;
	OneTime[0].cdat[6] = 0x0;	// clear
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0x48;	// write
	OneTime[1].cdat[7] = 0;
	RxMessageConfig2(index,0,0x49D,OneTime[0].cdat,OneTime[1].cdat,8,0);

// Detect and set for cht
	OneTime[0].cdat[0] = 0xFF;	// mask bits (0's don't cares, 1's are matches)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	
	OneTime[1].cdat[0] = 0xA;	// values (ORed)
	OneTime[1].cdat[1] = 3;
	OneTime[1].cdat[2] = 0xE;	// this byte must match
	index = RxMessageConfig1(0,0x8A,OneTime[0].cdat,OneTime[1].cdat,3);
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0xFF;
	OneTime[0].cdat[6] = 0x0;	// clear
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0x90;	// write
	OneTime[1].cdat[7] = 0;
	RxMessageConfig2(index,0,0x49D,OneTime[0].cdat,OneTime[1].cdat,8,0);

	// Configure Relay's
	RelayCnf[3] = REL_IGN | 1 | REL_NOT;   	// relay 4 comes on with Ignition switch setting not 1
	//RelayCnf[2] = REL_BUTT | 1 | REL_TOG;	// relay 3 toggles with button 1
	//RelayCnf[1] = REL_RX | 0 | REL_PULSE;	// relay 2 with first (0) rxtrap configured
	//RelayCnf[0] = REL_TX | 3 | REL_PULSE;	// relay 1 with 4th (3) TxMesg configured

/*************************************************************************
	User running loop 
**************************************************************************/
	cnt = 0;
	counter =0;
	while(!(keyChange & 0xF)) 
	{
		if (RxCAN0)		// new message to process
		{
			for (index = 1 ; index < RX0BUFFERS && rxframe0[index].MsgNum>=RX0BUFFERS ; index++);

			if (index < RX0BUFFERS)
			{
				ProcessRx(0,index);
				rxframe0[index].MsgNum = 0;
			}
			RxCAN0 = 0;
		}
		if (RxCAN1)		// new message to process
		{
			error = CAN1readStatus();

			index = RX1BUFFERS;
			index1 = RX1BUFFERS;
			if (error & MCP_RX1IF)
				index1 = CAN1read_canMsg( MCP_READ_RX1 );        
			if (error & MCP_RX0IF)
				index = CAN1read_canMsg( MCP_READ_RX0 );        
			if (error = CAN1readRegister(MCP_EFLG))
			{
				ErrCnt++;
				AlterMessageByte(1,0x6E2,7,1,ErrCnt,0);

				if (error & MCP_EFLG_RX1OVR)
					CAN1modifyRegister(MCP_EFLG,MCP_EFLG_RX1OVR,0);
				if (error & MCP_EFLG_RX0OVR)
					CAN1modifyRegister(MCP_EFLG,MCP_EFLG_RX0OVR,0);
			}

			// clear int flags
			IE0 = 0;					// clear IE0 flag re-enabling interupt
			RxCAN1 = 0;					// clear my flag interupt occured
			EX0 = 1;					// enable CAN1 interupt

			if (index < RX1BUFFERS)				// process recieved message from Rx0
			{
				ProcessRx(1,index);		// CAN1 messages
			}
			if (index1 < RX1BUFFERS)			// process recieved message from Rx1
			{
				ProcessRx(1,index1);
			}
		}

		if (StartTicks + ONESEC < msticks )	// every second
		{
			StartTicks = msticks;
			LED1 ^= 1;

			for (index1 = 0 ; index1 < TXBUFFS && OneTime[index1].arbID ; index1++);
			if (index1 < TXBUFFS)				// empty one time available
			{
				OneTime[index1].cdat[0] = (U8)(keys>>24);
				OneTime[index1].cdat[1] = (U8)(keys>>16);
				OneTime[index1].cdat[2] = (U8)(keys>>8);
				OneTime[index1].cdat[3] = (U8)keys;
				OneTime[index1].cdat[4] = CAN1readStatus();
				OneTime[index1].cdat[5] = CAN1readRxStat();
				OneTime[index1].cdat[6] = CAN1readRegister(MCP_CANSTAT);
				OneTime[index1].cdat[7] = CAN1readRegister(MCP_CANINTF);
				AlterMessageByte(0,0x6EA,0,OneTime[index1].cdat,8,1);
			}

			// VIN
			for (index1 = 0 ; index1 < TXBUFFS && OneTime[index1].arbID ; index1++);
			if (index1 < TXBUFFS)				// empty one time available
			{
				switch (vin++)
				{
					case 0:
						OneTime[index1].cdat[0] = 0;
						for (index = 0 ; index < 7 ; index++)
							OneTime[index1].cdat[1+index] = VIN[index];
						break;
					case 1:
						OneTime[index1].cdat[0] = 1;
						for (index = 7 ; index < 14 ; index++)
							OneTime[index1].cdat[index-6] = VIN[index];
						break;
					case 2:
						OneTime[index1].cdat[0] = 2;
						for (index = 14 ; index < 17 ; index++)
							OneTime[index1].cdat[index-13] = VIN[index];
						vin = 0;
						break;
				}
				AlterMessageByte(0,0x3D3,0,OneTime[index1].cdat,8,0);
			}

			if (counter++ >= 9)	// test every 10 seconds alter 6E2 message
			{
				for (index1 = 0 ; index1 < TXBUFFS && OneTime[index1].arbID ; index1++);
				if (index1 < TXBUFFS)				// empty one time available
				{
					OneTime[index1].cdat[0] = cnt++;
					AlterMessageByte(1,0x6E2,6,OneTime[index1].cdat,1,2);
					counter = 0;
				}
			}
		}


		if (RelayChange)
			CheckRelay();

		// check for buttons pressed and process
		if (keyChange & 0xFFFFF00)	// At least one Buttons changed state
		{
			CheckButts();			// SimpleButton Check

			keyChange &= (U32)((BUTTONMASK<<8) & 0xFF);	// clear unused buttons
		}


		if (keyChange & 0xF0)		// Ignition change
		{
			// check if a relay is set to Ignition change
			for (index1 = 0 ; index1 < 4; index1++){
				if ((RelayCnf[index1] & 0xF00) == REL_IGN)	// Relay controled by IGN select
				{
					if ((RelayCnf[index1]&0xFF) == ((keys>>4)&0xF))	// matching Select
						RelayChange |= 1<<index1;			// flag for relay "on" change
					else
						RelayChange |= 0x10<<index1;		// flag for relay "off" change
				}
			}

			for (index1 = 0 ; index1 < TXBUFFS && OneTime[index1].arbID ; index1++);
			if (index1 < TXBUFFS)				// empty one time available
			{
				IGN_OFF = 0;			// default IGN_OFF flag to off
				switch(keys & 0xF0)
				{
					case 0x10:	// IGN_OFF
						// ID 190, alter byte 0,1 
						OneTime[0].cdat[0] = 0;	      	//ignition run, start not active  04 = run ;02=ACC ;0D= NormalStart, key in Ign, 						
						OneTime[0].cdat[1] = 0;		  	//key in ignition
						OneTime[0].cdat[2] = 0;
						OneTime[0].cdat[3] = 0;
						OneTime[0].cdat[4] = 0;
						OneTime[0].sz = 5;
						OneTime[0].arbID = 0x190 | 0;
		//				AlterMessageByte(0,0x190,0,OneTime[index1].cdat,2,0);


/*						// ID 98, alter bytes 2,3  
						OneTime[0].cdat[0] = 0; 			// 0 speed			
						OneTime[0].cdat[1] = 0;
						OneTime[0].cdat[2] = 0;
						OneTime[0].cdat[3] = 0;
						OneTime[0].cdat[4] = 0;
						OneTime[0].cdat[5] = 0;
						OneTime[0].cdat[6] = 0;
						OneTime[0].cdat[7] = 0;
						OneTime[0].sz = 8;
						OneTime[0].arbID = 0x098 | 0;
		//				AlterMessageByte(0,0x098,0,OneTime[index1].cdat,4,0); */
						IGN_OFF = 1;	// TURN on IGN_OFF flag
						IGN_OFF_FLG= 0;
						break;

					case 0x20:	// IGN_ACC
						// ID 190, alter byte 0,1 
						OneTime[index1].cdat[0] = 0x03;	      	//ignition run, start not active  04 = run ;02=ACC ;0D= NormalStart, key in Ign, 						
						OneTime[index1].cdat[1] = 0x22;		  	//key in ignition
						AlterMessageByte(0,0x190,0,OneTime[index1].cdat,2,0);

						// ID 98, alter bytes 2,3  
						OneTime[index1].cdat[0] = 0x0; 			// 0 speed			
						OneTime[index1].cdat[1] = 0x0;
						OneTime[index1].cdat[2] = 0x0;
						OneTime[index1].cdat[3] = 0x0;
						AlterMessageByte(0,0x098,0,OneTime[index1].cdat,4,0);
						IGN_OFF_FLG = 1;
						break;
					case 0x30:	// IGN_START
						// ID 190, alter byte 0,1 
						OneTime[index1].cdat[0] = 0x05;	      	//ignition run, start not active  04 = run ;02=ACC ;0D= NormalStart, key in Ign, 						
						OneTime[index1].cdat[1] = 0x22;		  	//key in ignition
						AlterMessageByte(0,0x190,0,OneTime[index1].cdat,2,0);

						// ID 98, alter bytes 2,3  
						OneTime[index1].cdat[0] = 0x0; 			// 0 speed			
						OneTime[index1].cdat[1] = 0x0;
						OneTime[index1].cdat[2] = 0x0;
						OneTime[index1].cdat[3] = 0x0;
						AlterMessageByte(0,0x098,0,OneTime[index1].cdat,4,0);
						IGN_OFF_FLG = 1;
						break;
					case 0x40:	// IGN_RUN_0
						// ID 190, alter byte 0,1 
						OneTime[index1].cdat[0] = 0x04;	      	//ignition run, start not active  04 = run ;02=ACC ;0D= NormalStart, key in Ign, 						
						OneTime[index1].cdat[1] = 0x22;		  	//key in ignition
						AlterMessageByte(0,0x190,0,OneTime[index1].cdat,2,0);

						// ID 98, alter bytes 2,3  
						OneTime[index1].cdat[0] = 0x0; 			// 0 speed			
						OneTime[index1].cdat[1] = 0x0;
						OneTime[index1].cdat[2] = 0x0;
						OneTime[index1].cdat[3] = 0x0;
						AlterMessageByte(0,0x098,0,OneTime[index1].cdat,4,0);
						IGN_OFF_FLG = 1;
						break;
					case 0x50:	// IGN_RUN_5
					// ID 190, alter byte 0,1 
						OneTime[index1].cdat[0] = 0x04;	      	//ignition run, start not active  04 = run ;02=ACC ;0D= NormalStart, key in Ign, 						
						OneTime[index1].cdat[1] = 0x22;		  	//key in ignition
						AlterMessageByte(0,0x190,0,OneTime[index1].cdat,2,0);

						// ID 98, alter bytes 2,3  
						OneTime[index1].cdat[0] = 0x0; 			// 0 speed			
						OneTime[index1].cdat[1] = 0x0;
						OneTime[index1].cdat[2] = 0x0;
						OneTime[index1].cdat[3] = 0x81;;
						AlterMessageByte(0,0x098,0,OneTime[index1].cdat,4,0);
						IGN_OFF_FLG = 1;
						break;
					case 0x60:	// IGN_RUN_10
					// ID 190, alter byte 0,1 
						OneTime[index1].cdat[0] = 0x04;	      	//ignition run, start not active  04 = run ;02=ACC ;0D= NormalStart, key in Ign, 						
						OneTime[index1].cdat[1] = 0x22;		  	//key in ignition
						AlterMessageByte(0,0x190,0,OneTime[index1].cdat,2,0);

						// ID 98, alter bytes 2,3  
						OneTime[index1].cdat[0] = 0x0; 			// 0 speed			
						OneTime[index1].cdat[1] = 0x0;
						OneTime[index1].cdat[2] = 0x0;
						OneTime[index1].cdat[3] = 0xFD;
						AlterMessageByte(0,0x098,0,OneTime[index1].cdat,4,0);
						IGN_OFF_FLG = 1;
						break;
					
					case 0x70:
					/*	// ID 190, alter byte 0,1 
						OneTime[index1].cdat[0] = 0x04;	      	//ignition run, start not active  04 = run ;02=ACC ;0D= NormalStart, key in Ign, 						
						OneTime[index1].cdat[1] = 0x42;		  	//key in ignition
						AlterMessageByte(0,0x190,0,OneTime[index1].cdat,2,0);
						// ID 98, alter bytes 2,3  
						OneTime[index1].cdat[0] = 0x0; 			// 0 speed			
						OneTime[index1].cdat[1] = 0x0;
						OneTime[index1].cdat[2] = 0x0;
						OneTime[index1].cdat[3] = 0x0;
						AlterMessageByte(0,0x098,0,OneTime[index1].cdat,4,0);
						IGN_OFF_FLG = 1; */
						break;
					case 0x80:
						break;
				}

			keyChange &= ~0xF0;		// clear Ignition nibble
			}
		}
	}
}
void User2()	// program select 2, PNET
{

	U32 StartTicks = 0;
	U8  index,index1,ErrCnt,counter,vin,error, cnt;
	U8 Day_Night_mode=0x01;
	bit IGN_OFF=1;

	char VIN[17] = "1C3CCCCB7FN1124UC";
//	char VIN[17] = "2C3CDZBT0HH1110XM";

	error = 0;	ErrCnt=0;



/***************************************************************************************
****************************************************************************************
	CAUTION WATCH XDATA BOUNDARIES.  NOT AUTOMATIC ITS UP TO YOU!!!!!!
****************************************************************************************
	// CODE CURRENTLY USES 809 xdata bytes.  THEREFORE STARTING CAN BE AT 0x32A
	// watch at compile the xdata bytes used. if xdata > 810d need to change below
	// xdata space ends at 0xFFF.
***************************************************************************************/
//	Buttons = (BUTTABLE xdata *)0x32A;	// 20 buttons takes 32Abytes => 0x32A-0x4E1
	ButtCnt = 0;						// each Button = 20 bytes

//	Mesgs = (MSGTABLE xdata *)0x4E2;	// 0x4E2-0x93F -> space for 43 Tx messages 
	MesgCnt = 0;						// each Mesgs = 26 bytes

//	RxMesg = (RxMESG xdata *)0x940;		// 0x940-0xFF8 -> space for 43 Rx messages 
	RxCnt = 0;							// each RxMesg = 40 bytes

// AUTOMATED above some,
// Make Sure XDATASPACE define follows compile output value  
#define XDATASPACE 810		// starting free address

/***************************************************************************************
// Currently (809 xdata start) you get 43 Tx messages and 43 Rx message traps.  If you 
// need more of one and less of the other you can manipulate the starting numbers below
// to get it to work.
// also note RX0 buff size is 32 messages on CAN0, and RX1 buff size is 16 for CAN1
***************************************************************************************/
	MaxMesgs = 45;			// number of Tx message configs
	MaxRxTraps = 45;		// number of Rx message configs
	while(!MemManage(MAXBUTTONS,MaxMesgs,MaxRxTraps,XDATASPACE))	// 20 button configs  
	{
		MaxRxTraps--;		// reduce Message buffers till it fits.
		if (!MemManage(MAXBUTTONS,MaxMesgs,MaxRxTraps,XDATASPACE))
			MaxMesgs--;		// reduce Message buffers till it fits
	}
/**************************************************************************************/


/*************************************************************************
	Repeating message setups
	example below-
	message ID 0x6E2 is created at rate of 500mSec on port 1 (C). With 8 bytes 
	of data, shown are it's defaults.
**************************************************************************/
	
	//OneTime[0].cdat[0] = 0x03;
	//AddMessage(0,0x12B,100,1,OneTime[0].cdat);  // TGW_PWR_MAST
		
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	AddMessage(0,0x122,100,4,OneTime[0].cdat);  // CBC_I4 - IGN state
	
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	AddMessage(0,0x22D,200,8,OneTime[0].cdat);  // SWS_8 - Steering Wheel messages
	
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	AddMessage(0,0x273,1000,8,OneTime[0].cdat);  // ICS_KNOBS
	
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	
	OneTime[0].cdat[2] = 0x07;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	AddMessage(0,0x2AD,1000,8,OneTime[0].cdat);  // CBC_CFG2
	
	OneTime[0].cdat[0] = 0x18;
	OneTime[0].cdat[1] = 0x38;	
	OneTime[0].cdat[2] = 0x8B;
	OneTime[0].cdat[3] = 0;
	AddMessage(0,0x2C2,200,4,OneTime[0].cdat);  // CBC_I3
	
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0xEE;	
	OneTime[0].cdat[2] = 0xEE;
	OneTime[0].cdat[3] = 0x2E;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	AddMessage(0,0x2C5,2000,8,OneTime[0].cdat);  // EcuCfg16
	
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	AddMessage(0,0x2D3,1000,8,OneTime[0].cdat);  // ICS_MSG
	
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0x01; 	
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0x18;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0x01;
	AddMessage(0,0x2FA,500,8,OneTime[0].cdat);  // CBC_I2
	
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0x83;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	AddMessage(0,0x305,1000,8,OneTime[0].cdat);  // CBC_I6
	
	OneTime[0].cdat[0] = 0x19;
	OneTime[0].cdat[1] = 0;	
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0x01;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;

	AddMessage(0,0x381,2000,8,OneTime[0].cdat);  // VehCfg7
	
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0x01;	
	OneTime[0].cdat[2] = 0x4C;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	AddMessage(0,0x3A2,1000,6,OneTime[0].cdat);  // CBC_I5
	
	OneTime[0].cdat[0] = 0x01;
	OneTime[0].cdat[1] = 0;	
	OneTime[0].cdat[2] = 0x80;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0xFF;
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0;
	AddMessage(0,0x3B2,500,8,OneTime[0].cdat);  // CBC_I1
	
	OneTime[0].cdat[0] = 0x45;
	OneTime[0].cdat[1] = 0x30;	
	OneTime[0].cdat[2] = 0x22;
	OneTime[0].cdat[3] = 0x01;
	OneTime[0].cdat[4] = 0x61;
	OneTime[0].cdat[5] = 0xA7;
	OneTime[0].cdat[6] = 0xC7;
	OneTime[0].cdat[7] = 0x98;
	AddMessage(0,0x3B3,2000,8,OneTime[0].cdat);  // VehCfgCSM1
	
	OneTime[0].cdat[0] = 0xFD;
	OneTime[0].cdat[1] = 0x80;	
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	AddMessage(0,0x3B4,2000,8,OneTime[0].cdat);  // VehCfgCSM2
	
	OneTime[0].cdat[0] = 0x3E;
	OneTime[0].cdat[1] = 0x1E;	
	OneTime[0].cdat[2] = 0x1E;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0x00;
	OneTime[0].cdat[5] = 0x01;
	OneTime[0].cdat[6] = 0x90;
	OneTime[0].cdat[7] = 0x01;
	AddMessage(0,0x3DE,500,8,OneTime[0].cdat);  // CBC_CFG1
	
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	AddMessage(0,0x3E0,1000,8,OneTime[0].cdat);  // VIN
	
//	OneTime[0].cdat[0] = 0xFD;
	OneTime[0].cdat[0] = 0xF5;
	OneTime[0].cdat[1] = 0x0C;	
	OneTime[0].cdat[2] = 0x10;
	OneTime[0].cdat[3] = 0x08;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	AddMessage(0,0x3E3,2000,8,OneTime[0].cdat);  // NET_CFG_INT
	
	OneTime[0].cdat[0] = 0x89;
	OneTime[0].cdat[1] = 0xDA;	
	OneTime[0].cdat[2] = 0x98;
	OneTime[0].cdat[3] = 0x06;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	AddMessage(0,0x3E4,2000,8,OneTime[0].cdat);  //NET_CFG_PT
	
	OneTime[0].cdat[0] = 0x45;
	OneTime[0].cdat[1] = 0x29;	
	OneTime[0].cdat[2] = 0xA2;
	OneTime[0].cdat[3] = 0x15;
	OneTime[0].cdat[4] = 0x1E;
	OneTime[0].cdat[5] = 0x80;
	OneTime[0].cdat[6] = 0xA2;
	OneTime[0].cdat[7] = 0x10;
	AddMessage(0,0x3E8,2000,8,OneTime[0].cdat);  // VehCfg1
	
	OneTime[0].cdat[0] = 0x01;
	OneTime[0].cdat[1] = 0;	
	OneTime[0].cdat[2] = 0x06;
	OneTime[0].cdat[3] = 0x9E;
	OneTime[0].cdat[4] = 0x20;
	OneTime[0].cdat[5] = 0x20;
	OneTime[0].cdat[6] = 0x20;
	OneTime[0].cdat[7] = 0x20;
	AddMessage(0,0x3E9,2000,8,OneTime[0].cdat);  // VehCfg2
	
	OneTime[0].cdat[0] = 0x41;
	OneTime[0].cdat[1] = 0x0D;	
	OneTime[0].cdat[2] = 0x22;
	OneTime[0].cdat[3] = 0x48;
	OneTime[0].cdat[4] = 0x80;
	OneTime[0].cdat[5] = 0x24;
	OneTime[0].cdat[6] = 0x03;
	OneTime[0].cdat[7] = 0x40;
	AddMessage(0,0x3EA,2000,8,OneTime[0].cdat);  // VehCfg3
	
	OneTime[0].cdat[0] = 0x40;
	OneTime[0].cdat[1] = 0;	
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0x08;
	OneTime[0].cdat[7] = 0xC6;
	AddMessage(0,0x3EB,2000,8,OneTime[0].cdat);  // VehCfg4
	
	OneTime[0].cdat[0] = 0x45;
	OneTime[0].cdat[1] = 0x02;	
	OneTime[0].cdat[2] = 0x1E;
	OneTime[0].cdat[3] = 0x20;
	OneTime[0].cdat[4] = 0x19;
	OneTime[0].cdat[5] = 0x20; // A0 --> DSP_SK_PRSNT
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0x04;
	AddMessage(0,0x3F2,2000,8,OneTime[0].cdat);  // EcuCfg3
	
	OneTime[0].cdat[0] = 0x01;
	OneTime[0].cdat[1] = 0;	
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	AddMessage(0,0x44A,2000,8,OneTime[0].cdat);  // VehCfg5
	
	OneTime[0].cdat[0] = 0x01;
	OneTime[0].cdat[1] = 0;	
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	AddMessage(0,0x44C,2000,8,OneTime[0].cdat);  // VehCfg6
	
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	AddMessage(0,0x322,100,8,OneTime[0].cdat);  // GW_C1

/*	OneTime[0].cdat[0] = 0xFD;
	OneTime[0].cdat[1] = 0x2C;	
	OneTime[0].cdat[2] = 0x3F;
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0xFF;
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	AddMessage(0,0x401,100,8,OneTime[0].cdat); // CAN msg for DCSD screen wakeup */


/*************************************************************************
 	Button setup
	example below -
	pressing button 1 causes byte 5 of message 0x6E2 to be altered to 0xAA 
	for the next 5 outputs. The mask clears byte 5 leaving the rest alone.  
	The value (0xAA) is ORed into byte 5.
**************************************************************************/
	#define BUTTONMASK 0xF	// only button 1 defined (bit 1) 

	// Volume Up
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0xFF;		
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0x01;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
//U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
	SimpleButtonConfig(19,0,0x22D,OneTime[0].cdat,OneTime[1].cdat,1);

	// Volume Down
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0xFF;		
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0x04;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
//U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
	SimpleButtonConfig(9,0,0x22D,OneTime[0].cdat,OneTime[1].cdat,1);

	// Mode
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0xFF;		
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0x10;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
//U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
	SimpleButtonConfig(18,0,0x22D,OneTime[0].cdat,OneTime[1].cdat,1);

	// Preset
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0xFF;		
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0x01;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
//U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
	SimpleButtonConfig(8,0,0x22D,OneTime[0].cdat,OneTime[1].cdat,1);

	// Seek Up
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0;		// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0xFF;		
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0x40;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
//U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
	SimpleButtonConfig(17,0,0x22D,OneTime[0].cdat,OneTime[1].cdat,1);

	// Seek Down
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0xFF;		
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0x01;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
//U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
	SimpleButtonConfig(7,0,0x22D,OneTime[0].cdat,OneTime[1].cdat,1);

	// Phone Pickup
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0xFF;		
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0x10;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
//U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
	SimpleButtonConfig(16,0,0x22D,OneTime[0].cdat,OneTime[1].cdat,1);

	// Phone Hangup
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0xFF;		
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0x40;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
//U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
	SimpleButtonConfig(6,0,0x22D,OneTime[0].cdat,OneTime[1].cdat,1);

	// VR
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0xFF;		
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0x04;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
//U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
	SimpleButtonConfig(15,0,0x22D,OneTime[0].cdat,OneTime[1].cdat,1);

	// Screen Off
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0xFF;		
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0x20;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
//U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
	SimpleButtonConfig(5,0,0x2D3,OneTime[0].cdat,OneTime[1].cdat,1);

/*	// Mute
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0xFF;		
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0x01;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
//U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
	SimpleButtonConfig(14,0,0x2D3,OneTime[0].cdat,OneTime[1].cdat,1); */

	// Radio Off
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0xFF;		
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0x40;
//U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
	SimpleButtonConfig(14,0,0x2D3,OneTime[0].cdat,OneTime[1].cdat,1);

	// Back
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0xFF;		
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0x02;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
//U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
	SimpleButtonConfig(4,0,0x2D3,OneTime[0].cdat,OneTime[1].cdat,1);

	// Browse/Enter
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0xFF;		
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0x02;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
//U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
	SimpleButtonConfig(13,0,0x2D3,OneTime[0].cdat,OneTime[1].cdat,1);

	// Screenshot
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0;		
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0x4C;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
//U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
	SimpleButtonConfig(3,0,0x2D3,OneTime[0].cdat,OneTime[1].cdat,5);


	// ENG. Mode
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0;		
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0x0C;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
//U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
	SimpleButtonConfig(12,0,0x2D3,OneTime[0].cdat,OneTime[1].cdat,5);

	// Dealer Mode
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0;		
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0x8C;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
//U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
	SimpleButtonConfig(2,0,0x2D3,OneTime[0].cdat,OneTime[1].cdat,5);


	// Night Mode
	//Day_Night_mode ^= 0x01;
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0xFF;		
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0x00;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
//U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
	SimpleButtonConfig(11,0,0x2FA,OneTime[0].cdat,OneTime[1].cdat,0);


	// Day Mode
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0xFF;		
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0x01;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
//U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
	SimpleButtonConfig(1,0,0x2FA,OneTime[0].cdat,OneTime[1].cdat,0); 


/*
	// Door Ajar
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0xFF;		
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0x80;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
//U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
	SimpleButtonConfig(10,0,0x22D,OneTime[0].cdat,OneTime[1].cdat,1);
	// Theft Alarm
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0xFF;		
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0x80;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
//U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
	SimpleButtonConfig(,0,0x22D,OneTime[0].cdat,OneTime[1].cdat,1);
*/

// Phone Pickup Long Press
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0xFF;		
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0x10;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
//U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
	SimpleButtonConfig(10,0,0x22D,OneTime[0].cdat,OneTime[1].cdat,4);

// VR Long Press
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0xFF;		
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0x04;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
//U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
	SimpleButtonConfig(0,0,0x22D,OneTime[0].cdat,OneTime[1].cdat,4);




/*************************************************************************
 	RxMessage filters / traps
	Below example:
	First half, Watches for ID 0x6E8 on can port 1 (C).  Only byte 3 matters 
	as the mask is set to 0's for all else.  The matching value required in  
	byte 3 is 0x58. 8 bytes expected.
	Second half, configures output message 0x6E3 on port 1 (C). 8 bytes in 
	count.  mask is ANDed with cyclic message and ORed with value, but in 
	this example there is no cyclic message, therefor mask is ignored and 
	value is just output as the data 1 time. Take care to use the index value
	returned from the first function as input parameter to 2nd function
**************************************************************************/
	OneTime[0].cdat[0] = 0;		// mask bits (0's don't cares, 1's are matches)
	OneTime[0].cdat[1] = 0;
	OneTime[0].cdat[2] = 0;		// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;	// this byte matters
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;	 
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0x58;	// selector switches to x58 causes triggers
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
// configure Rx filter, look for this message match
//U8 RxMessageConfig1(U8 port,U32 ID,U8 mask[8],U8 value[8],U8 count);
	index = RxMessageConfig1(1,0x6E8,OneTime[0].cdat,OneTime[1].cdat,8);
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0xFF;
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0x88;	// values (ORed)
	OneTime[1].cdat[1] = 0x99;
	OneTime[1].cdat[2] = 0xAA;
	OneTime[1].cdat[3] = 0xBB;
	OneTime[1].cdat[4] = 0xCC;
	OneTime[1].cdat[5] = 0xDD;
	OneTime[1].cdat[6] = 0xEE;
	OneTime[1].cdat[7] = 0xFF;
// configure Tx output caused by above Recieved message
//void RxMessageConfig2(U8 index,U8 outport,U32 outID,U8 outmask[8],U8 outvalue[8],U8 cnt,U8 altercnt);
	RxMessageConfig2(index,1,0x6E3,OneTime[0].cdat,OneTime[1].cdat,8,1);


// Detect and set for English
	OneTime[0].cdat[0] = 0xFF;	// mask bits (0's don't cares, 1's are matches)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	
	OneTime[1].cdat[0] = 0x0A;	// values (ORed)
	OneTime[1].cdat[1] = 0x03;
	OneTime[1].cdat[2] = 0x01;		// this byte must match
	index = RxMessageConfig1(0,0x314,OneTime[0].cdat,OneTime[1].cdat,3);
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0;
	OneTime[0].cdat[2] = 0xFF;	
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0xFF;	// clear
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0x50;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0x08;
	OneTime[1].cdat[6] = 0;		// write
	OneTime[1].cdat[7] = 0;		// note altercount = 0 means permanant
	RxMessageConfig2(index,0,0x332,OneTime[0].cdat,OneTime[1].cdat,8,0);

// Detect and set for spa
	OneTime[0].cdat[0] = 0xFF;	// mask bits (0's don't cares, 1's are matches)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	
	OneTime[1].cdat[0] = 0x0A;	// values (ORed)
	OneTime[1].cdat[1] = 0x03;
	OneTime[1].cdat[2] = 0x04;		// this byte must match
	index = RxMessageConfig1(0,0x314,OneTime[0].cdat,OneTime[1].cdat,3);
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0;
	OneTime[0].cdat[2] = 0xFF;	
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0xFF;	// clear
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0x50;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0x20;
	OneTime[1].cdat[6] = 0;	// write
	OneTime[1].cdat[7] = 0;
	RxMessageConfig2(index,0,0x332,OneTime[0].cdat,OneTime[1].cdat,8,0);

// Detect and set for ger
	OneTime[0].cdat[0] = 0xFF;	// mask bits (0's don't cares, 1's are matches)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	
	OneTime[1].cdat[0] = 0x0A;	// values (ORed)
	OneTime[1].cdat[1] = 0x03;
	OneTime[1].cdat[2] = 0x00;		// this byte must match
	index = RxMessageConfig1(0,0x314,OneTime[0].cdat,OneTime[1].cdat,3);
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0;
	OneTime[0].cdat[2] = 0xFF;	
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0xFF;	// clear
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0x50;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0x00;
	OneTime[1].cdat[6] = 0;		// write
	OneTime[1].cdat[7] = 0;
	RxMessageConfig2(index,0,0x332,OneTime[0].cdat,OneTime[1].cdat,8,0);

// Detect and set for fre
	OneTime[0].cdat[0] = 0xFF;	// mask bits (0's don't cares, 1's are matches)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	
	OneTime[1].cdat[0] = 0x0A;	// values (ORed)
	OneTime[1].cdat[1] = 0x03;
	OneTime[1].cdat[2] = 0x02;		// this byte must match
	index = RxMessageConfig1(0,0x314,OneTime[0].cdat,OneTime[1].cdat,3);
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0;
	OneTime[0].cdat[2] = 0xFF;	
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0xFF;	// clear
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0x50;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0x10;
	OneTime[1].cdat[6] = 0;	// write
	OneTime[1].cdat[7] = 0;
	RxMessageConfig2(index,0,0x332,OneTime[0].cdat,OneTime[1].cdat,8,0);

// Detect and set for ita
	OneTime[0].cdat[0] = 0xFF;	// mask bits (0's don't cares, 1's are matches)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	
	OneTime[1].cdat[0] = 0x0A;	// values (ORed)
	OneTime[1].cdat[1] = 0x03;
	OneTime[1].cdat[2] = 0x03;		// this byte must match
	index = RxMessageConfig1(0,0x314,OneTime[0].cdat,OneTime[1].cdat,3);
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0;
	OneTime[0].cdat[2] = 0xFF;	
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0xFF;	// clear
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0x50;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0x18;
	OneTime[1].cdat[6] = 0;		// write
	OneTime[1].cdat[7] = 0;
	RxMessageConfig2(index,0,0x332,OneTime[0].cdat,OneTime[1].cdat,8,0);

// Detect and set for jap
	OneTime[0].cdat[0] = 0xFF;	// mask bits (0's don't cares, 1's are matches)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	
	OneTime[1].cdat[0] = 0x0A;	// values (ORed)
	OneTime[1].cdat[1] = 0x03;
	OneTime[1].cdat[2] = 0x05;		// this byte must match
	index = RxMessageConfig1(0,0x314,OneTime[0].cdat,OneTime[1].cdat,3);
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0;
	OneTime[0].cdat[2] = 0xFF;	
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0xFF;	// clear
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0x50;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0x28;
	OneTime[1].cdat[6] = 0;	// write
	OneTime[1].cdat[7] = 0;
	RxMessageConfig2(index,0,0x332,OneTime[0].cdat,OneTime[1].cdat,8,0);

// Detect and set for chs
	OneTime[0].cdat[0] = 0xFF;	// mask bits (0's don't cares, 1's are matches)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	
	OneTime[1].cdat[0] = 0x0A;	// values (ORed)
	OneTime[1].cdat[1] = 0x03;
	OneTime[1].cdat[2] = 0x09;		// this byte must match
	index = RxMessageConfig1(0,0x314,OneTime[0].cdat,OneTime[1].cdat,3);
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0;
	OneTime[0].cdat[2] = 0xFF;	
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0xFF;	// clear
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0x50;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0x48;
	OneTime[1].cdat[6] = 0;	// write
	OneTime[1].cdat[7] = 0;
	RxMessageConfig2(index,0,0x332,OneTime[0].cdat,OneTime[1].cdat,8,0);

// Detect and set for cht
	OneTime[0].cdat[0] = 0xFF;	// mask bits (0's don't cares, 1's are matches)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	
	OneTime[1].cdat[0] = 0x0A;	// values (ORed)
	OneTime[1].cdat[1] = 0x03;
	OneTime[1].cdat[2] = 0x0E;	// this byte must match
	index = RxMessageConfig1(0,0x314,OneTime[0].cdat,OneTime[1].cdat,3);
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0;
	OneTime[0].cdat[2] = 0xFF;	
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0xFF;	// clear
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0x50;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0x90;
	OneTime[1].cdat[6] = 0;	// write
	OneTime[1].cdat[7] = 0;
	RxMessageConfig2(index,0,0x332,OneTime[0].cdat,OneTime[1].cdat,8,0);

	// Configure Relay's
	RelayCnf[3] = REL_IGN | 1 | REL_NOT;   	// relay 4 comes on with Ignition switch setting not 1
	//RelayCnf[2] = REL_BUTT | 1 | REL_TOG;	// relay 3 toggles with button 1
	//RelayCnf[1] = REL_RX | 0 | REL_PULSE;	// relay 2 with first (0) rxtrap configured
	//RelayCnf[0] = REL_TX | 3 | REL_PULSE;	// relay 1 with 4th (3) TxMesg configured

/*************************************************************************
	User running loop 
**************************************************************************/
	cnt = 0;
	counter =0;
	
	while(!(keyChange & 0xF))
	{
		
		if (RxCAN0)		// new message to process
		{
			for (index = 1 ; index < RX0BUFFERS && rxframe0[index].MsgNum>=RX0BUFFERS ; index++);

			if (index < RX0BUFFERS)
			{
				ProcessRx(0,index);
				rxframe0[index].MsgNum = 0;
			}
			RxCAN0 = 0;
		}
		if (RxCAN1)		// new message to process
		{
			error = CAN1readStatus();

			index = RX1BUFFERS;
			index1 = RX1BUFFERS;
			if (error & MCP_RX1IF)
				index1 = CAN1read_canMsg( MCP_READ_RX1 );        
			if (error & MCP_RX0IF)
				index = CAN1read_canMsg( MCP_READ_RX0 );        
			if (error = CAN1readRegister(MCP_EFLG))
			{
				ErrCnt++;
				AlterMessageByte(1,0x6E2,7,1,ErrCnt,0);

				if (error & MCP_EFLG_RX1OVR)
					CAN1modifyRegister(MCP_EFLG,MCP_EFLG_RX1OVR,0);
				if (error & MCP_EFLG_RX0OVR)
					CAN1modifyRegister(MCP_EFLG,MCP_EFLG_RX0OVR,0);
			}

			// clear int flags
			IE0 = 0;					// clear IE0 flag re-enabling interupt
			RxCAN1 = 0;					// clear my flag interupt occured
			EX0 = 1;					// enable CAN1 interupt

			if (index < RX1BUFFERS)				// process recieved message from Rx0
			{
				ProcessRx(1,index);		// CAN1 messages
			}
			if (index1 < RX1BUFFERS)			// process recieved message from Rx1
			{
				ProcessRx(1,index1);
			}
		}

		if (StartTicks + ONESEC < msticks )	// every second
		{
			StartTicks = msticks;
			LED1 ^= 1;

			for (index1 = 0 ; index1 < TXBUFFS && OneTime[index1].arbID ; index1++);
			if (index1 < TXBUFFS)				// empty one time available
			{
				OneTime[index1].cdat[0] = (U8)(keys>>24);
				OneTime[index1].cdat[1] = (U8)(keys>>16);
				OneTime[index1].cdat[2] = (U8)(keys>>8);
				OneTime[index1].cdat[3] = (U8)keys;
				OneTime[index1].cdat[4] = CAN1readStatus();
				OneTime[index1].cdat[5] = CAN1readRxStat();
				OneTime[index1].cdat[6] = CAN1readRegister(MCP_CANSTAT);
				OneTime[index1].cdat[7] = CAN1readRegister(MCP_CANINTF);
				AlterMessageByte(0,0x6EA,0,OneTime[index1].cdat,8,1);
			}

			// VIN
			for (index1 = 0 ; index1 < TXBUFFS && OneTime[index1].arbID ; index1++);
			if (index1 < TXBUFFS)				// empty one time available
			{
				switch (vin++)
				{
					case 0:
						OneTime[index1].cdat[0] = 0;
						for (index = 0 ; index < 7 ; index++)
							OneTime[index1].cdat[1+index] = VIN[index];
						break;
					case 1:
						OneTime[index1].cdat[0] = 1;
						for (index = 7 ; index < 14 ; index++)
							OneTime[index1].cdat[index-6] = VIN[index];
						break;
					case 2:
						OneTime[index1].cdat[0] = 2;
						for (index = 14 ; index < 17 ; index++)
							OneTime[index1].cdat[index-13] = VIN[index];
						vin = 0;
						break;
				}
				AlterMessageByte(0,0x3E0,0,OneTime[index1].cdat,8,0);
			}

			if (counter++ >= 9)	// test every 10 seconds alter 6E2 message
			{
				for (index1 = 0 ; index1 < TXBUFFS && OneTime[index1].arbID ; index1++);
				if (index1 < TXBUFFS)				// empty one time available
				{
					OneTime[index1].cdat[0] = cnt++;
					AlterMessageByte(1,0x6E2,6,OneTime[index1].cdat,1,2);
					counter = 0;
				}
			}
		}


		if (RelayChange)
			CheckRelay();

		// check for buttons pressed and process
		if (keyChange & 0xFFFFF00)	// At least one Buttons changed state
		{
			CheckButts();			// SimpleButton Check

			keyChange &= (U32)((BUTTONMASK<<8) & 0xFF);	// clear unused buttons
		}


		if (keyChange & 0xF0)		// Ignition change
		{
			// check if a relay is set to Ignition change
			for (index1 = 0 ; index1 < 4; index1++){
				if ((RelayCnf[index1] & 0xF00) == REL_IGN)	// Relay controled by IGN select
				{
					if ((RelayCnf[index1]&0xFF) == ((keys>>4)&0xF))	// matching Select
						RelayChange |= 1<<index1;			// flag for relay "on" change
					else
						RelayChange |= 0x10<<index1;		// flag for relay "off" change
				}
			}

			for (index1 = 0 ; index1 < TXBUFFS && OneTime[index1].arbID ; index1++);
			if (index1 < TXBUFFS)				// empty one time available
			{
				IGN_OFF = 0;			// default IGN_OFF flag to off
				switch(keys & 0xF0)
				{
					case 0x10:	// IGN_OFF
						// ID 190, alter byte 0,1 
						OneTime[0].cdat[0] = 0x00;	      	//ignition run, start not active  04 = run ;02=ACC ;0D= NormalStart, key in Ign, 						
						OneTime[0].cdat[1] = 0x01;		  	//key in ignition
						OneTime[0].cdat[2] = 0x00;	      	//ignition run, start not active  04 = run ;02=ACC ;0D= NormalStart, key in Ign, 						
						OneTime[0].cdat[3] = 0x00;	
						OneTime[0].sz = 4;
						OneTime[0].arbID = 0x122;

						//OneTime[index1].cdat[0] = 0x00;	      	//ignition run, start not active  04 = run ;02=ACC ;0D= NormalStart, key in Ign, 						
						//OneTime[index1].cdat[1] = 0x01;		  	
						//AlterMessageByte(0,0x122,0,OneTime[index1].cdat,2,0);

/*						// ID 98, alter bytes 2,3  
						OneTime[index1].cdat[0] = 0x0; 			// 0 speed			
						OneTime[index1].cdat[1] = 0x0;
						OneTime[index1].cdat[2] = 0x0;
						OneTime[index1].cdat[3] = 0x0;
						OneTime[index1].cdat[4] = 0x0; 			// 0 speed			
						OneTime[index1].cdat[5] = 0x0;
						OneTime[index1].cdat[6] = 0x0;
						OneTime[index1].cdat[7] = 0x0;
						AlterMessageByte(0,0x322,0,OneTime[index1].cdat,8,0); */
						IGN_OFF = 1;	// TURN on IGN_OFF flag
						IGN_OFF_FLG = 0;
						break;
					case 0x20:	// IGN_ACC
						// ID 190, alter byte 0,1 
						OneTime[index1].cdat[0] = 0x03;	      	//ignition run, start not active  04 = run ;02=ACC ;0D= NormalStart, key in Ign, 						
						OneTime[index1].cdat[1] = 0x02;		  	//key in ignition
						AlterMessageByte(0,0x122,0,OneTime[index1].cdat,2,0);

						// ID 98, alter bytes 2,3  
						OneTime[index1].cdat[0] = 0x0; 			// 0 speed			
						OneTime[index1].cdat[1] = 0x0;
						OneTime[index1].cdat[2] = 0x0;
						OneTime[index1].cdat[3] = 0x0;
						OneTime[index1].cdat[4] = 0x0; 			// 0 speed			
						OneTime[index1].cdat[5] = 0x0;
						OneTime[index1].cdat[6] = 0x0;
						OneTime[index1].cdat[7] = 0x0;
						AlterMessageByte(0,0x322,0,OneTime[index1].cdat,8,0);
						IGN_OFF_FLG = 1;
						break;

					case 0x30:	// IGN_START
						// ID 190, alter byte 0,1 
						OneTime[index1].cdat[0] = 0x05;	      	//ignition run, start not active  04 = run ;02=ACC ;0D= NormalStart, key in Ign, 						
						OneTime[index1].cdat[1] = 0x02;		  	//key in ignition
						AlterMessageByte(0,0x122,0,OneTime[index1].cdat,2,0);

						// ID 98, alter bytes 2,3  
						OneTime[index1].cdat[0] = 0x0; 			// 0 speed			
						OneTime[index1].cdat[1] = 0x0;
						OneTime[index1].cdat[2] = 0x0;
						OneTime[index1].cdat[3] = 0x0;
						OneTime[index1].cdat[4] = 0x0; 			// 0 speed			
						OneTime[index1].cdat[5] = 0x0;
						OneTime[index1].cdat[6] = 0x0;
						OneTime[index1].cdat[7] = 0x0;
						AlterMessageByte(0,0x322,0,OneTime[index1].cdat,8,0);
						IGN_OFF_FLG = 1;
						break;
					case 0x40:	// IGN_RUN_0
						// ID 190, alter byte 0,1 
						OneTime[index1].cdat[0] = 0x04;	      	//ignition run, start not active  04 = run ;02=ACC ;0D= NormalStart, key in Ign, 						
						OneTime[index1].cdat[1] = 0x02;		  	//key in ignition
						AlterMessageByte(0,0x122,0,OneTime[index1].cdat,2,0);

						// ID 98, alter bytes 2,3  
						OneTime[index1].cdat[0] = 0x0; 			// 0 speed			
						OneTime[index1].cdat[1] = 0x0;
						OneTime[index1].cdat[2] = 0x0;
						OneTime[index1].cdat[3] = 0x0;
						OneTime[index1].cdat[4] = 0x0; 			// 0 speed			
						OneTime[index1].cdat[5] = 0x0;
						OneTime[index1].cdat[6] = 0x0;
						OneTime[index1].cdat[7] = 0x0;
						AlterMessageByte(0,0x322,0,OneTime[index1].cdat,8,0);
						IGN_OFF_FLG = 1;
						break;
					case 0x50:	// IGN_RUN_5
					// ID 190, alter byte 0,1 
						OneTime[index1].cdat[0] = 0x04;	      	//ignition run, start not active  04 = run ;02=ACC ;0D= NormalStart, key in Ign, 						
						OneTime[index1].cdat[1] = 0x02;		  	//key in ignition
						AlterMessageByte(0,0x122,0,OneTime[index1].cdat,2,0);

						// ID 98, alter bytes 2,3  
						OneTime[index1].cdat[0] = 0x0; 			// 0 speed			
						OneTime[index1].cdat[1] = 0x0;
						OneTime[index1].cdat[2] = 0x08;
						OneTime[index1].cdat[3] = 0x0C;
						OneTime[index1].cdat[4] = 0x0; 			// 0 speed			
						OneTime[index1].cdat[5] = 0x0;
						OneTime[index1].cdat[6] = 0x0;
						OneTime[index1].cdat[7] = 0x0;
						AlterMessageByte(0,0x322,0,OneTime[index1].cdat,8,0);
						IGN_OFF_FLG = 1;
						break;
					case 0x60:	// IGN_RUN_10
					// ID 190, alter byte 0,1 
						OneTime[index1].cdat[0] = 0x04;	      	//ignition run, start not active  04 = run ;02=ACC ;0D= NormalStart, key in Ign, 						
						OneTime[index1].cdat[1] = 0x02;		  	//key in ignition
						AlterMessageByte(0,0x122,0,OneTime[index1].cdat,2,0);

						// ID 98, alter bytes 2,3  
						OneTime[index1].cdat[0] = 0x0; 			// 0 speed			
						OneTime[index1].cdat[1] = 0x0;
						OneTime[index1].cdat[2] = 0x04;
						OneTime[index1].cdat[3] = 0x06;
						OneTime[index1].cdat[4] = 0x0; 			// 0 speed			
						OneTime[index1].cdat[5] = 0x0;
						OneTime[index1].cdat[6] = 0x0;
						OneTime[index1].cdat[7] = 0x0;
						AlterMessageByte(0,0x322,0,OneTime[index1].cdat,8,0);
						IGN_OFF_FLG = 1;
						break;
					
					case 0x70:
						break;

					case 0x80:
						break;
				}

			keyChange &= ~0xF0;		// clear Ignition nibble
			}
		}
		
	}	
	
}
void User3()	// program select 3 - Atlantis
{	

	xdata U32 StartTicks = 0;
	U8 index,index1,error,ErrCnt,counter,cnt,vin;
	bit IGN_OFF=1;

	char VIN[17] = "2C3CCABT6GH100096";
//	char VIN[17] = "12345678281234567";

	error = 0;	ErrCnt=0;

/***************************************************************************************
****************************************************************************************
	CAUTION WATCH XDATA BOUNDARIES.  NOT AUTOMATIC ITS UP TO YOU!!!!!!
****************************************************************************************
	// CODE CURRENTLY USES 809 xdata bytes.  THEREFORE STARTING CAN BE AT 0x32A
	// watch at compile the xdata bytes used. if xdata > 810d need to change below
	// xdata space ends at 0xFFF.
***************************************************************************************/
//	Buttons = (BUTTABLE xdata *)0x32A;	// 20 buttons takes 32Abytes => 0x32A-0x4E1
	ButtCnt = 0;						// each Button = 20 bytes

//	Mesgs = (MSGTABLE xdata *)0x4E2;	// 0x4E2-0x93F -> space for 43 Tx messages 
	MesgCnt = 0;						// each Mesgs = 26 bytes

//	RxMesg = (RxMESG xdata *)0x940;		// 0x940-0xFF8 -> space for 43 Rx messages 
	RxCnt = 0;							// each RxMesg = 40 bytes

// AUTOMATED above some,
// Make Sure XDATASPACE define follows compile output value  
#define XDATASPACE 810		// starting free address

/****************************************************************************************
// Currently (809 xdata start) you get 43 Tx messages and 43 Rx message traps.  If you 
// need more of one and less of the other you can manipulate the starting numbers below
// to get it to work.
// also note RX0 buff size is 32 messages on CAN0, and RX1 buff size is 16 for CAN1
***************************************************************************************/
	MaxMesgs = 45;			// number of Tx message configs
	MaxRxTraps = 45;		// number of Rx message configs
	while(!MemManage(MAXBUTTONS,MaxMesgs,MaxRxTraps,XDATASPACE))	// 20 button configs  
	{
		MaxRxTraps--;		// reduce Message buffers till it fits.
		if (!MemManage(MAXBUTTONS,MaxMesgs,MaxRxTraps,XDATASPACE))
			MaxMesgs--;		// reduce Message buffers till it fits
	}
/**************************************************************************************/


/*************************************************************************
	Repeating message setups
	example below-
	message ID 0x6E2 is created at rate of 500mSec on port 1 (C). With 8 bytes 
	of data, shown are it's defaults.
**************************************************************************/


/*	/// IPC_DISPLAY_INFO
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].sz = 3;
	OneTime[0].arbID = 0x092 | 0;
//	AddMessage(0,0x092,100,3,OneTime[0].cdat) */

/*	/// U_CONN_REQ - ETM
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	AddMessage(0,0x2EC,1000,5,OneTime[0].cdat); */

	/// SWC
/*	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].sz = 4;
	OneTime[0].arbID = 0x2EE | 0;
//	AddMessage(0,0x2EE,500,4,OneTime[0].cdat); */
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	AddMessage(0,0x2EE,250,4,OneTime[0].cdat); 

	/// STATUS_BH_BCM1
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0x20;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	AddMessage(0,0x356,250,8,OneTime[0].cdat);
//	AddMessage(0,0x356,1000,8,OneTime[0].cdat);

/*
/// SDW_INFO
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].sz = 6;
	OneTime[0].arbID = 0x3B4 | 0;
//	AddMessage(0,0x3B4,20000,6,OneTime[0].cdat); 
	/// PARK_INFO
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].sz = 6;
	OneTime[0].arbID = 0x3B6 | 0;
//	AddMessage(0,0x3B6,20000,6,OneTime[0].cdat); 
	
*/
/*	/// IPC_ACK
	OneTime[0].cdat[0] = 89;
	OneTime[0].sz = 1;
	OneTime[0].arbID = 0x3DA | 0; */

	/// STATUS_CCAN1
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0;
	AddMessage(0,0x3DC,100,7,OneTime[0].cdat);

	/// STATUS_CCAN2
	OneTime[0].cdat[0] = 0x3E;
	OneTime[0].cdat[1] = 0x1E;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0x1E;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0x03;
	OneTime[0].cdat[5] = 0x01;
	OneTime[0].cdat[6] = 0x91;
	OneTime[0].cdat[7] = 0x01;
	AddMessage(0,0x3DE,100,8,OneTime[0].cdat);

	/// VIN
	vin = 0;
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
//	AddMessage(0,0x3E0,100,8,OneTime[0].cdat);
	AddMessage(0,0x3E0,1000,8,OneTime[0].cdat);

/*	/// STATUS_CCAN3
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0x04;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0x3C;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0x03;
	OneTime[0].cdat[7] = 0;
	AddMessage(0,0x3E2,100,8,OneTime[0].cdat); */
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0x02;
	AddMessage(0,0x3E2,100,8,OneTime[0].cdat);

	/// STATUS_CCAN4
	OneTime[0].cdat[0] = 0x89;
	OneTime[0].cdat[1] = 0xDA;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0x98;
	OneTime[0].cdat[3] = 0x06;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	AddMessage(0,0x3E4,100,8,OneTime[0].cdat);

	/// STATUS_CCAN5
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0;
//	OneTime[0].cdat[7] = 0;
	OneTime[0].cdat[7] = 0x02;
//	AddMessage(0,0x3E6,1000,8,OneTime[0].cdat);
	AddMessage(0,0x3E6,100,8,OneTime[0].cdat);

	/// CLIMATIC_PANEL
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	AddMessage(0,0x41A,1000,8,OneTime[0].cdat);

/*	/// STATUS_BH_IPC
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	AddMessage(0,0x468,500,8,OneTime[0].cdat); */

	/// STATUS_BH_BCM2
/*	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0x80;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0xFF;
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xC0;
	OneTime[0].cdat[5] = 0x14;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	AddMessage(0,0x46C,500,8,OneTime[0].cdat); */
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0x20;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0x69;
	OneTime[0].cdat[3] = 0x2F;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0x02;
	AddMessage(0,0x46C,1000,8,OneTime[0].cdat);

/*	/// HAPTIC1
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	OneTime[0].sz = 8;
	OneTime[0].arbID = 0x4F6 | 0;
//	AddMessage(0,0x4F6,20000,8,OneTime[0].cdat); */

	/// STATUS_LIN
	OneTime[0].cdat[0] = 0x08;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	AddMessage(0,0x5B6,1000,8,OneTime[0].cdat); 


	/// IPC_VEHICLE_SETUP
/*	OneTime[0].cdat[0] = 0x02;
	OneTime[0].cdat[1] = 0x65;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0x68;
	OneTime[0].cdat[3] = 0x40;
	OneTime[0].cdat[4] = 0x05;
	OneTime[0].cdat[5] = 0x19;
	OneTime[0].cdat[6] = 0x90;
	OneTime[0].cdat[7] = 0x80;
	AddMessage(0,0x5BC,1000,8,OneTime[0].cdat); */
	OneTime[0].cdat[0] = 0x0B;
	OneTime[0].cdat[1] = 0x65;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0x68;
	OneTime[0].cdat[3] = 0x40;
	OneTime[0].cdat[4] = 0x05;
	OneTime[0].cdat[5] = 0x19;
	OneTime[0].cdat[6] = 0x90;
	OneTime[0].cdat[7] = 0x80;
	AddMessage(0,0x5BC,1000,8,OneTime[0].cdat);

/*	/// STATUS_TELEMATIC
	OneTime[0].cdat[0] = 0x10;
	OneTime[0].cdat[1] = 0x24;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	AddMessage(0,0x5BE,1000,8,OneTime[0].cdat); */

/*	/// TRIP_A
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	AddMessage(0,0x5C0,1000,8,OneTime[0].cdat); */

	/// BCM_VPS2
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	AddMessage(0,0x5C2,1000,3,OneTime[0].cdat); 

/*	/// TRIP_B
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	AddMessage(0,0x5C4,1000,8,OneTime[0].cdat); */

	/// IPC_VEHICLE_SETUP2
	OneTime[0].cdat[0] = 0x65;
	OneTime[0].cdat[1] = 0x43;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0x40;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0x02;
	OneTime[0].cdat[5] = 0x01;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	AddMessage(0,0x5CE,1000,8,OneTime[0].cdat); 

/*	/// CTRL_AMP
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0x14;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0x50;
	OneTime[0].cdat[3] = 0x50;
	OneTime[0].cdat[4] = 0x52;
	OneTime[0].cdat[5] = 0x80;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	AddMessage(0,0x5C6,1000,8,OneTime[0].cdat); */

/*	/// TELEMATIC_CLIMATE_SETUP
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	AddMessage(0,0x5C8,1000,4,OneTime[0].cdat); */

	/// BCM_IGNITION_ON_COUNTER
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	AddMessage(0,0x75C,1000,2,OneTime[0].cdat); 

	/// TRIP_A_B
/*	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	AddMessage(0,0x760,1000,6,OneTime[0].cdat); */
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0x03;
	OneTime[0].cdat[3] = 0xE8;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	AddMessage(0,0x760,1000,6,OneTime[0].cdat); 

/*	/// SERVICEINFO2
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	AddMessage(0,0x762,1000,8,OneTime[0].cdat);	*/

	/// TIME_DATE
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	AddMessage(0,0x764,1000,6,OneTime[0].cdat);


/*	/// CLUSTER_INDICATION
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	OneTime[0].sz = 8;
	OneTime[0].arbID = 0x7B6 | 0; */
	
	/// U_CONN_SEED
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].sz = 4;
	OneTime[0].arbID = 0x7B8 | 0; 

//	AddMessage(0,0x7B8,1000,4,OneTime[0].cdat); 

	/// STATUS_CCAN6
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	AddMessage(0,0x7C0,2000,3,OneTime[0].cdat);

/*	/// CFG_DATA_CODE_REQUEST
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	AddMessage(0,0x1E114000,20000,6,OneTime[0].cdat); */

/*	/// CFG_DATA_CODE_REQUEST
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].sz = 6;
	OneTime[0].arbID = 0x1E114000 | 0; */

	/// build repetative message 1E340000 on CAN0 (IGN)
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0x02;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0xFF;
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0xFF;
	AddMessage(0,0x1E340000,1000,6,OneTime[0].cdat); 

/*	/// build repetative message 1E340000 on CAN0 (IGN) - ETM
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0xCE;	// using OneTime structure as temp storage
	AddMessage(0,0x1E340024,1000,2,OneTime[0].cdat); */

/*************************************************************************
 	Button setup
	example below -
	pressing button 1 causes byte 5 of message 0x6E2 to be altered to 0xAA 
	for the next 5 outputs. The mask clears byte 5 leaving the rest alone.  
	The value (0xAA) is ORed into byte 5.
**************************************************************************/
	#define BUTTONMASK 0xF	// only button 1 defined (bit 1)	
	
	//Vol_Up
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;		// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0x01;
//U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
	SimpleButtonConfig(19,0,0x2EE,OneTime[0].cdat,OneTime[1].cdat,1);
	 
	
	//Vol_Dwn
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0x04;
//U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
	SimpleButtonConfig(9,0,0x2EE,OneTime[0].cdat,OneTime[1].cdat,1);

	//Mode
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0x01;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	SimpleButtonConfig(18,0,0x2EE,OneTime[0].cdat,OneTime[1].cdat,1);


	//Presets
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0x04;
	OneTime[1].cdat[3] = 0;
	SimpleButtonConfig(8,0,0x2EE,OneTime[0].cdat,OneTime[1].cdat,1);

/*	// Presets
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;		// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;		// values (ORed)
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;	
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;	
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0x08;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
	SimpleButtonConfig(8,0,0x41A,OneTime[0].cdat,OneTime[1].cdat,1); */

	// Seek Up
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0x40;
	SimpleButtonConfig(17,0,0x2EE,OneTime[0].cdat,OneTime[1].cdat,1);

	//Seek Down
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0;		// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0x01;
	OneTime[1].cdat[3] = 0;
	SimpleButtonConfig(7,0,0x2EE,OneTime[0].cdat,OneTime[1].cdat,1);

	// Phone Pickup
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0;		// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0x10;
	OneTime[1].cdat[3] = 0;
	SimpleButtonConfig(16,0,0x2EE,OneTime[0].cdat,OneTime[1].cdat,1);

	// Phone Hangup
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0;		// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0x40;
	OneTime[1].cdat[3] = 0;
	SimpleButtonConfig(6,0,0x2EE,OneTime[0].cdat,OneTime[1].cdat,1);

	// VR
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0x04;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	SimpleButtonConfig(15,0,0x2EE,OneTime[0].cdat,OneTime[1].cdat,1);

	// Screen Off
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;		// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;		// values (ORed)
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;	
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;	
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0x04;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
	SimpleButtonConfig(5,0,0x41A,OneTime[0].cdat,OneTime[1].cdat,1); 
	
	
/*	// Radio Off
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;		// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;		// values (ORed)
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;	
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;	
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0x10;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
	SimpleButtonConfig(5,0,0x41A,OneTime[0].cdat,OneTime[1].cdat,1); 	*/

	//Mute
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0x10;
	SimpleButtonConfig(14,0,0x2EE,OneTime[0].cdat,OneTime[1].cdat,1);

	// ICS Buttons ------------------------------------------------------------------------------------

	// Back
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;		// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;		// values (ORed)
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;	
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;	
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0x02;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
	SimpleButtonConfig(4,0,0x41A,OneTime[0].cdat,OneTime[1].cdat,1); 

	// Browse/Enter
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;		// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;		// values (ORed)
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;	
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;	
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0x08;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
	SimpleButtonConfig(13,0,0x41A,OneTime[0].cdat,OneTime[1].cdat,1);

	// Engineering Mode
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;		// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0;			// values (ORed)
	OneTime[0].cdat[5] = 0xFF;
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;	
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;	
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0x30;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
	SimpleButtonConfig(12,0,0x41A,OneTime[0].cdat,OneTime[1].cdat,7);

	// Dealer Mode
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;		// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0;			// values (ORed)
	OneTime[0].cdat[5] = 0xFF;
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;	
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;	
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0x31;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
	SimpleButtonConfig(2,0,0x41A,OneTime[0].cdat,OneTime[1].cdat,7);

	// Night Mode
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;		// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;		// values (ORed)
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;	
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;	
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0x02;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
	SimpleButtonConfig(11,0,0x46C,OneTime[0].cdat,OneTime[1].cdat,0);

	// Day Mode
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;		// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;		// values (ORed)
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;	
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;	
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0x00;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
	SimpleButtonConfig(1,0,0x46C,OneTime[0].cdat,OneTime[1].cdat,0);


	// Phone Pickup Long Press
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0;		// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0x10;
	OneTime[1].cdat[3] = 0;
	SimpleButtonConfig(10,0,0x2EE,OneTime[0].cdat,OneTime[1].cdat,10);

	// VR Long Press
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0x04;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	SimpleButtonConfig(0,0,0x2EE,OneTime[0].cdat,OneTime[1].cdat,10);


	/*
	// Screen Off
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0;		// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;		// values (ORed)
	OneTime[0].cdat[5] = 0xFF;
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;	
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0x0C;	
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
	SimpleButtonConfig(5,0,0x41A,OneTime[0].cdat,OneTime[1].cdat,1);
	
	// Mute
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;		// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0xFF;		// values (ORed)
	OneTime[0].cdat[5] = 0xFF;
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;	
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;	
	OneTime[1].cdat[3] = 0xC0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
	// Back
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;		// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0xFF;		// values (ORed)
	OneTime[0].cdat[5] = 0xFF;
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0;	
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;	
	OneTime[1].cdat[3] = 0xC0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
	SimpleButtonConfig(4,0,0x41A,OneTime[0].cdat,OneTime[1].cdat,1);
	*/
 
	/*
	// Screenshot
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0x04;
	SimpleButtonConfig(3,0,0x2EE,OneTime[0].cdat,OneTime[1].cdat,1);
	// ENG. Mode
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0x04;
	SimpleButtonConfig(12,0,0x2EE,OneTime[0].cdat,OneTime[1].cdat,1);
	// Dealer Mode
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0x04;
	SimpleButtonConfig(2,0,0x2EE,OneTime[0].cdat,OneTime[1].cdat,1);
	// 911
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0x04;
	SimpleButtonConfig(11,0,0x2EE,OneTime[0].cdat,OneTime[1].cdat,1);
	// Assist
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0x04;
	SimpleButtonConfig(1,0,0x2EE,OneTime[0].cdat,OneTime[1].cdat,1);
	// Door Ajar
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0x04;
	SimpleButtonConfig(10,0,0x2EE,OneTime[0].cdat,OneTime[1].cdat,1);
	// Theft Alarm
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0x04;
	SimpleButtonConfig(0,0,0x2EE,OneTime[0].cdat,OneTime[1].cdat,1);
	*/


/*************************************************************************
 	RxMessage filters / traps
	Below example:
	First half, Watches for ID 0x6E8 on can port 1 (C).  Only byte 3 matters 
	as the mask is set to 0's for all else.  The matching value required in  
	byte 3 is 0x58. 8 bytes expected.
	Second half, configures output message 0x6E3 on port 1 (C). 8 bytes in 
	count.  mask is ANDed with cyclic message and ORed with value, but in 
	this example there is no cyclic message, therefor mask is ignored and 
	value is just output as the data 1 time. Take care to use the index value
	returned from the first function as input parameter to 2nd function
**************************************************************************/

	OneTime[0].cdat[0] = 0;		// mask bits (0's don't cares, 1's are matches)
	OneTime[0].cdat[1] = 0;
	OneTime[0].cdat[2] = 0;		// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;	// this byte matters
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;	 
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	OneTime[1].cdat[0] = 0;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0x58;	// selector switches to x58 causes triggers
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
// configure Rx filter, look for this message match
//U8 RxMessageConfig1(U8 port,U32 ID,U8 mask[8],U8 value[8],U8 count);
	index = RxMessageConfig1(1,0x6E8,OneTime[0].cdat,OneTime[1].cdat,8);
	OneTime[0].cdat[0] = 0xFF;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0xFF;
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0x88;	// values (ORed)
	OneTime[1].cdat[1] = 0x99;
	OneTime[1].cdat[2] = 0xAA;
	OneTime[1].cdat[3] = 0xBB;
	OneTime[1].cdat[4] = 0xCC;
	OneTime[1].cdat[5] = 0xDD;
	OneTime[1].cdat[6] = 0xEE;
	OneTime[1].cdat[7] = 0xFF;
// configure Tx output caused by above Recieved message
//void RxMessageConfig2(U8 index,U8 outport,U32 outID,U8 outmask[8],U8 outvalue[8],U8 cnt,U8 altercnt);
	RxMessageConfig2(index,1,0x6E3,OneTime[0].cdat,OneTime[1].cdat,8,1);
	
	//  Check for any language change requests

	//  Check for request to switch to CHS
	OneTime[0].cdat[0] = 0xFF;		// mask bits (0's don't cares, 1's are matches)
	OneTime[0].cdat[1] = 0;
	OneTime[0].cdat[2] = 0;		// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0;	// this byte matters
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;	 
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	OneTime[1].cdat[0] = 0x0E;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;	// selector switches to x58 causes triggers
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
// configure Rx filter, look for this message match
//U8 RxMessageConfig1(U8 port,U32 ID,U8 mask[8],U8 value[8],U8 count);
	index = RxMessageConfig1(0,0x09E,OneTime[0].cdat,OneTime[1].cdat,8);
	OneTime[0].cdat[0] = 0;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;	
	OneTime[0].cdat[5] = 0xFF;
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0x0E;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
// configure Tx output caused by above Recieved message
//void RxMessageConfig2(U8 index,U8 outport,U32 outID,U8 outmask[8],U8 outvalue[8],U8 cnt,U8 altercnt);
	RxMessageConfig2(index,0,0x5BC,OneTime[0].cdat,OneTime[1].cdat,8,0);


	//  Check for request to switch to JAP
	OneTime[0].cdat[0] = 0xFF;		// mask bits (0's don't cares, 1's are matches)
	OneTime[0].cdat[1] = 0;
	OneTime[0].cdat[2] = 0;		// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0;	// this byte matters
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;	 
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	OneTime[1].cdat[0] = 0x0F;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;	// selector switches to x58 causes triggers
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
// configure Rx filter, look for this message match
//U8 RxMessageConfig1(U8 port,U32 ID,U8 mask[8],U8 value[8],U8 count);
	index = RxMessageConfig1(0,0x09E,OneTime[0].cdat,OneTime[1].cdat,8);
	OneTime[0].cdat[0] = 0;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;	
	OneTime[0].cdat[5] = 0xFF;
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0x0F;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
// configure Tx output caused by above Recieved message
//void RxMessageConfig2(U8 index,U8 outport,U32 outID,U8 outmask[8],U8 outvalue[8],U8 cnt,U8 altercnt);
	RxMessageConfig2(index,0,0x5BC,OneTime[0].cdat,OneTime[1].cdat,8,0);


//  Check for request to switch to KOR
	OneTime[0].cdat[0] = 0xFF;		// mask bits (0's don't cares, 1's are matches)
	OneTime[0].cdat[1] = 0;
	OneTime[0].cdat[2] = 0;		// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0;	// this byte matters
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;	 
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	OneTime[1].cdat[0] = 0x10;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;	// selector switches to x58 causes triggers
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
// configure Rx filter, look for this message match
//U8 RxMessageConfig1(U8 port,U32 ID,U8 mask[8],U8 value[8],U8 count);
	index = RxMessageConfig1(0,0x09E,OneTime[0].cdat,OneTime[1].cdat,8);
	OneTime[0].cdat[0] = 0;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;	
	OneTime[0].cdat[5] = 0xFF;
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0x10;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
// configure Tx output caused by above Recieved message
//void RxMessageConfig2(U8 index,U8 outport,U32 outID,U8 outmask[8],U8 outvalue[8],U8 cnt,U8 altercnt);
	RxMessageConfig2(index,0,0x5BC,OneTime[0].cdat,OneTime[1].cdat,8,0);


	//  Check for request to switch from ENG (US)
	OneTime[0].cdat[0] = 0xFF;		// mask bits (0's don't cares, 1's are matches)
	OneTime[0].cdat[1] = 0;
	OneTime[0].cdat[2] = 0;		// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0;	// this byte matters
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;	 
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	OneTime[1].cdat[0] = 0x0B;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;	// selector switches to x58 causes triggers
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
// configure Rx filter, look for this message match
//U8 RxMessageConfig1(U8 port,U32 ID,U8 mask[8],U8 value[8],U8 count);
	index = RxMessageConfig1(0,0x09E,OneTime[0].cdat,OneTime[1].cdat,8);
	OneTime[0].cdat[0] = 0;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0xFF;
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0x0B;	// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
// configure Tx output caused by above Recieved message
//void RxMessageConfig2(U8 index,U8 outport,U32 outID,U8 outmask[8],U8 outvalue[8],U8 cnt,U8 altercnt);
	RxMessageConfig2(index,0,0x5BC,OneTime[0].cdat,OneTime[1].cdat,8,0);


	//  Check for request to switch to Spanish
	OneTime[0].cdat[0] = 0xFF;		// mask bits (0's don't cares, 1's are matches)
	OneTime[0].cdat[1] = 0;
	OneTime[0].cdat[2] = 0;		// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0;	// this byte matters
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;	 
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	OneTime[1].cdat[0] = 0x03;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;	// selector switches to x58 causes triggers
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
// configure Rx filter, look for this message match
//U8 RxMessageConfig1(U8 port,U32 ID,U8 mask[8],U8 value[8],U8 count);
	index = RxMessageConfig1(0,0x09E,OneTime[0].cdat,OneTime[1].cdat,8);
	OneTime[0].cdat[0] = 0;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0xFF;
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0x03;	// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
// configure Tx output caused by above Recieved message
//void RxMessageConfig2(U8 index,U8 outport,U32 outID,U8 outmask[8],U8 outvalue[8],U8 cnt,U8 altercnt);
	RxMessageConfig2(index,0,0x5BC,OneTime[0].cdat,OneTime[1].cdat,8,0);

	//  Check for request to switch to French
	OneTime[0].cdat[0] = 0xFF;		// mask bits (0's don't cares, 1's are matches)
	OneTime[0].cdat[1] = 0;
	OneTime[0].cdat[2] = 0;		// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0;	// this byte matters
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;	 
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	OneTime[1].cdat[0] = 0x04;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;	// selector switches to x58 causes triggers
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
// configure Rx filter, look for this message match
//U8 RxMessageConfig1(U8 port,U32 ID,U8 mask[8],U8 value[8],U8 count);
	index = RxMessageConfig1(0,0x09E,OneTime[0].cdat,OneTime[1].cdat,8);
	OneTime[0].cdat[0] = 0;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0xFF;
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0x04;	// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
// configure Tx output caused by above Recieved message
//void RxMessageConfig2(U8 index,U8 outport,U32 outID,U8 outmask[8],U8 outvalue[8],U8 cnt,U8 altercnt);
	RxMessageConfig2(index,0,0x5BC,OneTime[0].cdat,OneTime[1].cdat,8,0);




//  Check for request to switch to Canadian
	OneTime[0].cdat[0] = 0xFF;		// mask bits (0's don't cares, 1's are matches)
	OneTime[0].cdat[1] = 0;
	OneTime[0].cdat[2] = 0;		// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0;	// this byte matters
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;	 
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	OneTime[1].cdat[0] = 0x0C;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;	// selector switches to x58 causes triggers
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
// configure Rx filter, look for this message match
//U8 RxMessageConfig1(U8 port,U32 ID,U8 mask[8],U8 value[8],U8 count);
	index = RxMessageConfig1(0,0x09E,OneTime[0].cdat,OneTime[1].cdat,8);
	OneTime[0].cdat[0] = 0;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0xFF;
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0x0C;	// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
// configure Tx output caused by above Recieved message
//void RxMessageConfig2(U8 index,U8 outport,U32 outID,U8 outmask[8],U8 outvalue[8],U8 cnt,U8 altercnt);
	RxMessageConfig2(index,0,0x5BC,OneTime[0].cdat,OneTime[1].cdat,8,0);

//  Check for request to switch to Mexican
	OneTime[0].cdat[0] = 0xFF;		// mask bits (0's don't cares, 1's are matches)
	OneTime[0].cdat[1] = 0;
	OneTime[0].cdat[2] = 0;		// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0;	// this byte matters
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;	 
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	OneTime[1].cdat[0] = 0x0A;		// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;	// selector switches to x58 causes triggers
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
// configure Rx filter, look for this message match
//U8 RxMessageConfig1(U8 port,U32 ID,U8 mask[8],U8 value[8],U8 count);
	index = RxMessageConfig1(0,0x09E,OneTime[0].cdat,OneTime[1].cdat,8);
	OneTime[0].cdat[0] = 0;	// mask bits (ANDed)
	OneTime[0].cdat[1] = 0xFF;
	OneTime[0].cdat[2] = 0xFF;	// using OneTime structure as temp storage
	OneTime[0].cdat[3] = 0xFF;
	OneTime[0].cdat[4] = 0xFF;
	OneTime[0].cdat[5] = 0xFF;
	OneTime[0].cdat[6] = 0xFF;
	OneTime[0].cdat[7] = 0xFF;
	OneTime[1].cdat[0] = 0x0A;	// values (ORed)
	OneTime[1].cdat[1] = 0;
	OneTime[1].cdat[2] = 0;
	OneTime[1].cdat[3] = 0;
	OneTime[1].cdat[4] = 0;
	OneTime[1].cdat[5] = 0;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
// configure Tx output caused by above Recieved message
//void RxMessageConfig2(U8 index,U8 outport,U32 outID,U8 outmask[8],U8 outvalue[8],U8 cnt,U8 altercnt);
	RxMessageConfig2(index,0,0x5BC,OneTime[0].cdat,OneTime[1].cdat,8,0);





	// Configure Relay's
	RelayCnf[3] = REL_IGN | 1 | REL_NOT;   	// relay 4 comes on with Ignition switch setting not 1
//	RelayCnf[2] = REL_BUTT | 1 | REL_TOG;	// relay 3 toggles with button 1
//	RelayCnf[1] = REL_RX | 0 | REL_PULSE;	// relay 2 with first (0) rxtrap configured
//	RelayCnf[0] = REL_TX | 3 | REL_PULSE;	// relay 1 with 4th (3) TxMesg configured

/*************************************************************************
	User running loop 
**************************************************************************/
	cnt = 0;
	counter =0;
	while(!(keyChange & 0xF)) 
	{
		if (RxCAN0)		// new message to process
		{
			for (index = 1 ; index < RX0BUFFERS && rxframe0[index].MsgNum>=RX0BUFFERS ; index++);

			if (index < RX0BUFFERS)
			{
				ProcessRx(0,index);
				rxframe0[index].MsgNum = 0;
			}
			RxCAN0 = 0;
		}
		if (RxCAN1)		// new message to process
		{
			error = CAN1readStatus();

			index = RX1BUFFERS;
			index1 = RX1BUFFERS;
			if (error & MCP_RX1IF)
				index1 = CAN1read_canMsg( MCP_READ_RX1 );        
			if (error & MCP_RX0IF)
				index = CAN1read_canMsg( MCP_READ_RX0 );        
			if (error = CAN1readRegister(MCP_EFLG))
			{
				ErrCnt++;
				AlterMessageByte(1,0x6E2,7,1,ErrCnt,0);

				if (error & MCP_EFLG_RX1OVR)
					CAN1modifyRegister(MCP_EFLG,MCP_EFLG_RX1OVR,0);
				if (error & MCP_EFLG_RX0OVR)
					CAN1modifyRegister(MCP_EFLG,MCP_EFLG_RX0OVR,0);
			}

			// clear int flags
			IE0 = 0;					// clear IE0 flag re-enabling interupt
			RxCAN1 = 0;					// clear my flag interupt occured
			EX0 = 1;					// enable CAN1 interupt

			if (index < RX1BUFFERS)				// process recieved message from Rx0
			{
				ProcessRx(1,index);		// CAN1 messages
			}
			if (index1 < RX1BUFFERS)			// process recieved message from Rx1
			{
				ProcessRx(1,index1);
			}
		}

		if (StartTicks + ONESEC < msticks )	// every second
		{
			StartTicks = msticks;
			LED1 ^= 1;

			for (index1 = 0 ; index1 < TXBUFFS && OneTime[index1].arbID ; index1++);
			if (index1 < TXBUFFS)				// empty one time available
			{
				OneTime[index1].cdat[0] = (U8)(keys>>24);
				OneTime[index1].cdat[1] = (U8)(keys>>16);
				OneTime[index1].cdat[2] = (U8)(keys>>8);
				OneTime[index1].cdat[3] = (U8)keys;
				OneTime[index1].cdat[4] = CAN1readStatus();
				OneTime[index1].cdat[5] = CAN1readRxStat();
				OneTime[index1].cdat[6] = CAN1readRegister(MCP_CANSTAT);
				OneTime[index1].cdat[7] = CAN1readRegister(MCP_CANINTF);
				AlterMessageByte(0,0x6EA,0,OneTime[index1].cdat,8,1);
			}

			// VIN
			for (index1 = 0 ; index1 < TXBUFFS && OneTime[index1].arbID ; index1++);
			if (index1 < TXBUFFS)				// empty one time available
			{
				switch (vin++)
				{
					case 0:
						OneTime[index1].cdat[0] = 0;
						for (index = 0 ; index < 7 ; index++)
							OneTime[index1].cdat[1+index] = VIN[index];
						break;
					case 1:
						OneTime[index1].cdat[0] = 1;
						for (index = 7 ; index < 14 ; index++)
							OneTime[index1].cdat[index-6] = VIN[index];
						break;
					case 2:
						OneTime[index1].cdat[0] = 2;
						for (index = 14 ; index < 21 ; index++)
							OneTime[index1].cdat[index-13] = VIN[index];
						vin = 0;
						break;
				}
				AlterMessageByte(0,0x3E0,0,OneTime[index1].cdat,8,0);
			}

			if (counter++ >= 9)	// test every 10 seconds alter 6E2 message
			{
				for (index1 = 0 ; index1 < TXBUFFS && OneTime[index1].arbID ; index1++);
				if (index1 < TXBUFFS)				// empty one time available
				{
					OneTime[index1].cdat[0] = cnt++;
					AlterMessageByte(1,0x6E2,6,OneTime[index1].cdat,1,2);
					counter = 0;
				}
			}
		}


		if (RelayChange)
			CheckRelay();

		// check for buttons pressed and process
		if (keyChange & 0xFFFFF00)	// At least one Buttons changed state
		{
			CheckButts();			// SimpleButton Check

			keyChange &= (U32)((BUTTONMASK<<8)&0xFF);	// clear unused buttons
		}


		if (keyChange & 0xF0)		// Ignition change
		{
			// check if a relay is set to Ignition change
			for (index1 = 0 ; index1 < 4; index1++){
				if ((RelayCnf[index1] & 0xF00) == REL_IGN)	// Relay controled by IGN select
				{
					if ((RelayCnf[index1]&0xFF) == ((keys>>4)&0xF))	// matching Select
						RelayChange |= 1<<index1;			// flag for relay "on" change
					else
						RelayChange |= 0x10<<index1;		// flag for relay "off" change
				}
			}

			for (index1 = 0 ; index1 < TXBUFFS && OneTime[index1].arbID ; index1++);
			if (index1 < TXBUFFS)				// empty one time available
			{
				IGN_OFF = 0;			// default IGN_OFF flag to off
				switch(keys & 0xF0)
				{
					case 0x10:	// IGN_OFF
						// ID 356, alter byte 2	
						
						OneTime[0].cdat[0] = 0;
						OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
						OneTime[0].cdat[2] = 0x20;
						OneTime[0].cdat[3] = 0;
						OneTime[0].cdat[4] = 0;
						OneTime[0].cdat[5] = 0;
						OneTime[0].cdat[6] = 0;
						OneTime[0].cdat[7] = 0;
						OneTime[0].sz = 8;
						OneTime[0].arbID = 0x356 | 0;

						OneTime[0].cdat[0] = 0;
						OneTime[0].cdat[1] = 0x20;	// using OneTime structure as temp storage
						OneTime[0].cdat[2] = 0x69;
						OneTime[0].cdat[3] = 0x2F;
						OneTime[0].cdat[4] = 0;
						OneTime[0].cdat[5] = 0;
						OneTime[0].cdat[6] = 0;
						OneTime[0].cdat[7] = 0x02;
						OneTime[0].sz = 8;
						OneTime[0].arbID = 0x46C | 0;
						
							  	
//						OneTime[index1].cdat[0] = 0;
//						OneTime[index1].cdat[1] = 0;
//						OneTime[index1].cdat[2] = 0x20;	
/*						OneTime[index1].cdat[3] = 0;
						OneTime[index1].cdat[4] = 0;
						OneTime[index1].cdat[5] = 0;
						OneTime[index1].cdat[6] = 0;
						OneTime[index1].cdat[7] = 0; */	  	
//						AlterMessageByte(0,0x356,0,OneTime[index1].cdat,3,0); 

						// ID 46C, alter byte 2  
//						OneTime[index1].cdat[0] = 0;			
//						OneTime[index1].cdat[1] = 0x20;
/*						OneTime[index1].cdat[2] = 0xFF;
						OneTime[index1].cdat[3] = 0xFF;
						OneTime[index1].cdat[4] = 0xC0;
						OneTime[index1].cdat[5] = 0x14;
						OneTime[index1].cdat[6] = 0;
						OneTime[index1].cdat[7] = 0; */
//						AlterMessageByte(0,0x46C,0,OneTime[index1].cdat,2,0); */
						IGN_OFF = 1;	// TURN on IGN_OFF flag
						IGN_OFF_FLG= 0;
						break;
					case 0x20:	// IGN_ACC
							// ID 356, alter byte 2
						OneTime[index1].cdat[0] = 0;
						OneTime[index1].cdat[1] = 0;		  	
						OneTime[index1].cdat[2] = 0x30;
/*						OneTime[index1].cdat[3] = 0;
						OneTime[index1].cdat[4] = 0;
						OneTime[index1].cdat[5] = 0;
						OneTime[index1].cdat[6] = 0;
						OneTime[index1].cdat[7] = 0; */		  	
						AlterMessageByte(0,0x356,0,OneTime[index1].cdat,3,0);

						// ID 46C, alter byte 2  
						OneTime[index1].cdat[0] = 0;			
						OneTime[index1].cdat[1] = 0x60;
/*						OneTime[index1].cdat[2] = 0xFF;
						OneTime[index1].cdat[3] = 0xFF;
						OneTime[index1].cdat[4] = 0xC0;
						OneTime[index1].cdat[5] = 0x14;
						OneTime[index1].cdat[6] = 0;
						OneTime[index1].cdat[7] = 0; */
						AlterMessageByte(0,0x46C,0,OneTime[index1].cdat,2,0);
						IGN_OFF_FLG = 1;
						break;

					case 0x30:	// IGN_START
						// ID 356, alter byte 2
						OneTime[index1].cdat[0] = 0;
						OneTime[index1].cdat[1] = 0;		  	
						OneTime[index1].cdat[2] = 0x60;
/*						OneTime[index1].cdat[3] = 0;
						OneTime[index1].cdat[4] = 0;
						OneTime[index1].cdat[5] = 0;
						OneTime[index1].cdat[6] = 0;
						OneTime[index1].cdat[7] = 0; */		  	
						AlterMessageByte(0,0x356,0,OneTime[index1].cdat,3,0);

						// ID 46C, alter byte 2  
						OneTime[index1].cdat[0] = 0;			
						OneTime[index1].cdat[1] = 0xA0;
/*						OneTime[index1].cdat[2] = 0xFF;
						OneTime[index1].cdat[3] = 0xFF;
						OneTime[index1].cdat[4] = 0xC0;
						OneTime[index1].cdat[5] = 0x14;
						OneTime[index1].cdat[6] = 0;
						OneTime[index1].cdat[7] = 0; */
						AlterMessageByte(0,0x46C,0,OneTime[index1].cdat,2,0);
						IGN_OFF_FLG = 1;
						break;

						
					case 0x40:	// IGN_RUN_0
						// ID 356, alter byte 2
						OneTime[index1].cdat[0] = 0;
						OneTime[index1].cdat[1] = 0;		  	
						OneTime[index1].cdat[2] = 0x80;	
/*						OneTime[index1].cdat[3] = 0;
						OneTime[index1].cdat[4] = 0;
						OneTime[index1].cdat[5] = 0;
						OneTime[index1].cdat[6] = 0;
						OneTime[index1].cdat[7] = 0;	  */	
						AlterMessageByte(0,0x356,0,OneTime[index1].cdat,3,0);

						// ID 46C, alter byte 2  
						OneTime[index1].cdat[0] = 0;			
						OneTime[index1].cdat[1] = 0x80;
/*						OneTime[index1].cdat[2] = 0xFF;
						OneTime[index1].cdat[3] = 0xFF;
						OneTime[index1].cdat[4] = 0xC0;
						OneTime[index1].cdat[5] = 0x14;
						OneTime[index1].cdat[6] = 0;
						OneTime[index1].cdat[7] = 0; */
						AlterMessageByte(0,0x46C,0,OneTime[index1].cdat,2,0);

						/// STATUS_CCAN5
						OneTime[0].cdat[0] = 0;
						OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
						OneTime[0].cdat[2] = 0;
						OneTime[0].cdat[3] = 0;
						OneTime[0].cdat[4] = 0;
						OneTime[0].cdat[5] = 0;
						OneTime[0].cdat[6] = 0;
						OneTime[0].cdat[7] = 0x02;
						AlterMessageByte(0,0x3E2,0,OneTime[index1].cdat,8,0);

						IGN_OFF_FLG = 1;
						break;

					case 0x50:	// IGN_RUN_5
						// ID 356, alter byte 2
						OneTime[index1].cdat[0] = 0;
						OneTime[index1].cdat[1] = 0;		  	
						OneTime[index1].cdat[2] = 0x80;		  	
/*						OneTime[index1].cdat[3] = 0;
						OneTime[index1].cdat[4] = 0;
						OneTime[index1].cdat[5] = 0;
						OneTime[index1].cdat[6] = 0;
						OneTime[index1].cdat[7] = 0; */
						AlterMessageByte(0,0x356,0,OneTime[index1].cdat,3,0);

						// ID 46C, alter byte 2
						OneTime[index1].cdat[0] = 0;  			
						OneTime[index1].cdat[1] = 0x80;
/*						OneTime[index1].cdat[2] = 0xFF;
						OneTime[index1].cdat[3] = 0xFF;
						OneTime[index1].cdat[4] = 0xC0;
						OneTime[index1].cdat[5] = 0x14;
						OneTime[index1].cdat[6] = 0;
						OneTime[index1].cdat[7] = 0; */
						AlterMessageByte(0,0x46C,0,OneTime[index1].cdat,2,0);

						/// STATUS_CCAN5
						OneTime[0].cdat[0] = 0;
						OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
						OneTime[0].cdat[2] = 0;
						OneTime[0].cdat[3] = 0;
						OneTime[0].cdat[4] = 0;
						OneTime[0].cdat[5] = 0x04;
						OneTime[0].cdat[6] = 0x08;
						OneTime[0].cdat[7] = 0x02;
						AlterMessageByte(0,0x3E2,0,OneTime[index1].cdat,8,0);

						IGN_OFF_FLG = 1;
						break;
					case 0x60:	// IGN_RUN_10
						// ID 356, alter byte 2
						OneTime[index1].cdat[0] = 0;
						OneTime[index1].cdat[1] = 0;		  	
						OneTime[index1].cdat[2] = 0x80;		 
/*						OneTime[index1].cdat[3] = 0;
						OneTime[index1].cdat[4] = 0;
						OneTime[index1].cdat[5] = 0;
						OneTime[index1].cdat[6] = 0;
						OneTime[index1].cdat[7] = 0; 	*/
						AlterMessageByte(0,0x356,0,OneTime[index1].cdat,3,0);

						// ID 46C, alter byte 2 
						OneTime[index1].cdat[0] = 0; 			
						OneTime[index1].cdat[1] = 0x80;
/*						OneTime[index1].cdat[2] = 0xFF;
						OneTime[index1].cdat[3] = 0xFF;
						OneTime[index1].cdat[4] = 0xC0;
						OneTime[index1].cdat[5] = 0x14;
						OneTime[index1].cdat[6] = 0;
						OneTime[index1].cdat[7] = 0;	*/
						AlterMessageByte(0,0x46C,0,OneTime[index1].cdat,2,0);

						/// STATUS_CCAN5
						OneTime[0].cdat[0] = 0;
						OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
						OneTime[0].cdat[2] = 0;
						OneTime[0].cdat[3] = 0;
						OneTime[0].cdat[4] = 0;
						OneTime[0].cdat[5] = 0x08;
						OneTime[0].cdat[6] = 0x08;
						OneTime[0].cdat[7] = 0x02;
						AlterMessageByte(0,0x3E2,0,OneTime[index1].cdat,8,0);

						IGN_OFF_FLG = 1;
						break;
					
					case 0x70:
						break;
					case 0x80:
						break;
				}

			keyChange &= ~0xF0;		// clear Ignition nibble
			}
		}
	}


}
void User4()	// program select 4
{	
	U8 cnt=0;

	while(!(keyChange & 0xF))
	{
		for (cnt = 0 ; cnt < 8 ; cnt++)
		{
			LED1 ^= 1;
			mSecDelay(100);
		}
	mSecDelay(1200);
	}
}
void User5()	// program select 5
{	
	U8 cnt=0;

	while(!(keyChange & 0xF))
	{
		for (cnt = 0 ; cnt < 10 ; cnt++)
		{
			LED1 ^= 1;
			mSecDelay(100);
		}
	mSecDelay(1000);
	}
}
void User6()	// program select 6
{	
	U8 cnt=0;

	while(!(keyChange & 0xF))
	{
		for (cnt = 0 ; cnt < 12 ; cnt++)
		{
			LED1 ^= 1;
			mSecDelay(100);
		}
	mSecDelay(800);
	}
}
void User7()	// program select 7
{	
	U8 cnt=0;

	while(!(keyChange & 0xF))
	{
		for (cnt = 0 ; cnt < 14 ; cnt++)
		{
			LED1 ^= 1;
			mSecDelay(100);
		}
	mSecDelay(600);
	}
}

void User8(void)	// example program select 8
{
}
