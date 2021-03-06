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

extern U32 keys,keyChange;
extern bit RxCAN0,RxCAN1;
extern U32 msticks;

void User1()	// program select 1
{	
	U32 StartTicks = 0;
	U8 index,index1,error,ErrCnt,counter,cnt,vin;
	bit IGN_OFF=1;

	char VIN[17] = "1C3CCCCB7FN1124UC";

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
	// build repetative message 3F2 on CAN0(ECU_CFG_3)
	OneTime[0].cdat[0] = 0x95;
	OneTime[0].cdat[1] = 0x02;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0x57;
	OneTime[0].cdat[3] = 0xA0;
	OneTime[0].cdat[4] = 0x09;
	OneTime[0].cdat[5] = 0x00;
	OneTime[0].cdat[6] = 0x00;
	OneTime[0].cdat[7] = 0x02;
//U8 AddMessage(U8 port,U32 ID,U16 Rate,U8 cnt,U8 dat[8])
	AddMessage(0,0x3F2,500,8,OneTime[0].cdat);

	// build repetative message 3E0 on CAN0 (vin)
	vin = 0;
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	AddMessage(0,0x3E0,500,8,OneTime[0].cdat);

	// build repetative message 3E8 on CAN0 ()
	OneTime[0].cdat[0] = 0x39;
	OneTime[0].cdat[1] = 0x1D;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0x22;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	AddMessage(0,0x3E8,500,8,OneTime[0].cdat);

	// build repetative message 3EA on CAN0 ()
	OneTime[0].cdat[0] = 0x01;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0xC0;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	AddMessage(0,0x3EA,100,8,OneTime[0].cdat);

	// build repetative message 22F on CAN0 ()
	OneTime[0].cdat[0] = 0x01;
	AddMessage(0,0x22F,1000,1,OneTime[0].cdat);

	// build repetative message 318 on CAN0 ()
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	AddMessage(0,0x318,1000,8,OneTime[0].cdat);

	// build repetative message 122 on CAN0 ()
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	AddMessage(0,0x122,1000,4,OneTime[0].cdat);

	// build repetative message 322 on CAN0 ()
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	AddMessage(0,0x322,1000,8,OneTime[0].cdat);

/*************************************************************************
 	Button setup
	example below -
	pressing button 1 causes byte 5 of message 0x6E2 to be altered to 0xAA 
	for the next 5 outputs. The mask clears byte 5 leaving the rest alone.  
	The value (0xAA) is ORed into byte 5.
**************************************************************************/
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
	OneTime[1].cdat[5] = 0xAA;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
//U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
	SimpleButtonConfig(1,1,0x6E2,OneTime[0].cdat,OneTime[1].cdat,5);
	#define BUTTONMASK 0x2	// only button 1 defined (bit 1) 

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
	RelayCnf[2] = REL_BUTT | 1 | REL_TOG;	// relay 3 toggles with button 1
	RelayCnf[1] = REL_RX | 0 | REL_PULSE;	// relay 2 with first (0) rxtrap configured
	RelayCnf[0] = REL_TX | 3 | REL_PULSE;	// relay 1 with 4th (3) TxMesg configured

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
				AlterMessageByte(0,0x3E0,0,OneTime[index1].cdat,8,1);
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
				}}

			for (index1 = 0 ; index1 < TXBUFFS && OneTime[index1].arbID ; index1++);
			if (index1 < TXBUFFS)				// empty one time available
			{
				IGN_OFF = 0;			// default IGN_OFF flag to off
				switch(keys & 0xF0)
				{
					case 0x10:	// IGN_OFF
						// ID 190, alter byte 0,1 
						OneTime[index1].cdat[0] = 0x01;	      	//ignition run, start not active  04 = run ;02=ACC ;0D= NormalStart, key in Ign, 						
						OneTime[index1].cdat[1] = 0x02;		  	//key in ignition
						AlterMessageByte(0,0x122,0,OneTime[index1].cdat,2,0);

						// ID 98, alter bytes 2,3  
						OneTime[index1].cdat[0] = 0x0; 			// 0 speed			
						OneTime[index1].cdat[1] = 0x0;
						AlterMessageByte(0,0x322,2,OneTime[index1].cdat,2,0);
						IGN_OFF = 1;	// TURN on IGN_OFF flag
						break;
					case 0x20:	// IGN_ACC
						// ID 190, alter byte 0,1 
						OneTime[index1].cdat[0] = 0x02;	      	//ignition run, start not active  04 = run ;02=ACC ;0D= NormalStart, key in Ign, 						
						OneTime[index1].cdat[1] = 0x02;		  	//key in ignition
						AlterMessageByte(0,0x122,0,OneTime[index1].cdat,2,0);

						// ID 98, alter bytes 2,3  
						OneTime[index1].cdat[0] = 0x0; 			// 0 speed			
						OneTime[index1].cdat[1] = 0x0;
						AlterMessageByte(0,0x322,2,OneTime[index1].cdat,2,0);
						break;
					case 0x30:	// IGN_RUN_0
						// ID 190, alter byte 0,1 
						OneTime[index1].cdat[0] = 0x04;	      	//ignition run, start not active  04 = run ;02=ACC ;0D= NormalStart, key in Ign, 						
						OneTime[index1].cdat[1] = 0x02;		  	//key in ignition
						AlterMessageByte(0,0x122,0,OneTime[index1].cdat,2,0);

						// ID 98, alter bytes 2,3  
						OneTime[index1].cdat[0] = 0x0; 			// 0 speed			
						OneTime[index1].cdat[1] = 0x0;
						AlterMessageByte(0,0x322,2,OneTime[index1].cdat,2,0);
						break;
					case 0x40:	// IGN_RUN_5
						// ID 190, alter byte 0,1 
						OneTime[index1].cdat[0] = 0;	      	//ignition run, start not active  04 = run ;02=ACC ;0D= NormalStart, key in Ign, 						
						OneTime[index1].cdat[1] = 0;		  	//key in ignition
						AlterMessageByte(0,0x122,0,OneTime[index1].cdat,2,0);

						// ID 98, alter bytes 2,3  
						OneTime[index1].cdat[0] = 0; 			// 5 speed			
						OneTime[index1].cdat[1] = 0;
						OneTime[index1].cdat[2] = 0x04;	      	//ignition run, start not active  04 = run ;02=ACC ;0D= NormalStart, key in Ign, 						
						OneTime[index1].cdat[3] = 0x05;
						AlterMessageByte(0,0x322,2,OneTime[index1].cdat,4,0);
						break;
					case 0x50:	// IGN_RUN_10
						// ID 190, alter byte 0,1 
						OneTime[index1].cdat[0] = 0x04;	      	//ignition run, start not active  04 = run ;02=ACC ;0D= NormalStart, key in Ign, 						
						OneTime[index1].cdat[1] = 0x02;		  	//key in ignition
						AlterMessageByte(0,0x122,0,OneTime[index1].cdat,2,0);	//key in ignition 

						// ID 98, alter byte 2,3 
						OneTime[index1].cdat[0] = 0x00; 		// 10 speed			
						OneTime[index1].cdat[1] = 0x00;
						OneTime[index1].cdat[2] = 0x08;
						AlterMessageByte(0,0x322,2,OneTime[index1].cdat,3,0);
						break;
					case 0x60:	// IGN_START
						// ID 190, alter byte 0,1 
						OneTime[index1].cdat[0] = 0x05;	      	//ignition run, start not active  04 = run ;02=ACC ;0D= NormalStart, key in Ign, 						
						OneTime[index1].cdat[1] = 0x22;		  	//key in ignition
						AlterMessageByte(0,0x122,0,OneTime[index1].cdat,2,0);

						// ID 98, alter bytes 2,3 
						OneTime[index1].cdat[0] = 0x0; 			// 0 speed			
						OneTime[index1].cdat[1] = 0x0;
						AlterMessageByte(0,0x322,2,OneTime[index1].cdat,2,0);
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
void User2()	// program select 2
{	
	U32 StartTicks = 0;
	U8 index,index1,error,ErrCnt,counter,cnt,vin;
	bit IGN_OFF=1;

	char VIN[17] = "1C4PJMBB0EW3001UC";

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
	// build repetative message 3F2 on CAN0(ECU_CFG_3)
	OneTime[0].cdat[0] = 0x95;
	OneTime[0].cdat[1] = 0x02;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0x57;
	OneTime[0].cdat[3] = 0xA0;
	OneTime[0].cdat[4] = 0x09;
	OneTime[0].cdat[5] = 0x00;
	OneTime[0].cdat[6] = 0x00;
	OneTime[0].cdat[7] = 0x02;
//U8 AddMessage(U8 port,U32 ID,U16 Rate,U8 cnt,U8 dat[8])
	AddMessage(0,0x3F2,500,8,OneTime[0].cdat);

	// build repetative message 3E0 on CAN0 (vin)
	vin = 0;
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	AddMessage(0,0x3E0,500,8,OneTime[0].cdat);

	// build repetative message 3E8 on CAN0 (IGN)
	OneTime[0].cdat[0] = 0x39;
	OneTime[0].cdat[1] = 0x1D;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0x22;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	AddMessage(0,0x3E8,500,8,OneTime[0].cdat);

	// build repetative message 098 on CAN0 (speed)
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 0;
	OneTime[0].cdat[7] = 0;
	AddMessage(0,0x3EA,100,8,OneTime[0].cdat);

	// build repetative message 49D on CAN0 (lang)
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0;
	OneTime[0].cdat[3] = 0;
	OneTime[0].cdat[4] = 0;
	OneTime[0].cdat[5] = 0;
	OneTime[0].cdat[6] = 8;	// defaulted to english?i
	OneTime[0].cdat[7] = 0;
	AddMessage(0,0x49D,1000,8,OneTime[0].cdat);

	// build repetative message 6EA on CAN0
	OneTime[0].cdat[0] = 0;
	OneTime[0].cdat[1] = 0x11;	// using OneTime structure as temp storage
	OneTime[0].cdat[2] = 0x22;
	OneTime[0].cdat[3] = 0x33;
	OneTime[0].cdat[4] = 0x44;
	OneTime[0].cdat[5] = 0x55;
	OneTime[0].cdat[6] = 0x66;
	OneTime[0].cdat[7] = 0x77;
	AddMessage(0,0x6EA,1000,8,OneTime[0].cdat);

/*************************************************************************
 	Button setup
	example below -
	pressing button 1 causes byte 5 of message 0x6E2 to be altered to 0xAA 
	for the next 5 outputs. The mask clears byte 5 leaving the rest alone.  
	The value (0xAA) is ORed into byte 5.
**************************************************************************/
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
	OneTime[1].cdat[5] = 0xAA;
	OneTime[1].cdat[6] = 0;
	OneTime[1].cdat[7] = 0;
//U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
	SimpleButtonConfig(1,1,0x6E2,OneTime[0].cdat,OneTime[1].cdat,5);
	#define BUTTONMASK 0x2	// only button 1 defined (bit 1) 

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
	RelayCnf[2] = REL_BUTT | 1 | REL_TOG;	// relay 3 toggles with button 1
	RelayCnf[1] = REL_RX | 0 | REL_PULSE;	// relay 2 with first (0) rxtrap configured
	RelayCnf[0] = REL_TX | 3 | REL_PULSE;	// relay 1 with 4th (3) TxMesg configured

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
				AlterMessageByte(0,0x3D3,0,OneTime[index1].cdat,8,1);
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
				}}

			for (index1 = 0 ; index1 < TXBUFFS && OneTime[index1].arbID ; index1++);
			if (index1 < TXBUFFS)				// empty one time available
			{
				IGN_OFF = 0;			// default IGN_OFF flag to off
				switch(keys & 0xF0)
				{
					case 0x10:	// IGN_OFF
						// ID 190, alter byte 0,1 
						OneTime[index1].cdat[0] = 0x00;	      	//ignition run, start not active  04 = run ;02=ACC ;0D= NormalStart, key in Ign, 						
						OneTime[index1].cdat[1] = 0x22;		  	//key in ignition
						AlterMessageByte(0,0x190,0,OneTime[index1].cdat,2,0);

						// ID 98, alter bytes 2,3  
						OneTime[index1].cdat[0] = 0x0; 			// 0 speed			
						OneTime[index1].cdat[1] = 0x0;
						AlterMessageByte(0,0x98,2,OneTime[index1].cdat,2,0);
						IGN_OFF = 1;	// TURN on IGN_OFF flag
						break;
					case 0x20:	// IGN_ACC
						// ID 190, alter byte 0,1 
						OneTime[index1].cdat[0] = 0x03;	      	//ignition run, start not active  04 = run ;02=ACC ;0D= NormalStart, key in Ign, 						
						OneTime[index1].cdat[1] = 0x22;		  	//key in ignition
						AlterMessageByte(0,0x190,0,OneTime[index1].cdat,2,0);

						// ID 98, alter bytes 2,3  
						OneTime[index1].cdat[0] = 0x0; 			// 0 speed			
						OneTime[index1].cdat[1] = 0x0;
						AlterMessageByte(0,0x98,2,OneTime[index1].cdat,2,0);
						break;
					case 0x30:	// IGN_RUN_0
						// ID 190, alter byte 0,1 
						OneTime[index1].cdat[0] = 0x04;	      	//ignition run, start not active  04 = run ;02=ACC ;0D= NormalStart, key in Ign, 						
						OneTime[index1].cdat[1] = 0x22;		  	//key in ignition
						AlterMessageByte(0,0x190,0,OneTime[index1].cdat,2,0);

						// ID 98, alter bytes 2,3  
						OneTime[index1].cdat[0] = 0x0; 			// 0 speed			
						OneTime[index1].cdat[1] = 0x0;
						AlterMessageByte(0,0x98,2,OneTime[index1].cdat,2,0);
						break;
					case 0x40:	// IGN_RUN_5
						// ID 190, alter byte 0,1 
						OneTime[index1].cdat[0] = 0x04;	      	//ignition run, start not active  04 = run ;02=ACC ;0D= NormalStart, key in Ign, 						
						OneTime[index1].cdat[1] = 0x22;		  	//key in ignition
						AlterMessageByte(0,0x190,0,OneTime[index1].cdat,2,0);

						// ID 98, alter bytes 2,3  
						OneTime[index1].cdat[0] = 0x0; 			// 5 speed			
						OneTime[index1].cdat[1] = 0x80;
						AlterMessageByte(0,0x98,2,OneTime[index1].cdat,2,0);
						break;
					case 0x50:	// IGN_RUN_10
						// ID 190, alter byte 0,1 
						OneTime[index1].cdat[0] = 0x04;	      	//ignition run, start not active  04 = run ;02=ACC ;0D= NormalStart, key in Ign, 						
						OneTime[index1].cdat[1] = 0x22;		  	//key in ignition
						AlterMessageByte(0,0x190,0,OneTime[index1].cdat,2,0);	//key in ignition 

						// ID 98, alter byte 2,3 
						OneTime[index1].cdat[0] = 0x01; 		// 10 speed			
						OneTime[index1].cdat[1] = 0x0;
						AlterMessageByte(0,0x98,2,OneTime[index1].cdat,2,0);
						break;
					case 0x60:	// IGN_START
						// ID 190, alter byte 0,1 
						OneTime[index1].cdat[0] = 0x05;	      	//ignition run, start not active  04 = run ;02=ACC ;0D= NormalStart, key in Ign, 						
						OneTime[index1].cdat[1] = 0x22;		  	//key in ignition
						AlterMessageByte(0,0x190,0,OneTime[index1].cdat,2,0);

						// ID 98, alter bytes 2,3 
						OneTime[index1].cdat[0] = 0x0; 			// 0 speed			
						OneTime[index1].cdat[1] = 0x0;
						AlterMessageByte(0,0x98,2,OneTime[index1].cdat,2,0);
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
void User3()	// program select 3
{	
	U8 cnt=0;

	while(!(keyChange & 0xF))
	{
		for (cnt = 0 ; cnt < 6 ; cnt++)
		{
			LED1 ^= 1;
			mSecDelay(100);
		}
	mSecDelay(1400);
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
	U8 cnt=0;

	while(!(keyChange & 0xF))
	{
		for (cnt = 0 ; cnt < 6 ; cnt++)
		{
			LED1 ^= 1;
			mSecDelay(100);
		}
	mSecDelay(400);
	}
}



