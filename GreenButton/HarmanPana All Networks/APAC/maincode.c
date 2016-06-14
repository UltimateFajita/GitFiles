//Can Bus Simulator
#include "c8051F040.h"
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
#include "main.h"
#include "spi.h"

CANFRAME xdata txframe;
CANFRAME xdata OneTime[TXBUFFS];
CANFRAME xdata rxframe0[RX0BUFFERS];
CANFRAME xdata rxframe1[RX1BUFFERS];
U32 xdata RxCAN1IDs[RX1BUFFERS];
U8 xdata MaxMesgs,MaxRxTraps;

MSGTABLE xdata *Mesgs;
U8 xdata MesgCnt;
BUTTABLE xdata *Buttons;
U8 xdata ButtCnt;
RxMESG xdata *RxMesg;
U8 xdata RxCnt;
U16 xdata RelayCnf[4];
U8 xdata RelayChange;
extern U8 xdata RxBuffCnt[2];

U32 keys,keyChange;
bit CANintMode,RxCAN0,RxCAN1;
extern U32 msticks;

void main(void) 
{ 
	WDTCN = 0xde;				// disable watchdog timer
	WDTCN = 0xad;

	config_IO();				// configure Port I/O
	Oscillator_Init();			// switch to external oscillator
	//======================================================

 	
	SPI_init();					// Enable SPI for CAN1 port
	CAN1begin(CAN_500KBPS);		// Initialize CAN1 at 500k

	//======================================================
								// active low	// 0 means CAN B, 1 means CAN C/I
	SEL_CAN_C = 1;				// Enable Port#1, bit#7 to SelectCAN_B 
	config_CAN_timing(125);		// Function call to start CAN

	EIE2  = 0x20;				// Enable CAN Interrupt
	EIP2 |= 0x20;				// Set CAN Interrupt Priority: HIGH 

	// Function call to start CAN
	SFRPAGE = CAN0_PAGE;
	CAN0CN  = 0x0A;	// Enables Int's, Error and IE (rx or tx)
	//=====================================================================

	ClearUserVars();
	//======================================================

	//Start Timer3 generating 1ms interupt
	Start_Timer3(1000000/998);	// adjust 998 value to get timeing exact
 	//======================================================

   	//Global enable 8051 interrupts
	EA = 1;
	mSecDelay(2);
	LED1 = 0;

	while(1)
		{
		switch(keys & 0xF)
		{
			case 1:
				User1();
				break;
			case 2:
				User2();
				break;
			case 3:
				User3();
				break;
			case 4:
				User4();
				break;
			case 5:
				User5();
				break;
			case 6:
				User6();
				break;
			case 7:
				User7();
				break;
			case 8:
				User8();
				break;
		}

		// check if a relay is set to prog selector
		for (RxCnt = 0 ; RxCnt < 4; RxCnt++)
			if ((RelayCnf[RxCnt] & 0xF00) == REL_PROG)	// Relay controled by PROG Select
				if ((RelayCnf[RxCnt]&0xFF) == (keys&0xF))	// matching Prog select
					RelayChange |= 1<<RxCnt;			// flag for relay change

		keyChange &= ~0xF;

		ClearUserVars();
	}
}

U8 MemManage(U8 Bcnt,U8 Mcnt,U8 Rcnt,U16 FirstAddress)
{
	if ((Bcnt*sizeof(BUTTABLE) + Mcnt*sizeof(MSGTABLE) + Rcnt*sizeof(RxMESG)) < (0x1000-FirstAddress))
	{
		Buttons = (BUTTABLE xdata *)FirstAddress;
		Mesgs = (MSGTABLE xdata *)(FirstAddress + sizeof(BUTTABLE)*Bcnt);
		RxMesg = (RxMESG xdata *)(FirstAddress + sizeof(BUTTABLE)*Bcnt+Mcnt*sizeof(MSGTABLE));
		return(1);
	}
	else
		return(0);
}

void ClearUserVars()
{
	U8 i,j;
	
	for (i = 0 ; i < 3 ; i++)
	{
		OneTime[i].MsgNum = 0;
		OneTime[i].arbID = 0;
		OneTime[i].sz = 0;
		for (j = 0 ; j < 8 ; j++)
			OneTime[i].cdat[j] = 0;
	}

	RxCAN1 = 0;
	RxCAN0 = 0;

	RELAY1 = 0;
	RELAY2 = 0;
	RelayChange = 0;
	for (i = 0 ; i < 4 ; i++)
		RelayCnf[i] = 0;

	clear_message_object(0,1,RX0BUFFERS-1);	// Clear CAN0 RAM
	clear_message_object(1,0,RX1BUFFERS);	// Clear CAN1 RAM

	MesgCnt = 0;
	ButtCnt = 0;
	RxCnt = 0;
	RxBuffCnt[0] = 1;
	RxBuffCnt[1] = 0;
}
	
void config_IO(void)
{
	char SFRPAGE_SAVE = SFRPAGE;

	SFRPAGE  = CONFIG_PAGE;
	
	XBR0	 = 0x02;	// enable SPI interface
	XBR1	 = 0x04;	// enable INT0 CAN1 interrupt via SPI
	XBR2	 = 0x40;	// enable crossbar and disable weak pullups
	XBR3     = 0x80;    // Configure CAN TX pin (CTX) as push-pull digital output
	
	P0MDOUT  = 0x8F;	// pin p7,3,2,1,0 push-pull, p4,5,6 are open drain outputs
	
	P1MDIN	 = 0xff; 	// All inputs col 1-8
	P1MDOUT  = 0x00;	// Port 1 is open drain as inputs

	P2MDIN 	|= 0xF3;	// p0,1,4,5,6,7 inputs
	P2MDOUT  = 0x0C;	// p2,3 is push pull	
	
	P3MDIN 	|= 0x0F;	// p0,1,2,3 inputs
	P3MDOUT  = 0x30;	// p4,5 is push pull (relay drivers), p6,p7 open drain outputs	

	SFRPAGE  = SFRPAGE_SAVE;
}

void CheckRelay()
{
	U8 i;
	bit Invert;

	for (i = 0 ; i < 4; i++)
	{
		Invert = (RelayCnf[i] & REL_NOT)?(1):(0);

		if ((RelayChange>>(i+4))&1)			// left on last pass turn off now
		{
			RelayCtrl(i+1,0^Invert);
			RelayChange &= ~(0x10<<i);		// clear Off flag
		}
		if ((RelayChange>>i)&1)
		{
			if (RelayCnf[i] & REL_TOG)		// toggle?
				RelayCtrl(i+1,2);
			else
			{
				RelayCtrl(i+1,1^Invert);
				if (RelayCnf[i] & REL_PULSE)	// auto off or pulsed
					RelayChange |= 0x10<<i;		// set flag to zero next pass
			}
			RelayChange &= ~(1<<i);			// clear On flag
		}
	}
}

void CheckButts()
{
	U8 button;
	U8 i,j,x;
	U8 value[8];
	U32 changebits;

	changebits = keyChange>>8;

	for (button = 0 ; button < 20 && !(changebits & 1) ; button++)
		changebits = changebits>>1;

	for (i = 0 ; i < ButtCnt && (Buttons+i)->button != button; i++);		// i=index to correct Button

	if (i < ButtCnt)
	{
		// check if a relay is set to this button
		for (j = 0 ; j < 4; j++)
			if ((RelayCnf[j] & 0xF00) == REL_BUTT)	// Relay controled by button
				if ((RelayCnf[j]&0xFF) == button)	// matching button
					if (!(RelayCnf[j] & REL_TOG) || ((keys>>(8+button))&1))
						RelayChange |= 1<<j;			// flag for relay change

		for (j = 0 ; j < MesgCnt && (Mesgs+j)->ID != (Buttons+i)->ID; j++);	// j=index to correct Message

		if (j < MesgCnt)				// found matching Repetative message, modify it.
		{
			if (keys>>(8+button) & 1)	// button was pressed
			{
				for (x = 0 ; x < (Mesgs+j)->cnt ; x++)
					value[x] = ((Mesgs+j)->dat[x] & (Buttons+i)->mask[x]) | (Buttons+i)->value[x];
				AlterMessageByte(0,(Mesgs+j)->ID,0,value,x,(Buttons+i)->cnt);
			}		
			else						// button released
			{							// check if held data then restore
				if ((Mesgs+j)->Altered == 0xFF)
				{
					for (x = 0 ; x < (Mesgs+j)->cnt ; x++)
						(Mesgs+j)->dat[x] = (Mesgs+j)->Restore[x];
					(Mesgs+j)->Altered = 0;
				}
			}

			keyChange &= ~((U32)1<<(button+8));	// clear the button, it's processed
		}
		else							// no matching Repeatitive message found, oneTime it. 
		{
			if (keys>>(8+button) & 1)	// button was pressed
			{
				for (j = 0 ; j < TXBUFFS & OneTime[j].arbID ; j++);
				if (j < TXBUFFS)				// empty one time available
				{
					for (x = 0 ; x < 8 ; x++)
						OneTime[j].cdat[x] = (Buttons+i)->value[x];
					OneTime[j].sz = 8;
					OneTime[j].arbID = (Buttons+i)->ID;

					keyChange &= ~((U32)1<<(button+8));	// clear the button, it's processed
				}
			}
		}
	}
}

// ProcessRx - new message in, do something with it
// port duh
// index of rxframe
void ProcessRx(U8 port,U8 index)
{
	U8 i,j,x;
	U8 value[8];
	U8 RxMsg;
	CANFRAME *rx;
	bit test=1;

	if (!port)
		rx = &rxframe0;
	else
		rx = &rxframe1;

	RxMsg = index;
	if (port)
		RxMsg |= 0x80;
	
	// index to RxMesg instructions
	for (i = 0 ; i < RxCnt && (RxMesg+i)->RxNum != RxMsg ; i++);	

	if (i < RxCnt)
	{
		do
		{
			if (!test)	// first pass failed re-index
			{
				// re-index to next matching RxMesg
				for (i++ ; i < RxCnt && (RxMesg+i)->RxNum != RxMsg ; i++);
				if (i < RxCnt) test = 1;	
			}
	
			for (j = 0 ; j < (RxMesg+i)->incnt && test; j++)
				if (((rx+index)->cdat[j] & (RxMesg+i)->inMask[j]) != (RxMesg+i)->inValue[j])
					test = 0;	// non-matching do nothing

		} while (!test && i<RxCnt);		// if failed look again

		if (test)
		{
			for (j = 0 ; j < 4; j++)
				if ((RelayCnf[j] & 0xF00) == REL_RX)	// Relay controled by RxMesg
					if ((RelayCnf[j]&0xFF) == i)		// outputing correct message
						RelayChange |= 1<<j;			// flag for relay change

			for (j = 0 ; j < MesgCnt && (Mesgs+j)->ID != (RxMesg+i)->outID; j++);	// j=index to correct Message

			if (j < MesgCnt)				// found matching Repetative message, modify it.
			{
				for (x = 0 ; x < (Mesgs+j)->cnt ; x++)
					value[x] = ((Mesgs+j)->dat[x] & (RxMesg+i)->outMask[x]) | (RxMesg+i)->outValue[x];
				AlterMessageByte(port,(Mesgs+j)->ID,0,value,x,(RxMesg+i)->altercnt);
			}
			else							// no matching Repeatitive message found, oneTime it. 
			{
				for (j = 0 ; j < TXBUFFS & OneTime[j].arbID ; j++);
				if (j < TXBUFFS)				// empty one time available
				{
					for (x = 0 ; x < (RxMesg+i)->outcnt ; x++)
						OneTime[j].cdat[x] = (RxMesg+i)->outValue[x];
					OneTime[j].sz = (RxMesg+i)->outcnt;
					OneTime[j].arbID = (RxMesg+i)->outID;
				}
			}
		}
	}
}

// used to start a repeating message on the bus
// port > can port 0 or 1
// ID > can ID
// Rate > the cyclic timing of the message in milliseconds
// cnt > bytes of message 1-8
// dat[] > data bytes to output
U8 AddMessage(U8 port,U32 ID,U16 Rate,U8 cnt,U8 dat[8])
{
	U8 i;

	if (port) ID |= PORTFLG;

	(Mesgs+MesgCnt)->ID = ID;
	(Mesgs+MesgCnt)->Rate = Rate;
	(Mesgs+MesgCnt)->ticks = 0;
	(Mesgs+MesgCnt)->cnt = cnt;
	for (i = 0 ; i < cnt ; i++)
	{
		(Mesgs+MesgCnt)->dat[i] = dat[i];
		(Mesgs+MesgCnt)->Restore[i] = dat[i];
	}
	(Mesgs+MesgCnt)->Altered = 0;	// non altered data

	if (MesgCnt < MaxMesgs)
		MesgCnt++;

	return (MesgCnt-1);
}

// Used to Alter a Repeating messages data bytes.
// port > can port 0 or 1
// ID > can ID
// index > starting data byte to alter
// *value > pointer to first new data value
// numbytes > number of bytes to alter
// count > is number of times to repeat / hold new values on bus. 
void AlterMessageByte(U8 port,U32 ID,U8 index,U8 *value,U8 numbytes,U8 count)
{
	U8 i,j;
	
	if (port) ID |= PORTFLG;

	for (i = 0 ; i < MesgCnt && ID != (Mesgs+i)->ID; i++);

	if (i < MesgCnt)					// message found
	{
		for (j = 0 ; j < numbytes && (index+j) < 8; j++)
		{
			(Mesgs+i)->dat[index+j] = *(value+j);		// alter the data value
			if (count)
				(Mesgs+i)->Altered = count;				// number of Tx's to alter before restoring (0 permanant)
			else
				(Mesgs+i)->Restore[index+j] = *(value+j);	// permanant alter
		}
	}
}

//	Used to configure each button
// button = 0-19  (upper right is 0, upper left is 9, lower left is 19
// port > can port 0 or 1
// ID > can ID
// mask > is ANDed with current data then,
// value > is ORed with current data
// count > is number of times to repeat / hold new values on bus. 
U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count)
{
	U8 i;

	if (port) ID |= PORTFLG;

	(Buttons+ButtCnt)->button = button;
	(Buttons+ButtCnt)->ID = ID;
	(Buttons+ButtCnt)->cnt = count;
	for (i = 0 ; i < 8 ; i++)
	{
		(Buttons+ButtCnt)->mask[i] = mask[i];
		(Buttons+ButtCnt)->value[i] = value[i];
	}
	if (ButtCnt < MAXBUTTONS)
		ButtCnt++;

	return (ButtCnt-1);
}

// Used to configure the recieve side of a RxMessage filter/trap
// port - 0 or 1
// ID is ID
// mask[] - 0's mean don't cares, 1's mean match bits
// value[] -> matching value after mask off above
// count > input message byte count.
// returns index for use by 2nd half 
U8 RxMessageConfig1(U8 port,U32 ID,U8 mask[8],U8 value[8],U8 count)
{
	U8 i;

	(RxMesg+RxCnt)->RxNum = initRxFrameID(port, ID);	// Rx for both CAN ports

	(RxMesg+RxCnt)->incnt = count;
	for (i = 0 ; i < count ; i++)
	{
		(RxMesg+RxCnt)->inMask[i] = mask[i];
		(RxMesg+RxCnt)->inValue[i] = value[i];
	}
	
	if (RxCnt < MaxRxTraps)
		RxCnt++;

	return (RxCnt-1);
}
// 2nd half of above used to configure output message of a RxMessage filter trap
// (too many variables passed in one routine)
// index of which RxMesg filter, passed from 1st half
// outport - output message on port 0 or 1
// outID = output CAN ID
// outmask > is ANDed with current data then,
// outvalue > is ORed with current data
// count number of bytes in output message
void RxMessageConfig2(U8 index,U8 outport,U32 outID,U8 outmask[8],U8 outvalue[8],U8 count,U8 altercount)
{
	U8 i;

	if (outport) outID |= PORTFLG;
	(RxMesg+index)->outID = outID;
	(RxMesg+index)->outcnt = count;
	(RxMesg+index)->altercnt = altercount;
	for (i = 0 ; i < count ; i++)
	{
		(RxMesg+index)->outMask[i] = outmask[i];
		(RxMesg+index)->outValue[i] = outvalue[i];
	}
}


/*********************************************************************************************************
** Function name:           RelayCtrl
** Descriptions:            turn on / off Relay pins
*********************************************************************************************************/
void RelayCtrl(U8 num,U8 type)
{
	U8 relay=0;
	static bit R3,R4;

	switch (num)
	{
		case 1:
			if (type > 1)
				RELAY1 ^= 1;
			else
				RELAY1 = type;
			break;
		case 2:
			if (type > 1)
				RELAY2 ^= 1;
			else
				RELAY2 = type;
			break;
		case 3:
			if (type > 1)
				R3 ^= 1;
			else
				R3 = type;

			relay = 0x10;
			type = ((U8)R3)<<4;
			break;
		case 4:
			if (type > 1)
				R4 ^= 1;
			else
				R4 = type;

			relay = 0x20;
			type = ((U8)R4)<<5;
			break;
	}

	if (relay)
		CAN1modifyRegister(MCP_BFPCTRL,relay,type);
}
