#include "c8051F040.h"
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
#include "main.h"
#include "spi.h"

extern CANFRAME xdata rxframe0[RX0BUFFERS];
extern CANFRAME xdata rxframe1[RX1BUFFERS];

unsigned char xdata IntReg;
//unsigned long	accumulator[ANALOG_INPUTS] = {0L};	// for integrated ADC samples
extern bit CANintMode,RxCAN0;
extern xdata CANFRAME txframe;
extern U32 xdata RxCAN1IDs[RX1BUFFERS];
U8 xdata RxBuffCnt[2];

/****************************************************************************
* Name: TxFrame
*  
* Description:  Transmit single can message in txframe out port 
* Parameters:
*		port - CAN port 0 or 1
*		
*****************************************************************************/
void TxFrame(U8 port)	// Tx for both CAN ports
{
	U8 TxBuff;

	if (port == 0)
	{
		txframe.MsgNum = 32;	// last MsgObj is for Tx
		tx_can_frame(0);
	}
	if (port == 1)
	{
		if (CAN1getNextFreeTXBuf(&TxBuff) != MCP_ALLTXBUSY)
		{
			CAN1write_canMsg( TxBuff );
			CAN1start_transmit( TxBuff );              
		}
	}
}

/****************************************************************************
* Name: initRxFrameID
*  
* Description:  initilize a Rx recieve buffer to trap a particular ID
* Parameters:
*		port 	- CAN port 0 or 1
*		MsgNum 	- # 0-31 of allowable Message buffers.
*		arbID 	- arbitration ID of message desired	
*		
*****************************************************************************/
U8 initRxFrameID(U8 port,U32 arbID)	// Rx for both CAN ports
{
	U8 i;
	CANFRAME *rx;
	U8 BuffSize;

	if (!port)
	{
		rx = &rxframe0;
		BuffSize = RX0BUFFERS;
	}
	else
	{
		rx = &rxframe1;
		BuffSize = RX1BUFFERS;
	}

	for (i = (port)?(0):(1) ; i < RxBuffCnt[port] && (rx+i)->arbID != arbID ; i++);

	if (i < BuffSize && (rx+i)->arbID != arbID)	// not already set
	{
		(rx+i)->arbID = arbID;	// set place holder
		if (port == 0)
		{
			init_msg_object_rx (i, arbID);
		}
		if (port == 1)
		{
			RxCAN1IDs[i] = arbID;
			EX0 = 1;	// enable CAN1 interupt
		}
		RxBuffCnt[port]++;
	}

i |= (port)?(0x80):(0);
return(i);
} 


/****************************************************************************
* Name: clear_message_object
*  
* Description:  Clear Message Objects
* Parameters:             
*		port 	- CAN port 0 or 1
*		mesgobj	- # 0-31 of allowable Message buffers. first mesgobj
*		count 	- following msgobj to also clear 0-31	
*				      
*****************************************************************************/
void clear_message_object (U8 port, unsigned char mesgobj,unsigned char count) 
{
	U8 BuffSize;

	if (port == 0)
	{
	   	SFRPAGE = CAN0_PAGE;
		CAN0ADR = IF1CMDMSK;          			//Point to Command Mask Register 1
		CAN0DATL = 0xFF;              			//Set direction to WRITE all IF registers to Msg Obj
		BuffSize = RX0BUFFERS;
	}
	else
		BuffSize = RX1BUFFERS;

	while (mesgobj < BuffSize && count)
	{
		if (port == 0)
		{
			CAN0ADR = IF1CMDRQST;  				//Write blank IF registers to GLOBAL msg obj
			CAN0DATL = mesgobj;
			rxframe0[mesgobj].arbID = 0xFFFFFFFF;
			rxframe0[mesgobj].MsgNum = 0;
		}
		if (port == 1)
		{
			RxCAN1IDs[mesgobj] = 0xFFFFFFFF;
			rxframe1[mesgobj].arbID = 0xFFFFFFFF;
			rxframe1[mesgobj].MsgNum = 0;
		}

		mesgobj++;
		count--;
	}
}		

/****************************************************************************
* Name: config_CAN_timing
*  
* Description:  Configure CAN Timing - 500kbps
*              
*       	
*		
*				      
*****************************************************************************/

void config_CAN_timing (unsigned int CANbaud)
{
	SFRPAGE = CAN0_PAGE;
	CAN0CN |= 0x41;							// Set CCE bit to enable write access
	CAN0ADR = BITREG;						// Point to the Bit Timing Register

	if (SYSCLK == 18000000)
	{
		switch (CANbaud)
		{
			case 83:
				CAN0DATH = 0x5A;					// 18.0mhz BRP=11 83.3kbps
				CAN0DATL = 0xCB;					// Tseg2=6, Tseg1=11, SJW=4
				break;
			case 125:
				CAN0DATH = 0x5A;					// 18.0mhz BRP=7 125kbps
				CAN0DATL = 0xC7;					// Tseg2=6, Tseg1=11, SJW=4
				break;
			case 500:
			default:
				CAN0DATH = 0x5A;					// 18.0mhz BRP=1 500kbps
				CAN0DATL = 0xC1;					// Tseg2=6, Tseg1=11, SJW=4
				break;
		}
	}
	if (SYSCLK == 24000000)
	{
//		CAN0DATH = 0x1C;					// 24.0mhz BRP=2 500kbps 87.5% sample point
//		CAN0DATL = 0x02;
		CAN0DATH = 0x54;					// 24.0mhz BRP=2 500kbps 68.8% sample point
		CAN0DATL = 0x03;
	}


}


/****************************************************************************
* Name: rx_can_frame
*  
* Description:  Receive CAN Data from the IF buffer
*              		
*				      
*****************************************************************************/

// Modified for 29BIT/11BIT operation
// MSB if set signals 29BIT XTD ID

void rx_can_frame (U8 IntReg)
{
	SFRPAGE = CAN0_PAGE;

	rxframe0[IntReg].MsgNum = IntReg;

	CAN0ADR = IF2CMDMSK;       						// Point to Command Mask 1
	CAN0DATL = 0x3F;           						// Config to READ CAN RAM, read data bytes, clr NewDat and IntPnd, arb ID
	CAN0ADR = IF2CMDRQST;      						// Point to Command Request Reg.
	CAN0DATL = rxframe0[IntReg].MsgNum;			// Move new data for RX from Msg Obj "MsgNum"

	if (IntReg >= RxBuffCnt[0])	// bogus Tx return now
		return;

	//Get Arb ID
	CAN0ADR = IF2ARB2;								//Point to Arbitration 2

	// check for XTD ID	
	if (CAN0DATH & 0x40)
	{
		// XTD ID
		CAN0ADR = IF2ARB2;								//Point to Arbitration 2
		rxframe0[IntReg].arbID = (((unsigned long)(CAN0DAT & 0x1FFF)) << 16) | 0x80000000;						
		CAN0ADR = IF2ARB1;							//Point to Arbitration 1
		rxframe0[IntReg].arbID += ((unsigned long)(CAN0DATH) << 8) | (CAN0DATL);
	}
	else
		// 11 BIT ID - ID28-18
		rxframe0[IntReg].arbID = ((unsigned long)(CAN0DAT & 0x1FFF)) >> 2;


	//Get Data Size
	CAN0ADR = IF2MSGC;								//Point to IF2 Message Control Register
	rxframe0[IntReg].sz = CAN0DATL & 0x0F;
	
	//Move new data to a buffer	
	CAN0ADR = IF2DATA1;     	   					// Point to 1st byte of Data Field	   
	rxframe0[IntReg].cdat[0] = CAN0DATL;
	CAN0ADR = IF2DATA1;     	   		   
	rxframe0[IntReg].cdat[1] = CAN0DATH;

	CAN0ADR = IF2DATA2;
	rxframe0[IntReg].cdat[2] = CAN0DATL;
	CAN0ADR = IF2DATA2;
	rxframe0[IntReg].cdat[3] = CAN0DATH;

	CAN0ADR = IF2DATB1;
	rxframe0[IntReg].cdat[4] = CAN0DATL;
	CAN0ADR = IF2DATB1;
	rxframe0[IntReg].cdat[5] = CAN0DATH;

	CAN0ADR = IF2DATB2;
	rxframe0[IntReg].cdat[6] = CAN0DATL;
	CAN0ADR = IF2DATB2;
	rxframe0[IntReg].cdat[7] = CAN0DATH;	

	RxCAN0 = 1;
}

/****************************************************************************
* Name: CAN Interrupt
*  
* Description:  Interrupt routine for CAN RX/TX
*              
*				      
*****************************************************************************/

void INT_CAN() interrupt 19							// CAN interrupt
{
	U8 IntReg;
	unsigned char hldSFRpage = SFRPAGE;
	U8 status;

	SFRPAGE = CAN0_PAGE;

	CAN0ADR = INTREG;

	IntReg = CAN0DATH;
	if (IntReg)										// status interrupt should never happen
	{
		IntReg = CAN0CN;
		status = CAN0STA;

		// Clear Rx & Tx bits
		CAN0STA = 0;
//		status = 0;

		SFRPAGE = hldSFRpage;
		return;
	}

	IntReg = CAN0DATL;								// read register interrupt number
	if (!IntReg || IntReg > 0x20 )					// SANITY CHECK
	{
		SFRPAGE = hldSFRpage;
		return;
	}


	// Get CAN status
	// status = CAN0STA;
	// Clear Rx & Tx bits
	CAN0STA = CAN0STA & 0xe7;

	if (CAN0STA & 0xE0)							// error interrupt ?
		{
		CAN0CN = 1;								// pull INIT high
		CAN0CN = 0xA;							// clear INIT flag
		}	


	if (!(CANintMode))
	{
		// CAN reception?
		rx_can_frame(IntReg);
	}
	else
	{
		CAN0STA &= ~0x8;						// Clear status bits
		CANintMode = 0;			
	}	

	SFRPAGE = hldSFRpage;
}

/****************************************************************************
* Name: tx_can_frame
*  
* Description:  Transmit CAN frame - Tx CAN Frame is global
*              
*       
*	returns timeout value, zero is failure		
*				      
*****************************************************************************/
unsigned char tx_can_frame (unsigned char KWPopts) 
{   		
	int timeout = 4096;						// roughley 2.5ms

	EIE2 &= ~0x01;                 			// disable Timer3 interrupts

	SFRPAGE = CAN0_PAGE;
	CAN0STA &= ~BIT3;						// Clear status bits

	CAN0ADR = IF1CMDMSK;          			// Point to Command Mask 1
	CAN0DAT = 0x00B7;             			// Config to WRITE to CAN RAM, write data bytes, set TXrqst/NewDat, Clr IntPnd
	CAN0ADR = IF1ARB1;

	// Check for 29BIT XTD ID - (BIT 31 is set)
	if (txframe.arbID & EXTFLG)
	{
		// 29BIT XTD ID
		CAN0DAT = (unsigned int)(txframe.arbID & 0x0000FFFF);
		CAN0DAT = ((unsigned int)((txframe.arbID & 0x7FFF0000) >> 16)) | 0xe000;
	}
	else
	{
		// 11BIT ID
		CAN0DAT = 0x0000;					// Arb1 Data			
		CAN0DAT = ((unsigned int)(txframe.arbID << 2)) | 0xA000;
	}

	CAN0ADR = IF1MSGC;				 		// Data Size

	CAN0DAT = 0x0880 | txframe.sz;			// w/ TxIE set (faster w/ TxIE set)						 
	if (!(KWPopts & 0x80))
		CANintMode = 1;	

	CAN0ADR = IF1DATA1;           			// Point to 1st byte of Data Field	
	CAN0DAT = (unsigned int)txframe.cdat[0] | ((unsigned int)txframe.cdat[1] << 8);
	CAN0DAT = (unsigned int)txframe.cdat[2] | ((unsigned int)txframe.cdat[3] << 8);
	CAN0DAT = (unsigned int)txframe.cdat[4] | ((unsigned int)txframe.cdat[5] << 8);
	CAN0DAT = (unsigned int)txframe.cdat[6] | ((unsigned int)txframe.cdat[7] << 8);

	CAN0ADR = IF1CMDRQST;         			// Point to Command Request Reg.
	CAN0DATL = txframe.MsgNum;     			// Move new data for TX to Msg Obj "MsgNum"

	EIE2 |= 0x01;                 			// enable Timer3 interrupts

	if (!(KWPopts & 0x80))
	{
		while (CANintMode && timeout)		// interrupt will clear CANintMode
			timeout--;
	}

	return(timeout);
}

/****************************************************************************
* Name: init_msg_object_grx
*  
* Description:  Init Message Object for general RX
*              
*       
*		
*				      
*****************************************************************************/
/*
void init_msg_object_grx (unsigned char xtd)
{
	SFRPAGE = CAN0_PAGE;
	CAN0ADR = IF2CMDMSK;        		  	// Point to Command Mask 1
	//CAN0DAT = 0x00B8;						// Set to WRITE, and alter all Msg Obj except ID MASK and data bits
	CAN0DAT = 0x00f8;						// set to write 					
	
	CAN0ADR = IF2MSK1;						// set Mask
	CAN0DAT = 0x0000;
	CAN0ADR = IF2MSK2;
	CAN0DAT = 0x0000;

	CAN0ADR = IF2ARB1;           		 	// Point to arbitration1 register
	CAN0DAT = 0x0000;
	if (xtd)
		CAN0DAT = (0 << 2) | 0xc000;
	else
		CAN0DAT = (0 << 2) | 0x8000;
		
	CAN0DAT = 0x1480;             			// Msg Cntrl: set RX IE, remote frame function not enabled
	CAN0ADR = IF2CMDRQST;       		  	// Point to Command Request reg.
	CAN0DATL = GLOBAL_RX;       		   	// Select Global Msg Obj  --initiates write to Msg Obj
   											// 3-6 CAN clock cycles to move IF register contents to the Msg Obj in CAN RAM.
}
*/

//Initialize Message Object for RX
void init_msg_object_rx (unsigned char MsgNum,unsigned long arbID) 
{
	SFRPAGE = CAN0_PAGE;
	CAN0ADR = IF2CMDMSK;  		        	// Point to Command Mask 1
	CAN0DAT = 0x00B8;     		        	// Set to WRITE, and alter all Msg Obj except ID MASK and data bits
	CAN0ADR = IF2ARB1;    		        	// Point to arbitration1 register

	if (arbID & EXTFLG)
	{
		// 29BIT XTD ID
		CAN0DAT = (unsigned int)(arbID & 0x0000FFFF);
		CAN0DAT = ((unsigned int)((arbID & 0x7FFF0000) >> 16)) | 0xc000;
	}
	else
	{
		// 11BIT ID
		CAN0DAT = 0x0000;
		CAN0DAT = (arbID << 2) | 0x8000;
	}


	CAN0DAT = 0x0480;     		        	// Msg Cntrl: set RX IE, remote frame function not enabled
	CAN0ADR = IF2CMDRQST; 		        	// Point to Command Request reg.
	CAN0DATL = MsgNum;    		        	// Select Msg Obj passed into function parameter list --initiates write to Msg Obj
 											// 3-6 CAN clock cycles to move IF register contents to the Msg Obj in CAN RAM.
}

