#include "c8051F040.h"
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
#include "main.h"

//U16 OneSecConst;
U32 msticks;
extern bit RxCAN1;
extern U8 xdata RelayChange;
extern U16 xdata RelayCnf[4];
extern MSGTABLE xdata *Mesgs;
extern U8 xdata MesgCnt;
extern CANFRAME xdata txframe;
extern CANFRAME xdata OneTime[3];

void Oscillator_Init(void)
{
	char	SFPRPAGE_SAVE = SFRPAGE;	// Save Current SFR Page

	////////////////////////////////////////
	// switch to external oscillator
	////////////////////////////////////////
	SFRPAGE = CONFIG_PAGE;	// switch to config page to config oscillator
	
	OSCXCN  = 0x67;     // start external oscillator; 18.0/24.0 MHz Crystal
	codedelay(255);		// delay about 1ms  

	while ((OSCXCN & 0x80) == 0);   

	CLKSEL |= 0x01;		// switch to external oscillator

	SFRPAGE = SFPRPAGE_SAVE;	// Restore SFR page
}

// MilliSec Delay -  delay function
void mSecDelay(unsigned int delay)
{
	delay = delay + (int)msticks;

													// did delay wrap low? then
	while (delay < (int)msticks && !RxCAN1)			// wait till msticks wraps once
	;												// delay should be > msticks to start

	while ((int)msticks < delay && !RxCAN1)			// wait for msticks >= delay
	;
}

void codedelay(unsigned int delay)
{
	while (delay && !RxCAN1)
	{
		delay--;
	}
} 


// Start Timer3
void Start_Timer3(U16 OneSecConst)
{
	unsigned int Reload;

	// Configure Timer3 to auto-reload and generate an interrupt at interval
	// specified by <counts> using SYSCLK/12 as its time base.
   	SFRPAGE = TMR3_PAGE;            		// Switch to Timer 3 page

	Reload = SYSCLK / OneSecConst;
//	Reload = SYSCLK * .000256;
	TMR3CN = 0x00;                   		// Stop Timer3; Clear TF3;
	TMR3CF = 0x8;                       	// use SYSCLK as timebase GLB

	RCAP3   = -Reload;         				// Init reload values
	TMR3    = 0xffff;               		// set to reload immediately

	// debug - Timer3 high priority
	//EIP2 |= 0x01;

	EIE2 |= 0x01;                 			// enable Timer3 interrupts
	TR3 = 1;                        		// start Timer3
											// Init Timer3 to generate interrupts - 256uSec,
											// (4608 ticks @18M) or (6144 @24M)
				 	 						// use SYSCLK instead of SYSCLK/12 for accuracy
											// This leads to msticks being bytes 4-1 of 5 byte timer
											// byte 0 is TMR3 / 18 (GLB)
	msticks = 0;
}

extern U32 keys,keyChange;
void INT_T3_OV() interrupt 14					// Timer 3 overflow
{
	TF3 = 0;                   	    			// clear TF3
	msticks++;

	ReadInputs();								// read input selects and buttons.

	CheckMesgs();
}


U8 Rotary2value(U8 inbits,U8 old)
{
	switch (inbits)
		{
		case 1:
			inbits = 5;
			break;
		case 2:
			inbits = 6;
			break;
		case 4:
			inbits = 7;
			break;
		case 8:
			inbits = 8;
			break;
		case 0x10:
			inbits = 1;
			break;
		case 0x20:
			inbits = 2;
			break;
		case 0x40:
			inbits = 3;
			break;
		case 0x80:
			inbits = 4;
			break;
		default:
			inbits =old;
		}
	return (inbits);
}

void ReadInputs()
{
	U32 Col1,Col2;
	U16 col;
	U8 Ign,Soft;
	U8 port1,port2,port3,temp;

	// Read Row 1 of buttons and Read Ignition Rotary
	ROW1 = 0;
	ROTARY_IGN = 0;
	codedelay(50);

	port1 = P1;
	port2 = P2; 
	port3 = P3;
	ROW1 = 1;
	ROTARY_IGN = 1;

	col = (U16)port1 + ((U16)(port2&0x3)<<8);
	Col1 = (U32)(((~col) & 0x3FF))<<18;

	temp = (port2 & 0xF0) + (port3 & 0xF);
	Ign = Rotary2value(~temp,((keys>>4)&0xF));

	// Read Row 2 of buttons and Read Soft Rotary
	ROW2 = 0;
	ROTARY_SOFT = 0;
	codedelay(50);
	
	port1 = P1;
	port2 = P2; 
	port3 = P3;
	ROW2 = 1;
	ROTARY_SOFT = 1;

	col = port1 + ((port2&0x3)<<8);
	Col2 = (U32)(((~col) & 0x3FF))<<8;

	temp = (port2 & 0xF0) + (port3 & 0xF);
	Soft = Rotary2value(~temp,(keys&0xF));

	keyChange |= keys ^ (Col1+Col2+((U32)Ign<<4)+(U32)Soft);	// set new changes
	keys = Col1+Col2+((U32)Ign<<4)+(U32)Soft;
}


void CheckMesgs()
{
	U8 i,j,port;
	bit found = 0;

	for (i = 0 ; i < TXBUFFS ; i++)
	{
		if (OneTime[i].arbID)
		{
			if (OneTime[i].arbID & PORTFLG)
				port = 1;
			else port = 0;
			txframe.arbID = (OneTime[i].arbID & 0x9FFFFFFF);
			txframe.sz = OneTime[i].sz;
			for (j = 0 ; j < txframe.sz ; j++)
				txframe.cdat[j] = OneTime[i].cdat[j];

			TxFrame(port);
			OneTime[i].arbID = 0;
			found = 1;
		}
	}

	for (i = 0 ; i < MesgCnt ; i++)
	{
		if ((Mesgs+i)->Rate <= (Mesgs+i)->ticks && !found)	// output
		{
			(Mesgs+i)->ticks=0;

			if ((Mesgs+i)->ID & PORTFLG) port = 1; else port = 0;
			txframe.arbID = (Mesgs+i)->ID & 0x9FFFFFFF;												
			txframe.sz = (Mesgs+i)->cnt;
			for (j = 0 ; j < txframe.sz ; j++)
				txframe.cdat[j] = (Mesgs+i)->dat[j];

			TxFrame(port);
			found = 1;

			if ((Mesgs+i)->Altered)
			{
				if ((Mesgs+i)->Altered != 0xFF)	// not held data
				{
					(Mesgs+i)->Altered--;
					if (!(Mesgs+i)->Altered)	// time to restore data
					{
						for (j = 0 ; j < (Mesgs+i)->cnt ; j++)
							(Mesgs+i)->dat[j] = (Mesgs+i)->Restore[j];
					}
				}
			}

			for (j = 0 ; j < 4 ; j++)
			{
				if ((RelayCnf[j]&0xF00) == REL_TX) // Relay configured to TxMesg
				{
					if ((RelayCnf[j]&0xFF) == i)		// outputing correct message
						RelayChange |= 1<<j;			// flag for relay change
				}
			}
		}
		else
			(Mesgs+i)->ticks++;
	}
}