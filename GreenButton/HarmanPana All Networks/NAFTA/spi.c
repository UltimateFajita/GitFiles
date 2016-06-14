#include "c8051F040.h"
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
#include "main.h"
#include "spi.h"

// global's remember cygnal world local's maintain data space
U8 hldSFRPAGE;
extern CANFRAME xdata txframe;
extern CANFRAME xdata rxframe1[RX1BUFFERS];
extern U32 xdata RxCAN1IDs[RX1BUFFERS];
extern U8 xdata RxBuffCnt[2];

//////////////////////////////////////////////////////////////////////////////////
// SPI Helper Functions
//////////////////////////////////////////////////////////////////////////////////

// Initialize SPI Port for CAN1 Communications
void SPI_init()
{
	SFRPAGE = SPI0_PAGE;

	SPIEN = 0;								// Disable SPI

	SPI0CFG = 0x40;							// set Clock Polarity=0, Clock Phase=0, Master Mode=1

	SPI0CKR = 0;							// set bit rate	0 = 9M SPI_SPEED

	SPI0CN = 0x0F;							// set 4-wire Single-Master (with NSS high), enable SPI
}


// Receive SPI Byte with dummy xmit 0xff 
U8 SPI_Rxbyte(void)
{	
//	SFRPAGE = SPI0_PAGE;
	
	SPIF = 0;									// clear end of xfer indicator
	SPI0DAT = 0xff;								// load with dummy data
	while (!SPIF);								// wait for transaction complete
	return SPI0DAT;								// return received data byte

}

// Transmit & Receive SPI byte
U8 SPI_Txbyte(U8 value)
{
//	SFRPAGE = SPI0_PAGE;

	SPIF = 0;									// clear end of xfer indicator
	SPI0DAT = value;							// load with dummy data
	while (!SPIF);								// wait for transaction complete
	return SPI0DAT;								// return received data byte
}

// (dis)Enable SPI CS and SFRPAGE for CAN1
void spiCAN(U8 Enable)
{
	if (Enable)
	{
		hldSFRPAGE = SFRPAGE;
		SFRPAGE = SPI0_PAGE;
		NSSMD0 = 0;
	}
	else
	{
		NSSMD0 = 1;
		SFRPAGE = hldSFRPAGE;
	}
}

#define spi_readwrite	SPI_Txbyte
#define spi_read		SPI_Rxbyte

/*********************************************************************************************************
** Function name:           CAN1_reset
** Descriptions:            reset the device
*********************************************************************************************************/
void CAN1reset(void)                                      
{
    spiCAN(1);
    spi_readwrite(MCP_RESET);
    spiCAN(0);

    codedelay(20);
}

/*********************************************************************************************************
** Function name:           CAN1readRegister
** Descriptions:            read register
*********************************************************************************************************/
U8 CAN1readRegister(U8 address)                                                                     
{
	U8 ret;

	spiCAN(1);
	ret = 0x55;				// dummy lines added compiler was being stupid
	spi_readwrite(MCP_READ);
	ret = 0xAA;				// dummy lines added compiler was being stupin
    spi_readwrite(address);
 	ret = spi_read();

	spiCAN(0);
    return(ret);
}

/*********************************************************************************************************
** Function name:           CAN1readRegisterS
** Descriptions:            read registerS
*********************************************************************************************************
void CAN1readRegisterS(U8 address, U8 values[], U8 n)
{
	U8 i;
	
	spiCAN(1);
	i = 0xAA;				// dummy lines added compiler was being stupid
	spi_readwrite(MCP_READ);
	spi_readwrite(address);
	// mcp2515 has auto-increment of address-pointer
	for (i=0; i<n; i++) 
    {
		values[i] = spi_read();
	}

	spiCAN(0);
}*/

/*********************************************************************************************************
** Function name:           CAN1setRegister
** Descriptions:            set register
*********************************************************************************************************/
void CAN1setRegister(U8 address, U8 value)
{
	spiCAN(1);
    spi_readwrite(MCP_WRITE);
    spi_readwrite(address);
    spi_readwrite(value);

	spiCAN(0);
}

/*********************************************************************************************************
** Function name:           CAN1setRegisterS
** Descriptions:            set registerS
*********************************************************************************************************/
void CAN1setRegisterS(U8 address, U8 values[], U8 n)
{
	U8 i;

	spiCAN(1);
    spi_readwrite(MCP_WRITE);
    spi_readwrite(address);
       
    for (i=0; i<n; i++) 
    {
        spi_readwrite(values[i]);
    }

	spiCAN(0);
}

/*********************************************************************************************************
** Function name:           CAN1modifyRegister
** Descriptions:            set bit of one register
*********************************************************************************************************/
void CAN1modifyRegister(U8 address, U8 mask, U8 value)
{
	spiCAN(1);
    spi_readwrite(MCP_BITMOD);
    spi_readwrite(address);
    spi_readwrite(mask);
    spi_readwrite(value);

	spiCAN(0);
}

/*********************************************************************************************************
** Function name:           CAN1readStatus
** Descriptions:            read mcp2515's Status
*********************************************************************************************************/
U8 CAN1readStatus(void)                             
{
	U8 ret;
	spiCAN(1);
	spi_readwrite(MCP_READ_STATUS);
	ret = spi_read();

	spiCAN(0);
	return (ret);
}

/*********************************************************************************************************
** Function name:           CAN1readRxStat
** Descriptions:            read Rx Status
*********************************************************************************************************/
U8 CAN1readRxStat(void)                             
{
	U8 ret;
	spiCAN(1);
	spi_readwrite(MCP_RX_STATUS);
	ret = spi_read();

	spiCAN(0);
	return (ret);
}

/*********************************************************************************************************
** Function name:           CAN1setCANCTRL_Mode
** Descriptions:            set control mode
*********************************************************************************************************/
U8 CAN1setCANCTRL_Mode(U8 newmode)
{
    U8 i;

    CAN1modifyRegister(MCP_CANCTRL, MODE_MASK, newmode);

    i = CAN1readRegister(MCP_CANCTRL);
    i &= MODE_MASK;

    if ( i == newmode ) 
    {
        return MCP2515_OK;
    }

    return MCP2515_FAIL;
}

/*********************************************************************************************************
** Function name:           CAN1configRate
** Descriptions:            set baudrate
*********************************************************************************************************/
U8 CAN1configRate(U8 canSpeed)            
{
    U8 set, cfg1, cfg2, cfg3;
    set = 1;
    switch (canSpeed) 
    {
        case (CAN_1000KBPS):
        cfg1 = 0x40;			// SJW = 1 (2 TQ's), BRP = 0 -> .125uSec / TQ
        cfg2 = 0x89;			// PRSEG = 1 -> PropSeg is 2 TQ's, PHSEG1 = 1 -> Phase1 is 2 TQ's 
        cfg3 = 2;				// PHSEG2 = 2 -> Phase2 is 3 TQ's
        break;					// Bit Time is #TQ's * BRP :: 1+2+2+3 = 8 TQ's and 8*.125uSec => 1000Kbits

        case (CAN_500KBPS):
        cfg1 = 0x40;			// SJW = 1 (2 TQ's), BRP = 0 -> .125uSec / TQ
        cfg2 = 0xA3;			// PRSEG = 3 -> PropSeg is 4 TQ's, PHSEG1 = 4 -> Phase1 is 5 TQ's
        cfg3 = 5;				// PHSEG2 = 5 -> Phase2 is 6 TQ's
        break;					// Bit Time is #TQ's * BRP :: 1+4+5+6 = 16 TQ's and 16*.125uSec => 500Kbits

        case (CAN_125KBPS):
        cfg1 = 0x43;			// SJW = 1 (2 TQ's), BRP = 3 -> .5uSec / TQ
        cfg2 = 0xA3;			// PRSEG = 3 -> PropSeg is 4 TQ's, PHSEG1 = 4 -> Phase1 is 5 TQ's
        cfg3 = 5;				// PHSEG2 = 5 -> Phase2 is 6 TQ's
        break;					// Bit Time is #TQ's * BRP :: 1+4+5+6 = 16 TQ's and 16*.5uSec => 125Kbits
        
        case (CAN_83KBPS):
        cfg1 = 0x45;			// SJW = 1 (2 TQ's), BRP = 5 -> .75uSec / TQ
        cfg2 = 0xA3;			// PRSEG = 3 -> PropSeg is 4 TQ's, PHSEG1 = 4 -> Phase1 is 5 TQ's 
        cfg3 = 5;				// PHSEG2 = 5 -> Phase2 is 6 TQ's
        break;					// Bit Time is #TQ's * BRP :: 1+4+5+6 = 16 TQ's and 16*.75uSec => 83.33Kbits

        case (CAN_50KBPS):
        cfg1 = 0x47;			// SJW = 1 (2 TQ's), BRP = 7 -> 1uSec / TQ
        cfg2 = 0xAD;			// PHSEG1 = 5 -> Phase1 is 6 TQ's, PRSEG = 5 -> PropSeg is 6 TQ's
        cfg3 = 6;				// PHSEG2 = 6 -> Phase2 is 7 TQ's
        break;					// Bit Time is #TQ's * BRP :: 1+6+6+7 = 20 TQ's and 20*1uSec => 50Kbits


        default:
        set = 0;
        break;
    }

    if (set) {
        CAN1setRegister(MCP_CNF1, cfg1);
        CAN1setRegister(MCP_CNF2, cfg2);
        CAN1setRegister(MCP_CNF3, cfg3);
        return MCP2515_OK;
    }
    else {
        return MCP2515_FAIL;
    }
}

/*********************************************************************************************************
** Function name:           CAN1write_id
** Descriptions:            write can id
*********************************************************************************************************/
void CAN1write_id( U8 mcp_addr, U32 id )
{
    U16 canid;
    U8 tbufdata[4];

    canid = (U16)(id & 0x0FFFF);

    if ( id & EXTFLG) 
    {
        tbufdata[MCP_EID0] = (U8) (canid & 0xFF);
        tbufdata[MCP_EID8] = (U8) (canid >> 8);
        canid = (U16)(id >> 16);
        tbufdata[MCP_SIDL] = (U8) (canid & 0x03);
        tbufdata[MCP_SIDL] += (U8) ((canid & 0x1C) << 3);
        tbufdata[MCP_SIDL] |= MCP_TXB_EXIDE_M;
        tbufdata[MCP_SIDH] = (U8) (canid >> 5 );
    }
    else 
    {
        tbufdata[MCP_SIDH] = (U8) (canid >> 3 );
        tbufdata[MCP_SIDL] = (U8) ((canid & 0x07 ) << 5);
        tbufdata[MCP_EID0] = 0;
        tbufdata[MCP_EID8] = 0;
    }
    CAN1setRegisterS( mcp_addr, tbufdata, 4 );
}

/*********************************************************************************************************
** Function name:           CAN1read_id
** Descriptions:            read can id
*********************************************************************************************************
void CAN1read_id( U8 mcp_addr, U32* id )
{
    U8 tbufdata[4];

    CAN1readRegisterS( mcp_addr, tbufdata, 4 );

    *id = (tbufdata[MCP_SIDH]<<3) + (tbufdata[MCP_SIDL]>>5);

    if ( (tbufdata[MCP_SIDL] & MCP_TXB_EXIDE_M) ==  MCP_TXB_EXIDE_M ) 
    {
                                                                        // extended id                  
        *id = (*id<<2) + (tbufdata[MCP_SIDL] & 0x03);
        *id = (*id<<8) + tbufdata[MCP_EID8];
        *id = (*id<<8) + tbufdata[MCP_EID0];
        *id |= EXTFLG;
    }
}*/

/*********************************************************************************************************
** Function name:           CAN1initCANBuffers
** Descriptions:            init canbuffers
*********************************************************************************************************/
void CAN1initCANBuffers(void)
{
    U8 i;
	U8 a1, a2, a3;
    
    U32 ulMask = 0x1FFFFFFF, ulFilt = 0;


    CAN1write_id(MCP_RXM0SIDH, ulMask | EXTFLG);			/*Set both masks to 0           */
    CAN1write_id(MCP_RXM1SIDH, ulMask | EXTFLG);			/*Mask register ignores ext bit */
    
                                                                        /* Set all filters to 0         */
    CAN1write_id(MCP_RXF0SIDH, ulFilt | EXTFLG);			/* RXB0: extended               */
    CAN1write_id(MCP_RXF1SIDH, ulFilt);			/* RXB1: standard               */
    CAN1write_id(MCP_RXF2SIDH, ulFilt | EXTFLG);			/* RXB2: extended               */
    CAN1write_id(MCP_RXF3SIDH, ulFilt);			/* RXB3: standard               */
    CAN1write_id(MCP_RXF4SIDH, ulFilt | EXTFLG);
    CAN1write_id(MCP_RXF5SIDH, ulFilt);
    CAN1setRegister(MCP_RXB0CTRL, 0);
    CAN1setRegister(MCP_RXB1CTRL, 0);

                                                                        /* Clear, deactivate the three  */
                                                                        /* transmit buffers             */
                                                                        /* TXBnCTRL -> TXBnD7           */
    a1 = MCP_TXB0CTRL;
    a2 = MCP_TXB1CTRL;
    a3 = MCP_TXB2CTRL;
    for (i = 0; i < 14; i++) {                                          /* in-buffer loop               */
        CAN1setRegister(a1, 0);
        CAN1setRegister(a2, 0);
        CAN1setRegister(a3, 0);
        a1++;
        a2++;
        a3++;
    }
}

/*********************************************************************************************************
** Function name:           CAN1init
** Descriptions:            init the device
*********************************************************************************************************/
U8 CAN1init(U8 canSpeed)                       
{

	U8 res;

    CAN1reset();



    if (res = CAN1setCANCTRL_Mode(MODE_CONFIG))
		return res;
                                                                                         
    if(res = CAN1configRate(canSpeed))						// set boadrate
    	return res;

                                                                                     
    CAN1initCANBuffers();									// init canbuffers
	CAN1setRegister(MCP_BFPCTRL,0x0C);						// init RxPins as digital outputs
                                                                                   
    CAN1setRegister(MCP_CANINTE, MCP_RX0IF | MCP_RX1IF);	// interrupt mode

    /* enable both receive-buffers to receive messages with std. and ext. identifiers
	   and enable rollover          */
    CAN1modifyRegister(MCP_RXB0CTRL,
    MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK,
    MCP_RXB_RX_STDEXT | MCP_RXB_BUKT_MASK );
    CAN1modifyRegister(MCP_RXB1CTRL, MCP_RXB_RX_MASK,
    MCP_RXB_RX_STDEXT);
                                                                    
	CAN1init_FiltersNMasks(RX0_MASK,0);		// both masks to except ALL
	CAN1init_FiltersNMasks(RX1_MASK,0);

    if (res = CAN1setCANCTRL_Mode(MODE_NORMAL))				// enter normal mode                                                                
    	return res;

    return res;
}

/*********************************************************************************************************
** Function name:           CAN1begin
** Descriptions:            init can and set speed
*********************************************************************************************************/
U8 CAN1begin(U8 speedset)
{

    if (CAN1init(speedset) == MCP2515_OK)
		return CAN_OK;
	else
		return CAN_FAILINIT;
}

/*********************************************************************************************************
** Function name:           CAN1init_Mask
** Descriptions:            init canid Masks
*********************************************************************************************************/
U8 CAN1init_FiltersNMasks(U8 num, U32 ulData)
{
    U8 res = MCP2515_OK;

    if ((res = CAN1setCANCTRL_Mode(MODE_CONFIG)) > 0)
		return res;
    
    switch (num)
	{
		case 0:	// filter 0
		CAN1write_id(MCP_RXF0SIDH,ulData);
		break;

		case 1:	// filter 1
		CAN1write_id(MCP_RXF1SIDH,ulData);
		break;

		case 2:	// filter 2
		CAN1write_id(MCP_RXF2SIDH,ulData);
		break;

		case 3:	// filter 3
		CAN1write_id(MCP_RXF3SIDH,ulData);
		break;

		case 4:	// filter 4
		CAN1write_id(MCP_RXF4SIDH,ulData);
		break;

		case 5:	// filter 5
		CAN1write_id(MCP_RXF5SIDH,ulData);
		break;

		case 6:	// mask 0
		CAN1write_id(MCP_RXM0SIDH,ulData);
		break;

		case 7:	// mask 1
		CAN1write_id(MCP_RXM1SIDH,ulData);
		break;
	}
    
    return (CAN1setCANCTRL_Mode(MODE_NORMAL));
}

/*********************************************************************************************************
** Function name:           CAN1read_canMsg
** Descriptions:            read CAN1 msg into rxbuf
*********************************************************************************************************/
U8 CAN1read_canMsg(U8 RxBuff)
{
	U8 i,j;
	U32 canID;

	spiCAN(1);
	spi_readwrite(RxBuff);

    i=spi_read();		
	j = spi_read();
	canID = (U32)(i<<3) + (j>>5);

    if ( (j & MCP_TXB_EXIDE_M) ==  MCP_TXB_EXIDE_M )			// extended id 
    {
                                                                        
        canID = (canID<<2) + (j & 0x03);
        canID = (canID<<8) + (U32)spi_read();
        canID = (canID<<8) + (U32)spi_read();
        canID |= EXTFLG;
    }
	else
	{
	i = spi_read();		// not used but read anyways
	j = spi_read();
	}

	for (i = 0 ; i < RxBuffCnt[1] && RxCAN1IDs[i] != canID ; i++)
		;	

	if (i < RxBuffCnt[1])
	{
		rxframe1[i].arbID = canID;
		rxframe1[i].sz = spi_read();
		for (j = 0 ; j < rxframe1[i].sz ; j++)
			rxframe1[i].cdat[j] = spi_read();
	}				


	spiCAN(0);

return(i);
}


/*********************************************************************************************************
** Function name:           CAN1getNextFreeTXBuf
** Descriptions:            get Next free txbuf
*********************************************************************************************************/
U8 CAN1getNextFreeTXBuf(U8 *txbuf_n)
{
    U8 i;
	U8 ctrlval;
    U8 ctrlregs[MCP_N_TXBUFFERS] = { MCP_TXB0CTRL, MCP_TXB1CTRL, MCP_TXB2CTRL };

    *txbuf_n = 0x00;

                                                                        // check all 3 TX-Buffers       
    for (i=0; i<MCP_N_TXBUFFERS; i++)
	{
        ctrlval = CAN1readRegister( ctrlregs[i] );
        if ( (ctrlval & MCP_TXB_TXREQ_M) == 0 )
		{
            *txbuf_n = ctrlregs[i];                                   // return SIDH-address of Buffer 
                                                                                                    
            return MCP2515_OK;
        }
    }
    return MCP_ALLTXBUSY;
}


/*********************************************************************************************************
** Function nam+e:           CAN1write_canMsg
** Descriptions:            write msg
*********************************************************************************************************/
void CAN1write_canMsg( U8 buffer_sidh_addr)
{
    U16 canid;
	U8 tbufdata[4];
	U8 i;

    canid = (U16)(txframe.arbID & 0x0FFFF);
    if ( txframe.arbID & EXTFLG) 
    {
        tbufdata[MCP_EID0] = (U8) (canid & 0xFF);
        tbufdata[MCP_EID8] = (U8) (canid >> 8);
        canid = (U16)(txframe.arbID >> 16);
        tbufdata[MCP_SIDL] = (U8) (canid & 0x03);
        tbufdata[MCP_SIDL] += (U8) ((canid & 0x1C) << 3);
        tbufdata[MCP_SIDL] |= MCP_TXB_EXIDE_M;
        tbufdata[MCP_SIDH] = (U8) (canid >> 5 );
    }
    else 
    {
        tbufdata[MCP_SIDH] = (U8) (canid >> 3 );
        tbufdata[MCP_SIDL] = (U8) ((canid & 0x07 ) << 5);
        tbufdata[MCP_EID0] = 0;
        tbufdata[MCP_EID8] = 0;
    }

	spiCAN(1);
	switch(buffer_sidh_addr)
	{
		case MCP_TXB0CTRL:
			spi_readwrite(MCP_LOAD_TX0);
			break;
		case MCP_TXB1CTRL:
			spi_readwrite(MCP_LOAD_TX1);
			break;
		case MCP_TXB2CTRL:
			spi_readwrite(MCP_LOAD_TX2);
			break;
	}
	for (i = 0 ; i < 4 ; i++)
		spi_readwrite(tbufdata[i]);		// write ID
	spi_readwrite(txframe.sz);			// write size (DLC)
	for (i = 0 ; i < txframe.sz ; i++)
		spi_readwrite(txframe.cdat[i]);	// write data

	spiCAN(0);

}

/*********************************************************************************************************
** Function name:           CAN1start_transmit
** Descriptions:            start transmit
*********************************************************************************************************/
void CAN1start_transmit(U8 mcp_addr)              
{
	spiCAN(1);

	switch(mcp_addr)
	{
		case MCP_TXB0CTRL:
			spi_readwrite(MCP_RTS_TX0);
			break;
		case MCP_TXB1CTRL:
			spi_readwrite(MCP_RTS_TX1);
			break;
		case MCP_TXB2CTRL:
			spi_readwrite(MCP_RTS_TX2);
			break;
	}

	spiCAN(0);
}

/****************************************************************************
* Name: CAN1 Interrupt
*  
* Description:  Interrupt routine for CAN1 RX/TX
*              
*				      
*****************************************************************************/
extern bit RxCAN1;

void INT_CAN1() interrupt 0				// Ext0 (CAN1 interrupt)
{
	RxCAN1 = 1;
	EX0 = 0;	// dis-able CAN1 interupt
}			