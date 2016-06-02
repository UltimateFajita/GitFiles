#define U8 unsigned char
#define U16 unsigned int
#define U32 unsigned long

#define ONESEC 998				// 1000 msticks to one second (998 tested to 1sec)
#define RX0BUFFERS 32			// CAN0 has 32 hardware buffers
#define RX1BUFFERS 16			// CAN1 buffers are only software 2 hardware are used
#define TXBUFFS 3

#define MAXBUTTONS 20

//===========================================================================
#define BIT3			8
#define BAUDRATE		115200		// Baud rate of UART in bps

#define SYSCLK			18000000	// Output of PLL derived from (INTCLK*2)
#define SAR_CLK			2500000		// Desired SAR clock speed
#define INT_DEC			25600			// Integrate and decimate ratio
//#define INT_DEC		204			// 204 samples time = 10msecs

#define ANALOG_ERRORSCALE	669		// 3345 = 5V 3345/5 = 669
#define ANALOG_INPUTS	3			// Number of AIN pins to measure
                                    // (min=1, max=8)
//===========================================================================
// Hardware output pin definitions
//---------------------------------------------------------------------------
sbit LED1           = P2^3;	// led output pin 
sbit SEL_CAN_C      = P2^2;	// CAN select pin 0 means can B, 1 means CAN c/i
sbit ROW1			= P0^5;	// Row1 mux to switches
sbit ROW2			= P0^6;	// Row2 mux
sbit ROTARY_IGN		= P3^6;	// Rotary IGN source
sbit ROTARY_SOFT	= P3^7;	// Rotary Soft source
sbit RELAY1			= P3^4;	// Relay 1 output
sbit RELAY2			= P3^5;	// Relay 2 output


//---------------------------------------------------------------------
// 16-bit SFR Definitions for 'F04x
//--------------------------------------------------------------------
sfr16 DP       = 0x82;                 // data pointer
sfr16 RCAP2    = 0xCA;                 // Timer2 reload/capture value
sfr16 RCAP3    = 0xCA;                 // Timer3 reload/capture value
sfr16 RCAP4    = 0xCA;                 // Timer4 reload/capture value
sfr16 TMR2     = 0xCC;                 // Timer2 counter/timer
sfr16 TMR3     = 0xCC;                 // Timer3 counter/timer
sfr16 TMR4     = 0xCC;                 // Timer4 counter/timer
sfr16 DAC1     = 0xD2;                 // DAC1 data
sfr16 CAN0DAT  = 0xD8;                 // CAN data window


typedef struct tCANFRAME
{
	U8 MsgNum;
	U8 sz;
	U32 arbID;
	U8 cdat[8];
} CANFRAME;


typedef struct tBUTTABLE	// size 22 * 20 max = 440d = 1B8x (800-9B7 space)
{
	U8 button,cnt;
	U32 ID;
	U8 mask[8],value[8];
} BUTTABLE;

typedef struct tMSGTABLE	// @size 26 * x 
{
	U32 ID;
	U16 Rate,ticks;
	U8 cnt,Altered;
	U8 dat[8],Restore[8];
} MSGTABLE;

typedef struct tRxMESG		// size 40 * y 
{							// burnin bytes up fast
	U8 RxNum;
	U8 inMask[8],inValue[8];
	U32 outID;
	U8 outMask[8],outValue[8];
	U8 incnt,outcnt,altercnt;
} RxMESG;

#define EXTFLG 0x8000000
#define PORTFLG 0x40000000

// relay defines
#define REL_TOG 0x8000		// Toggle control
#define REL_NOT 0x4000		// invert logic
#define REL_PULSE 0x2000	// pulsed output auto off next pass
#define REL_IGN 0x200
#define REL_BUTT 0x100
#define REL_PROG 0x300
#define REL_RX 0x400
#define REL_TX 0x500


// function deffs
// main.c
U8 MemManage(U8 Bcnt,U8 Mcnt,U8 Rcnt38,U16 FirstAddress);
void ClearUserVars(void);
void config_IO(void);
void CheckButts(void);
void CheckRelay();
U8 AddMessage(U8 port,U32 ID,U16 Rate,U8 cnt,U8 dat[8]);
void AlterMessageByte(U8 port,U32 ID,U8 index,U8 *value,U8 numbytes,U8 count);
U8 SimpleButtonConfig(U8 button,U8 port,U32 ID,U8 mask[8],U8 value[8],count);
U8 RxMessageConfig1(U8 port,U32 ID,U8 mask[8],U8 value[8],U8 incnt);
void RxMessageConfig2(U8 index,U8 outport,U32 outID,U8 outmask[8],U8 outvalue[8],U8 outcnt,U8 altercnt);
void ProcessRx(U8 port,U8 index);
void RelayCtrl(U8 num,U8 on);

// user.c
void User1(void);
void User2(void);
void User3(void);
void User4(void);
void User5(void);
void User6(void);
void User7(void);
void User8(void);

// timers.c
void Oscillator_Init(void);
void mSecDelay(unsigned int msec);
void codedelay(unsigned int delay);
void Start_Timer3(U16);
U8 Rotary2value(U8 inbits,U8 old);
void ReadInputs();
void CheckMesgs();

// can.c
void TxFrame(U8 port);
U8 initRxFrameID(U8 port, U32 arbID);
void clear_message_object (U8 port,unsigned char,unsigned char); 
void config_CAN_timing(unsigned int);
void rx_can_frame (U8 IntReg);
unsigned char tx_can_frame (unsigned char KWPopts); 
void init_msg_object_rx (unsigned char MsgNum,unsigned long);

