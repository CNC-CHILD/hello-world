//###########################################################################
//--------M�¶���----------ԭ������------2015-04-04--------------------------------
//--------M�¶���----------SPI���߲���ʵ��----оƬW25Q16��������------------------

//       ��ο�TI�ٷ��ֲᡶsprug71b--SPI.pdf��
//-------�ο�оƬ�ֲᡶW25Q16BV.pdf��


/*--------------------�������ý���----����LED��������5050 ��ɫLED��Ӧ��IO�ڽ���-------------------------
//ʹ��TI�����DELAY_US()�������裺
//1. ������� DSP2803x_usDelay.asm�ļ�
//2. ����extern Uint16 RamfuncsLoadSize����RamfuncsRunStart��RamfuncsLoadStart��DSP2803x_GlobalPrototypes.h��������
//3. ʹ��memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (Uint32)&RamfuncsLoadSize);��ʼ����RAM���д���
//4. ����DELAY_US()���ɣ��βη�Χ�� long doubl����λΪus ��1��Ϊ1000���룬1����Ϊ1000΢�

//## ����ԭ��ͼ��Ӳ����Դ��������#
//����	    IO��	                ˵��
//D400	    OPWM7B/LED_BK/GPIO41	IO���
//D401	    LED1/GPIO34             IO���
//D402	    GPIO43	                IO���
//
//D5�ĺ�ɫ	OPWM4B/GPIO7	        IO���
//D5����ɫ	OPWM5A/GPIO8	        IO���
//D5����ɫ	OPWM5B/GPIO9	        IO���
//
//��������S100 IKEY/GPIO27	        IO���� ����ɫ��
//ɨ�谴��S101 ��TM1650����	        �����̡�IndependentProject_IIC_TM1650��
//ɨ�谴��S102 ��TM1650����	        �����̡�IndependentProject_IIC_TM1650��

//## ��ɫ5050��װLED��λ�ڰ������½ǣ���74HC125��5V����������ߵ�ƽ������
//## J1����ñΪ74HC125��ʹ�ܶ�,��ɫLED��J1�Ŀ���
//## J2����ñΪʹ����ɫ LED��J2λ����ɫLED���Ϸ���������·��GND
//##
-------------------------------------------------------------------------------------*/


//--------W25Q16���飨SPI������������Ҫ���е�Ӳ������--------------------------------------------
//�̽�J104���ϵ�����ñ���ṩDSP IIC���ߵ�SDA�źŵ�IIC����������TM1650��AT24C02
//�̽�J104���µ�����ñ���ṩDSP IO����W25Q16оƬ��CS����2
//�̽�J101���µ�����ñ���ṩDSP��SCIRX/GPIO28��CH340������

//--------W25Q16���飨SPI������������˵������Ҫ����--------------------------------------------



//���ڵ����������ã�128000 8N1


//###########################################################################
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "HK_all_include.h"

void InitLED(void);

Uint16 Error;
Uint16 Tmp = 0;
extern Uint16 RamfuncsLoadSize;


void main(void) 
{

// eCAN control registers require read/write access using 32-bits.  Thus we
// will create a set of shadow registers for this example.  These shadow
// registers will be used to make sure the access is 32-bits and not 16.
//   struct ECAN_REGS ECanaShadow;

// Step 1. Initialize System Control:b
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the DSP2803x_SysCtrl.c CAN_Node_IDfile.
   InitSysCtrl();

// Step 2. Initalize GPIO:
// This example function is found in the DSP2803x_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
// InitGpio();  // Skipped for this example


// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts
   DINT;

// Initialize PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.
// This function is found in the DSP2803x_PieCtrl.c file.
   InitPieCtrl();

// Disable CPU interrupts and clear all CPU interrupt flags:
   IER = 0x0000;
   IFR = 0x0000;

// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in DSP2803x_DefaultIsr.c.
// This function is found in DSP2803x_PieVect.c.
   InitPieVectTable();

   // Copy time critical code and Flash setup code to RAM
   // The  RamfuncsLoadStart, RamfuncsLoadSize, and RamfuncsRunStart
   // symbols are created by the linker. Refer to the F28035v1.cmd file.
   //ʵ����RAM�����д���İ��ˣ�����ʹ�ùٷ���DELAY_US
   memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (Uint32)&RamfuncsLoadSize);

   //------------------������ϵͳ��ʼ���õģ�һ�㲻��----------------------------

	InitLED();
	D401ON();
	D402LOW();
	Timer0_init();

  	SCI_Init();
	open_uart_debug();
	printf("\r\n\r\nDSP is Ready!");
	printf("\r\nԭ����Ʒ����--------Ӳ��SPI��������FLASHоƬW25Q16ʵ��-------\r\n");
	printf("\r\n\r\n������Ӳ�����ã�ע�⣺����ñ��ȷ����Ϊ��������!");
	printf("\r\nJ101Ϊ����USBת���ڣ�����ͨ��ʵ����Ҫ�̽Ӹ�λ������ñ");
	printf("\r\nJ104��������ñ�̽ӣ��ṩIIC������DSP����");
	printf("\r\nJ104����������ñ�̽ӣ��ṩSPI����W25Q16��CS��DSP����");

	printf("\r\n\r\n���ڵ������֣�HEX ���������FLASH���в���");
	printf("\r\nһ�η�������һ֡���ݣ�DSP�ڽ������һ���ֽ�5ms�����ͨ�Ű�����");
	printf("\r\nд�� ͨ�Ÿ�ʽ:  0x64 + 0x06 + 3byte FLASH��ַ + 3byte ���ݳ��� + 1byte ����");
	printf("\r\n���� ͨ�Ÿ�ʽ:  0x64 + 0x03 + 3byte FLASH��ַ + 3byte ���ݳ���");
	printf("\r\n��������flashоƬ ͨ�Ÿ�ʽ��0x64 + 0xFF");
	printf("\r\n��������� handleCommFlash() ����");
	printf("\r\nFLASH оƬ�в�д���������벻ҪƵ�����в��������� ����������Ӱ������");
	printf("\r\n16Mbit FLASH ��ַ���Ϊ0x1FFFFF");

	//TM1650 оƬ��ʼ��-------------------------------------
	softResetIIC_BUS();         //�����λIIC���豸
	InitI2C_Gpio();             //io ��ʼ��ΪIIC
	I2CA_Init();                //HW IIC��ʼ����100KHz
	//<TM1650.pdf>P4  ��ʾ��������
	LigntVal = 0x11;//BIT6��BIT4Ϊ���ȵ��ڣ�BIT0��  1 ����/0�ر�
	TM1650_Send(CMD_SEG,LigntVal);   //1�����ȣ�������ʾ
	TM1650_Send(DIG1,SEG7Table[0]);  //GID1
	TM1650_Send(DIG2,SEG7Table[1]);  //GID2
	TM1650_Send(DIG3,SEG7Table[2]);  //GID3
	TM1650_Send(DIG4,SEG7Table[3]);  //GID4

	Running.showNum = 2516;//��ʼ���������ʾ����

	//�ر�4λ�����ǿ����ʾ
	//��ʾ���ȼ�����showNum;��Χ 0 - 0x0F,�Լ�SEGHexBLK������ʾ����SEGHexBLK
	Running.showhex[0]=SEGHexBLK;
	Running.showhex[1]=SEGHexBLK;
	Running.showhex[2]=SEGHexBLK;
	Running.showhex[3]=SEGHexBLK;

	//������RGB ��ɫLED����˸״̬�� ��ɫ����������100ms�����ڲ���1000ms
	//��ʱ�����
	Running.Led595.ledState = BLEUON;
	Running.Led595.ledONZKB = 100;
	Running.Led595.ledTogglePre = 1000;


	//---------------------------SPI���߳�ʼ��------------------------------------
	SPI_IOinit();
	spi_fifo_init(); // Initialize the Spi FIFO
	spi_init();		 // init SPI


	//��W25Q16BV.pdf��
	//P17  W25Q16 ID����
	//P20  ID ָ�������ȡָ���ͨ�Ÿ�ʽ
	SPIFlash.Jedec_ID=Jedec_ID_Read();
	printf("\r\n\r\nflash ID is 0x%lx",SPIFlash.Jedec_ID);

	//��W25Q16BV.pdf�� P18
	SPIFlash.SReg = Read_Status_2Reg();
	printf("\r\nPowr ON the W25Q16 REG is %x",SPIFlash.SReg&0x03ff);

	//W25Q16оƬ��ʼ��
	WREN();
	WrSReg(0x0000);//�ϵ�Ҫ����״̬��ȡ��д��������д��
	SPIFlash.SReg = Read_Status_2Reg();
	printf("\r\nafter Write the W25Q16 REG is %x",SPIFlash.SReg&0x03ff);

	//�ж����ò���-----5
	PieCtrlRegs.PIECTRL.bit.ENPIE = 1;          // Enable the PIE block
	EINT;  // Enable Global interrupt INTM
	ERTM;

    while(1) 
    {
    	if(timer0Base.Mark_Para.Status_Bits.OnemsdFlag == 1)
    	{
    		timer0Base.Mark_Para.Status_Bits.OnemsdFlag = 0;

			if(timer0Base.msCounter >= 200)//ms
			{
				timer0Base.msCounter = 0;
			    D400TOGGLE();
			    D401TOGGLE();
			    D402TOGGLE();
			}
    	  }

    	/************************************************************************/
    	//���ڲ���SPI Flash W25Q16
    	//д�� ͨ�Ÿ�ʽ:  0x64 + 0x06 + 3byte FLASH��ַ + 3byte ���ݳ��� + 1byte ���ݣ���
    	//���� ͨ�Ÿ�ʽ:  0x64 + 0x03 + 3byte FLASH��ַ + 3byte ���ݳ���
    	/************************************************************************/
		if(SCI_Msg.timerOut >= RTU_TIMEROUT)
		{
		   SCI_Msg.Mark_Para.Status_Bits.rFifoDataflag = 0;
		   SCI_Msg.timerOut = 0;
		   SCI_Msg.Mark_Para.Status_Bits.DISRevflag = 1;

		   handleCommFlash();//FLASHоƬ�Ĳ�������

		   SCI_Msg.Mark_Para.Status_Bits.DISRevflag = 0;
		}

    	SegScanTask(&Running);  //����ܡ���ɫLED��������������------�ؼ�����
    	handleScanKey(&Running);//����ɨ�谴��
    }
}									

//#################################################
//��ʼ��LED
//D400�� GPIO41
//-----------------------------------------------
void InitLED()
{
	EALLOW;
	D400MUX = 0;
	D400DIR = 1;
	D401MUX = 0;
	D401DIR = 1;
	D402MUX = 0;
	D402DIR = 1;

	RGB_R_MUX = 0;
	RGB_R_DIR = 1;
	RGB_G_MUX = 0;
	RGB_G_DIR = 1;
	RGB_B_MUX = 0;
	RGB_B_DIR = 1;
	EDIS;
}

//===========================================================================
// No more.
//===========================================================================

 
