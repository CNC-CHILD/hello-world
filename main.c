//###########################################################################
//--------M新动力----------原创例程------2015-04-04--------------------------------
//--------M新动力----------SPI总线操作实验----芯片W25Q16操作例程------------------

//       请参考TI官方手册《sprug71b--SPI.pdf》
//-------参考芯片手册《W25Q16BV.pdf》


/*--------------------基础配置介绍----常用LED、按键、5050 三色LED对应的IO口介绍-------------------------
//使用TI定义的DELAY_US()函数步骤：
//1. 工程添加 DSP2803x_usDelay.asm文件
//2. 声明extern Uint16 RamfuncsLoadSize，（RamfuncsRunStart和RamfuncsLoadStart在DSP2803x_GlobalPrototypes.h已声明）
//3. 使用memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (Uint32)&RamfuncsLoadSize);初始化在RAM运行代码
//4. 调用DELAY_US()即可，形参范围是 long doubl；单位为us （1秒为1000毫秒，1毫秒为1000微妙）

//## 根据原理图，硬件资源分配如下#
//外设	    IO口	                说明
//D400	    OPWM7B/LED_BK/GPIO41	IO输出
//D401	    LED1/GPIO34             IO输出
//D402	    GPIO43	                IO输出
//
//D5的红色	OPWM4B/GPIO7	        IO输出
//D5的绿色	OPWM5A/GPIO8	        IO输出
//D5的蓝色	OPWM5B/GPIO9	        IO输出
//
//独立按键S100 IKEY/GPIO27	        IO输入 （黑色）
//扫描按键S101 由TM1650驱动	        见例程《IndependentProject_IIC_TM1650》
//扫描按键S102 由TM1650驱动	        见例程《IndependentProject_IIC_TM1650》

//## 三色5050封装LED，位于板子左下角，由74HC125的5V输出驱动（高电平点亮）
//## J1跳线帽为74HC125的使能端,三色LED受J1的控制
//## J2跳线帽为使能三色 LED（J2位于三色LED的上方）电流回路到GND
//##
-------------------------------------------------------------------------------------*/


//--------W25Q16试验（SPI），开发板需要进行的硬件配置--------------------------------------------
//短接J104靠上的跳线帽，提供DSP IIC总线的SDA信号到IIC器件，包含TM1650和AT24C02
//短接J104靠下的跳线帽，提供DSP IO口与W25Q16芯片的CS连接2
//短接J101靠下的跳线帽，提供DSP的SCIRX/GPIO28与CH340的连接

//--------W25Q16试验（SPI），运行例程说明、重要函数--------------------------------------------



//串口调试助手设置：128000 8N1


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
   //实现在RAM中运行代码的搬运，例如使用官方的DELAY_US
   memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (Uint32)&RamfuncsLoadSize);

   //------------------以上是系统初始化用的，一般不动----------------------------

	InitLED();
	D401ON();
	D402LOW();
	Timer0_init();

  	SCI_Init();
	open_uart_debug();
	printf("\r\n\r\nDSP is Ready!");
	printf("\r\n原创精品例程--------硬件SPI操作外扩FLASH芯片W25Q16实验-------\r\n");
	printf("\r\n\r\n开发板硬件配置：注意：跳线帽正确配置为左右连接!");
	printf("\r\nJ101为板载USB转串口，串口通信实验需要短接该位置跳线帽");
	printf("\r\nJ104靠上跳线帽短接，提供IIC器件与DSP连接");
	printf("\r\nJ104靠下跳跳线帽短接，提供SPI器件W25Q16的CS与DSP连接");

	printf("\r\n\r\n串口调试助手，HEX 发送命令，对FLASH进行操作");
	printf("\r\n一次发送完整一帧数据，DSP在接收最后一个字节5ms后进行通信包解析");
	printf("\r\n写入 通信格式:  0x64 + 0x06 + 3byte FLASH地址 + 3byte 数据长度 + 1byte 数据");
	printf("\r\n读出 通信格式:  0x64 + 0x03 + 3byte FLASH地址 + 3byte 数据长度");
	printf("\r\n擦除整个flash芯片 通信格式：0x64 + 0xFF");
	printf("\r\n具体操作看 handleCommFlash() 函数");
	printf("\r\nFLASH 芯片有擦写的寿命，请不要频繁进行擦除操作； 读出操作不影响寿命");
	printf("\r\n16Mbit FLASH 地址最大为0x1FFFFF");

	//TM1650 芯片初始化-------------------------------------
	softResetIIC_BUS();         //软件复位IIC从设备
	InitI2C_Gpio();             //io 初始化为IIC
	I2CA_Init();                //HW IIC初始化，100KHz
	//<TM1650.pdf>P4  显示命令设置
	LigntVal = 0x11;//BIT6到BIT4为亮度调节，BIT0是  1 开启/0关闭
	TM1650_Send(CMD_SEG,LigntVal);   //1级亮度，开启显示
	TM1650_Send(DIG1,SEG7Table[0]);  //GID1
	TM1650_Send(DIG2,SEG7Table[1]);  //GID2
	TM1650_Send(DIG3,SEG7Table[2]);  //GID3
	TM1650_Send(DIG4,SEG7Table[3]);  //GID4

	Running.showNum = 2516;//初始化数码管显示内容

	//关闭4位数码管强制显示
	//显示优先级高于showNum;范围 0 - 0x0F,以及SEGHexBLK；不显示放置SEGHexBLK
	Running.showhex[0]=SEGHexBLK;
	Running.showhex[1]=SEGHexBLK;
	Running.showhex[2]=SEGHexBLK;
	Running.showhex[3]=SEGHexBLK;

	//定义了RGB 三色LED的闪烁状态： 蓝色、点亮参数100ms，周期参数1000ms
	//定时器里对
	Running.Led595.ledState = BLEUON;
	Running.Led595.ledONZKB = 100;
	Running.Led595.ledTogglePre = 1000;


	//---------------------------SPI总线初始化------------------------------------
	SPI_IOinit();
	spi_fifo_init(); // Initialize the Spi FIFO
	spi_init();		 // init SPI


	//《W25Q16BV.pdf》
	//P17  W25Q16 ID参数
	//P20  ID 指令表格与读取指令的通信格式
	SPIFlash.Jedec_ID=Jedec_ID_Read();
	printf("\r\n\r\nflash ID is 0x%lx",SPIFlash.Jedec_ID);

	//《W25Q16BV.pdf》 P18
	SPIFlash.SReg = Read_Status_2Reg();
	printf("\r\nPowr ON the W25Q16 REG is %x",SPIFlash.SReg&0x03ff);

	//W25Q16芯片初始化
	WREN();
	WrSReg(0x0000);//上点要设置状态，取消写保护才能写入
	SPIFlash.SReg = Read_Status_2Reg();
	printf("\r\nafter Write the W25Q16 REG is %x",SPIFlash.SReg&0x03ff);

	//中断配置步骤-----5
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
    	//串口操作SPI Flash W25Q16
    	//写入 通信格式:  0x64 + 0x06 + 3byte FLASH地址 + 3byte 数据长度 + 1byte 数据（）
    	//读出 通信格式:  0x64 + 0x03 + 3byte FLASH地址 + 3byte 数据长度
    	/************************************************************************/
		if(SCI_Msg.timerOut >= RTU_TIMEROUT)
		{
		   SCI_Msg.Mark_Para.Status_Bits.rFifoDataflag = 0;
		   SCI_Msg.timerOut = 0;
		   SCI_Msg.Mark_Para.Status_Bits.DISRevflag = 1;

		   handleCommFlash();//FLASH芯片的操作函数

		   SCI_Msg.Mark_Para.Status_Bits.DISRevflag = 0;
		}

    	SegScanTask(&Running);  //数码管、三色LED、按键操作函数------关键函数
    	handleScanKey(&Running);//处理扫描按键
    }
}									

//#################################################
//初始化LED
//D400用 GPIO41
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

 
