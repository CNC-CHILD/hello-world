#include <stdio.h>
#include <file.h>
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File  
#include "HK_all_include.h"


/*

	union
	  {
	    Uint16 All;
	    struct{
	            Uint16  UartRevFlag           :1;  //接收到数据标志
	            Uint16  HWOVFlag          :1;  //DSP硬件接收数据溢出标志

	            Uint16  rFifoDataflag            :1;  //接收内存非空
	            Uint16  rFifoFullflag            :1;  //接收内存溢出
	    		}Status_Bits;
	  	}Mark_Para;

	char rxData[UartRxLEN];						//接收缓存
	Uint16 rxReadIndex;                         //接收FIFO读出索引
	Uint16 rxWriteIndex;                        //接收FIFO写入索引

	Uint16 timerOut;                            //超时判断
}
*/

Uart_Msg SCI_Msg={0, {0},0,0, 0};


//#################################################
//串口接收缓存处理函数
//-----------------------------------------------
void handleRxFIFO()
{
	Uint16 wrIndex =0;

	if(SCI_Msg.Mark_Para.Status_Bits.rFifoDataflag == 1 )
	{
		SCI_Msg.Mark_Para.Status_Bits.rFifoDataflag =0;
		while(SCI_Msg.rxReadIndex != SCI_Msg.rxWriteIndex)
		{
			//这里处理接收到的数据
			wrIndex = (SCI_Msg.rxWriteIndex!=0)?(SCI_Msg.rxWriteIndex-1):(UartRxLEN-1); //获取写入FIFO指针，上传到PC
			printf("\r\nrxData[%d] = %c ; wr:%d",SCI_Msg.rxReadIndex,SCI_Msg.rxData[SCI_Msg.rxReadIndex],wrIndex);
		   //串口点亮LED
			switch(SCI_Msg.rxData[SCI_Msg.rxReadIndex])
			{
			   case 0x01:  D401TOGGLE();break;
			   case 0x02:  D402TOGGLE();break;
			   default:break;
			}
			SCI_Msg.rxReadIndex=(++SCI_Msg.rxReadIndex)%(UartRxLEN);
		}

	}

	//这里做缓存溢出处理
	if(SCI_Msg.Mark_Para.Status_Bits.rFifoFullflag ==1)
	{
		SCI_Msg.Mark_Para.Status_Bits.rFifoFullflag = 0;
		printf("\r\n---------------FIFO overflow-------------------------------------");

		SCI_Msg.rxReadIndex  = 0;
		SCI_Msg.rxWriteIndex  = 0;//SCI_Msg.rxReadIndex;
	}

	//这里做硬件串口接收溢出处理
	if(SCI_Msg.Mark_Para.Status_Bits.HWOVFlag == 1)
	{
		SCI_Msg.Mark_Para.Status_Bits.HWOVFlag = 0;
		printf("\r\n---------------SCI HW Overflow ISR Clear-------------------------");
	}

	//扫描硬件FIFO溢出处理
	if(SciaRegs.SCIFFRX.bit.RXFFOVF == 1)
	{
		SciaRegs.SCIFFRX.bit.RXFFOVRCLR=1;
		SciaRegs.SCIFFRX.bit.RXFIFORESET = 0;  //Write 0 to reset the FIFO pointer to zero, and hold in reset.
		SciaRegs.SCIFFRX.bit.RXFIFORESET = 1 ; //Re-enable receive FIFO operation
		printf("\r\n---------------Mannul HW Overflow  Clear------------------------");

	}
}

//#################################################
//串口接收中断函数
//采用FIFO机制（缓存）
//SCI_FIFO_LEN 定义为 1，最大为4
//-----------------------------------------------
interrupt void uartRx_isr(void)
{
	if(SciaRegs.SCIFFRX.bit.RXFFOVF == 0)//接收FIFO未溢出
	{
		SCI_Msg.Mark_Para.Status_Bits.rFifoDataflag = 1;

		if((SCI_Msg.rxWriteIndex + SCI_FIFO_LEN) != SCI_Msg.rxReadIndex )
		{
			//接收数据
			while(SciaRegs.SCIFFRX.bit.RXFFST)
			{
				SCI_Msg.rxData[SCI_Msg.rxWriteIndex] = SciaRegs.SCIRXBUF.all;
				SCI_Msg.rxWriteIndex=(++SCI_Msg.rxWriteIndex)%(UartRxLEN);
			}
		}
		else//接收缓存满
		{
			//用户这里做缓存满的处理,
			SciaRegs.SCIFFRX.bit.RXFIFORESET = 0;  //Write 0 to reset the FIFO pointer to zero, and hold in reset.
			SciaRegs.SCIFFRX.bit.RXFIFORESET = 1 ; //Re-enable receive FIFO operation

			SCI_Msg.Mark_Para.Status_Bits.rFifoFullflag = 1;
		}
	}
	else
	{
		//用户这里做串口硬件溢出的处理,可以完全读取出FIFO里的数据或者清空FIFO
		//这里清空FIFO操作
		SciaRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear HW Overflow flag
		SciaRegs.SCIFFRX.bit.RXFIFORESET = 0;  //Write 0 to reset the FIFO pointer to zero, and hold in reset.
		SciaRegs.SCIFFRX.bit.RXFIFORESET = 1 ; //Re-enable receive FIFO operation
		SCI_Msg.Mark_Para.Status_Bits.HWOVFlag = 1;

	}
    SciaRegs.SCIFFRX.bit.RXFFINTCLR=1;   // Clear Interrupt flag
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
}




//#################################################
//-----------------------------------------------
//串口初始化
//128000  8N1
// P21 框图介绍中断
//-----------------------------------------------
void SCI_Init()
{
	InitSciaGpio();
	SciaRegs.SCICTL1.bit.SWRESET = 0;

 	SciaRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback
                                   // No parity,8 char bits,
                                   // async mode, idle-line protocol

// baud = LSPCLK/8/((BRR+1) 
// baud @LSPCLK = 15MHz (60 MHz SYSCLK)
	SciaRegs.SCIHBAUD    =0x0000;
    SciaRegs.SCILBAUD    =14;      //0XC2-->9600 ; 97--> 19200 ;0x30-->38400;14-->128000

    SciaRegs.SCICTL1.bit.SWRESET = 1;     // Relinquish SCI from Reset
    SciaRegs.SCIFFTX.bit.SCIRST=1;

	SciaRegs.SCIFFRX.bit.RXFFIL  = SCI_FIFO_LEN;  //设置FIFO深度
	SciaRegs.SCICTL1.bit.TXENA = 1;       //使能发送
	SciaRegs.SCICTL1.bit.RXENA = 1;       //使能接收

//	SciaRegs.SCICTL2.bit.TXINTENA =1;
//	SciaRegs.SCICTL2.bit.RXBKINTENA =1;
//  SciaRegs.SCIFFTX.bit.TXFFIENA = 0; //禁止发送中断使能
	//中断配置步骤-----1
	SciaRegs.SCIFFTX.bit.SCIFFENA = 1; //使能FIFO中断
	SciaRegs.SCIFFRX.bit.RXFFIENA=1;
	EALLOW;
	PieVectTable.SCIRXINTA = &uartRx_isr; //中断配置步骤-----2
	EDIS;
	PieCtrlRegs.PIEIER9.bit.INTx1 = 1;    //中断配置步骤-----3
	IER |= M_INT9;						  //中断配置步骤-----4

	SciaRegs.SCIFFCT.all=0x00;

	SciaRegs.SCIFFTX.bit.TXFIFOXRESET=1;
	SciaRegs.SCIFFRX.bit.RXFIFORESET=1;
}


//#################################################
//-----------------------------------------------
//发送一个字节,超时机制,SYSCLK = 60MHz
//-----------------------------------------------
void scia_xmit(int a)
{
	Uint32 WaitTimer = 0;

	while (SciaRegs.SCIFFTX.bit.TXFFST != 0)
	//while(SciaRegs.SCICTL2.bit.TXEMPTY != 1)
	{
		WaitTimer++;
		if(WaitTimer > TIMEROUTSCI)break;
	}
	if(WaitTimer <= TIMEROUTSCI)
		SciaRegs.SCITXBUF=a;
}






//-------------------------------以下都为 标准 printf函数连接--------------------------------------------------------------
//#################################################
//-----------------------------------------------
//Printf 函数连接
//-----------------------------------------------
void open_uart_debug (void)
{
	int status;
	status = add_device("uart", _MSA, my_open, my_close, my_read, my_write, my_lseek, my_unlink, my_rename);
	if (status == 0) {
		freopen("uart:", "w", stdout);		// open uart and redirect stdout to UART
		setvbuf(stdout, NULL, _IONBF, 0); 	// disable buffering for stdout
	}
}

int my_open(const char *path, unsigned flags, int fno)
{
	//scia_fifo_init();
	//scia_echoback_init();
	path = path;
	flags = flags;
	fno = fno;
	return 0;
}

int my_close(int fno)
{
	fno =fno;
	return 0;
}

int my_read(int fno, char *buffer, unsigned count)
{
	fno = fno;
	buffer = buffer;
	count = count;
	return 0;
}

int my_write(int fno, const char *buffer, unsigned count)
{	
	int i=0;
	fno = fno;
	
	while(count-- > 0) {
		scia_xmit(buffer[i++]);
	}
	return count;
}

off_t my_lseek(int fno, off_t offset, int origin)
{
	fno = fno; offset = offset; origin = origin;
	return 0;
}

int my_unlink(const char *path)
{
	path = path;
	return 0;
}

int my_rename(const char *old_name, const char *new_name)
{
	old_name = old_name;
	new_name = new_name;
	return 0;
}

