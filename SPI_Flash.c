/*
 * SPI_Flash.c
 *
 *  Created on: 2013-12-18
 *      Author: cotto
 */
#include "HK_all_include.h"


uint8_t upper_128[128]={0};

/*
typedef struct W25Q64Flash_Type{
	long Jedec_ID;
	uint16_t SReg;

	uint32_t flashAddr;
	uint8_t  SciFlashCMD;
	uint16_t flashLen;
}
W25Q64Flash;*/

W25Q64Flash SPIFlash={0,0,  0,0,0};

void SPI_IOinit()
{
EALLOW;
	EECSMUX = 0;
	EECSDIR = 1;

	GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0;   // Enable pull-up on GPIO16 (SPISIMOA)
	GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;   // Enable pull-up on GPIO17 (SPISOMIA)
	GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;   // Enable pull-up on GPIO18 (SPICLKA)
	GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;   // Enable pull-up on GPIO19 (SPISTEA)

	GpioCtrlRegs.GPAQSEL2.bit.GPIO16 = 3; // Asynch input GPIO16 (SPISIMOA)
	GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 3; // Asynch input GPIO17 (SPISOMIA)
	GpioCtrlRegs.GPAQSEL2.bit.GPIO18 = 3; // Asynch input GPIO18 (SPICLKA)
	GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = 3; // Asynch input GPIO19 (SPISTEA)

	GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 1; // Configure GPIO16 as SPISIMOA
	GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 1; // Configure GPIO17 as SPISOMIA
	GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 1; // Configure GPIO18 as SPICLKA
	GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 1; // Configure GPIO19 as SPISTEA

EDIS;
}

//CLOCK PHASE = 0:
//CLOCK POLARITY  = 0

void spi_init()
{
	CE_High();

	SpiaRegs.SPICCR.bit.SPISWRESET = 0; //P25  SW Reset on
	SpiaRegs.SPICCR.bit.SPICHAR = 7;    //p26   8bit
	SpiaRegs.SPICCR.bit.CLKPOLARITY = 0 ; //P25 When no SPI data is sent, SPICLK is at low
	SpiaRegs.SPICTL.bit.CLK_PHASE = 1;   //P25   Data is output on the rising edge of the SPICLK signal;
	                                     //      input data is latched on the falling edge of the SPICLK signal.
	SpiaRegs.SPICTL.bit.MASTER_SLAVE  =1;//P26   SPI configured as a master.

	SpiaRegs.SPIBRR =3;             // For SPIBRR = 3 to 127;  SPI Baud Rate = LSPCLK/(SPIBRR+1)

	SpiaRegs.SPICCR.bit.SPISWRESET = 1; //P25  SW Reset off

    SpiaRegs.SPIPRI.bit.FREE = 1;   // Set so breakpoints don't disturb xmission
    SpiaRegs.SPIPRI.bit.STEINV = 0; //P34 SPISTE is active low (normal)
}

uint8_t Send_Byte(uint16_t a)
{
	Uint16 rdata;

	SpiaRegs.SPICTL.bit.TALK  =1;                //P23 Enable Transmit path
	SpiaRegs.SPITXBUF=(a<<8)&0xff00;
    while(SpiaRegs.SPIFFRX.bit.RXFFST == 0) { } 	// Wait until data is received
//	while(SpiaRegs.SPISTS.bit.INT_FLAG == 0) {} // Waits until data rx’d
    rdata = SpiaRegs.SPIRXBUF;
    return rdata;
}

uint16_t Get_Byte(void)
{
	uint8_t rdata;
	SpiaRegs.SPICTL.bit.TALK = 0; // Disable Transmit pat
	SpiaRegs.SPITXBUF = DUMMYDATA; // Send dummy to start
	// NOTE: because TALK = 0, data does not tx onto SPISIMOA pin
	//while(SpiaRegs.SPISTS.bit.INT_FLAG !=1) {} // Wait until data rece
	while(SpiaRegs.SPIFFRX.bit.RXFFST ==0) { } 	// Wait until data is received
	rdata = SpiaRegs.SPIRXBUF; // Master reads data
	return  rdata;
}

void spi_fifo_init()
{
// Initialize SPI FIFO registers
//    SpiaRegs.SPIFFTX.all=0xE040;
	 SpiaRegs.SPIFFTX.bit.SPIRST =1;  //P31 SPI FIFO can resume transmit or receive.
	 SpiaRegs.SPIFFTX.bit.SPIFFENA =1;//SPI FIFO enhancements are enabled
	 SpiaRegs.SPIFFTX.bit.TXFFST = 1; //Re-enable Transmit FIFO operation
	 SpiaRegs.SPIFFTX.bit.TXFFINTCLR = 1;//Write 1 to clear TXFFINT flag in bit 7.
	 SpiaRegs.SPIFFTX.bit.TXFFIENA = 0; //不用中断


    //SpiaRegs.SPIFFRX.all=0x2044;
	 SpiaRegs.SPIFFRX.bit.RXFFOVFCLR = 1;
	 SpiaRegs.SPIFFRX.bit.RXFFINTCLR  =1;
	 SpiaRegs.SPIFFRX.bit.RXFFST = 1;
	 SpiaRegs.SPIFFRX.bit.RXFFIENA = 0; //不用FIFO接收中断
	 SpiaRegs.SPIFFRX.bit.RXFFIL = 4;

    SpiaRegs.SPIFFCT.all=0x0;  //P32
}


//《W25Q16BV.pdf》
//P17  W25Q16 ID参数
//P20  ID 指令表格与读取指令的通信格式
unsigned long Jedec_ID_Read(void)
{
	unsigned long temp1,temp2,temp3;
	temp1 = 0;
	temp2 = 0;
	temp3 = 0;

	CE_Low();			 /* enable device */
	Send_Byte(0x9F);		 /* send JEDEC ID command (9Fh) */
	//spi_xmit(0);
	//spi_xmit(0);
	//spi_xmit(0);
	temp1=Get_Byte();
	temp2=Get_Byte();
	temp3=Get_Byte();
	CE_High();			 /* disable device */

	temp1=((temp1<<16)|(temp2<<8)|(temp3))&0x00FFFFFF;

	return temp1;
}


/************************************************************************/
//P15/P22
//写入状态寄存器
//先写入低8位，后写入高2位
/************************************************************************/
void WrSReg(uint16_t setReg)
{
	CE_Low();
	Send_Byte(WRITE_SREG_IST);
	Send_Byte((setReg&0x00ff));
	Send_Byte((setReg&0x0300)>>8);
	CE_High();
}

/************************************************************************/
//P33
//写入A23--A0
//指令 0x02
//1到256 byte写入
//最后一个byte要写入 0
/************************************************************************/
void PageProgram(unsigned long Dst, unsigned char *byte,unsigned char len)
{
	unsigned char i;
	CE_Low();					/* enable device */
	Send_Byte(PAGE_PRG_IST); 			/* send Byte Program command */
	Send_Byte(((Dst & 0xFFFFFF) >> 16));/* send 3 address bytes */
	Send_Byte(((Dst & 0xFFFF) >> 8));
	Send_Byte(Dst & 0xFF);
	for(i=0;i<len;i++)
	{
		//判断忙标志，再进行写操作
		//printf(" %x",(*byte)&0xff);
		Send_Byte(*(byte++));
	}
	CE_High();				/* disable device */
}


/************************************************************************/
//P23
//指令 0x03
//地址 A23--A0
/************************************************************************/
void ReadData(unsigned long Dst, unsigned char *Rxbuf,unsigned long len)
{
	unsigned long i = 0;
	CE_Low();				/* enable device */
	Send_Byte(READ_DATA_IST); 			/* read command */
	Send_Byte(((Dst & 0xFFFFFF) >> 16)); 	/* send 3 address bytes */
	Send_Byte(((Dst & 0xFFFF) >> 8));
	Send_Byte(Dst & 0xFF);
	for (i = 0; i < len; i++)		/* read until no_bytes is reached */
	{
		*(Rxbuf++) = Get_Byte();	/* receive byte and store at address 80H - FFH */
		printf(" 0x%x",(*(Rxbuf-1))&0xff);//调试预留函数，可取消该句
	}
	CE_High();				/* disable device */
}


/************************************************************************/
//P23
//指令 0x03
//地址 A23--A0
/************************************************************************/
uint16_t Read_Status_2Reg(void)
{
	uint16_t wbyte1,wbyte2;

	CE_Low();
	Send_Byte(READ_STATUS_REG_IST);
	wbyte1 = Get_Byte();
	CE_High();

	CE_Low();
	Send_Byte(0x35);		//READ_STATUS_REG_IST
	wbyte2 = Get_Byte();
	CE_High();

	wbyte2<<=6;
	wbyte2 &=0xff00;
	wbyte2 += wbyte1&0xff;
	CE_High();
	return wbyte2;
}


/************************************************************************/
//FLASH写入操作，只能从逻辑1写为逻辑1或者逻辑0； 擦除操作，以块为单位进行擦除，擦除后的块内容全为0xFF
//串口操作SPI Flash W25Q16
//写入 通信格式: 0x64 + 0x06 + 3byte FLASH地址 + 3byte 数据长度 + 1byte 数据（）
//读出 通信格式: 0x64 + 0x03 + 3byte FLASH地址 + 3byte 数据长度
//擦除整个芯片 : 0X64 + FF
/************************************************************************/
uint8_t handleCommFlash()
{
	uint8_t i;
	uint32_t readBufIndex=0;

	if(SCI_Msg.rxData[FLASH_ID_I] != SPIFLASH_ID)
	{
		SCI_Msg.rxWriteIndex = 0;
		SCI_Msg.rxReadIndex = 0;
		return 1;
	}

	SPIFlash.CMD = SCI_Msg.rxData[FLASH_CMD_I];
	//这里处理接收到的数据
	//wrIndex = (SCI_Msg.rxWriteIndex!=0)?(SCI_Msg.rxWriteIndex-1):(UartRxLEN-1); //获取写入FIFO指针，上传到PC
	//printf("\r\nrxData[%d] = %c ; wr:%d",SCI_Msg.rxReadIndex,SCI_Msg.rxData[SCI_Msg.rxReadIndex],wrIndex);
	//SCI_Msg.rxReadIndex=(++SCI_Msg.rxReadIndex)%(UartRxLEN);

	//获取地址
	SPIFlash.Addr =  SCI_Msg.rxData[FLASH_ADDRH_I]; SPIFlash.Addr<<=8;
	SPIFlash.Addr += SCI_Msg.rxData[FLASH_ADDRM_I]; SPIFlash.Addr<<=8;
	SPIFlash.Addr += SCI_Msg.rxData[FLASH_ADDRL_I];
	//这里可增加对地址范围的判断


	//获取写入长度
	SPIFlash.Len  =   SCI_Msg.rxData[FLASH_DATALENH_I]; SPIFlash.Len<<=8;
	SPIFlash.Len+= SCI_Msg.rxData[FLASH_DATALENM_I];SPIFlash.Len<<=8;
	SPIFlash.Len+= SCI_Msg.rxData[FLASH_DATALENL_I];
	//这里可增加对数据长度做出范围判断

	//操作判断
	  switch(SPIFlash.CMD)
	  {
	  	  case 3: //读出
				printf("\r\n\r\n读出起始地址：0x%0.8lx  长度0x%0.8lx ",SPIFlash.Addr,SPIFlash.Len);
				if(SPIFlash.Len>128)
				{
					SPIFlash.Len=128;
					printf("\r\n本例程一次读出数据长度限制为128字节");
				}
				else if(SPIFlash.Len<=16)
					ReadData(SPIFlash.Addr,upper_128,SPIFlash.Len);
				else
				{
					for(;SPIFlash.Len>16;)//这个upper_128数组只有128个长度，注意不要溢出了
					{
						printf("\r\n地址：0x%0.8lx",SPIFlash.Addr);
						ReadData(SPIFlash.Addr,upper_128,16);
						SPIFlash.Len-=16;
						SPIFlash.Addr+=16;
						readBufIndex+=16;
					}
					ReadData(SPIFlash.Addr,&upper_128[readBufIndex],SPIFlash.Len);
				}

			break;

	  	  case 6: //写入

				printf("\r\n\r\n写入起始地址：0x%0.8lx ；长度 0x%0.8lx ；重复写入的数据 0x%x；对FLASH操作，只能由1写为1或0，不能由0写为1，擦除则全为1",SPIFlash.Addr,SPIFlash.Len,SCI_Msg.rxData[FLASH_CY_DATA_I]);

				if(SPIFlash.Len>128)
				{
					SPIFlash.Len = 128;
					printf("\r\n本例程一次写入数据长度限制为128字节");
				}

				printf("\r\n写入前数据为 ");
	  		    ReadData(SPIFlash.Addr,upper_128,SPIFlash.Len);

				for(i=0;i<SPIFlash.Len;i++)
				{
					upper_128[i] = SCI_Msg.rxData[FLASH_CY_DATA_I];
				}
				WREN();
				PageProgram(SPIFlash.Addr,upper_128,SPIFlash.Len);
				Wait_Busy();

			    printf("\r\n写入后数据为 ");
				ReadData(SPIFlash.Addr,upper_128,SPIFlash.Len);

			break;

	  	  case 0xff: //擦除整个芯片
	  		  	printf("\r\n\r\n请不要发频繁擦除芯片；正开始擦除整个芯片.....稍等片刻");
	  		  	WREN();
				Chip_Erase();
				Wait_Busy();
				printf("\r\n芯片擦除完毕");

			break;
	  	  default:break;
	  }

	//操作完毕，复位变量
	SPIFlash.CMD = 0;
	SPIFlash.Addr = 0;
	SPIFlash.Len = 0;

	SCI_Msg.rxWriteIndex = 0;
	SCI_Msg.rxReadIndex = 0;

	return 0;

}


/************************************************************************/
//spi Flash操作
/************************************************************************/
void FlashOperate()
{

}
















/************************************************************************/
//P35
//指令 0x20
//地址 A23--A0
/************************************************************************/
void Sector4K_Erase(unsigned long Dst)
{
	Dst = Dst/0x1000;
	CE_Low();
	Send_Byte(SECTOR_ERASE_IST);
	Send_Byte(((Dst & 0xFFFFFF) >> 16));
	Send_Byte(((Dst & 0xFFFF) >> 8));
	Send_Byte(Dst & 0xFF);
	CE_High();
}



/************************************************************************/
/* PROCEDURE: Poll_SO							*/
/*									*/
/* This procedure polls for the SO line during AAI programming  	*/
/* waiting for SO to transition to 1 which will indicate AAI programming*/
/* is completed								*/
/*									*/
/* Input:								*/
/*		SO							*/
/*									*/
/* Output:								*/
/*		None							*/
/************************************************************************/
void Poll_SO(void)
{
	unsigned char temp = 0;
	CE_Low();
	temp = F032B_SO_READ();
       	while (temp == 0x00)	/* waste time until not busy */
		{
			temp = F032B_SO_READ();
		}

	CE_High();
}

/************************************************************************/
/* PROCEDURE: Read_Status_Register					*/
/*									*/
/* This procedure read the status register and returns the byte.	*/
/*									*/
/* Input:								*/
/*		None							*/
/*									*/
/* Returns:								*/
/*		byte							*/
/************************************************************************/
unsigned char Read_Status_Register(void)
{
	unsigned char byte = 0;
	CE_Low();			/* enable device */
	Send_Byte(READ_STATUS_REG_IST);		/* send RDSR command */
	byte = Get_Byte();		/* receive byte */
	CE_High();			/* disable device */
	return byte;
}


/************************************************************************/
/* PROCEDURE: EWSR							*/
/*									*/
/* This procedure Enables Write Status Register.  			*/
/*									*/
/* Input:								*/
/*		None							*/
/*									*/
/* Returns:								*/
/*		Nothing							*/
/************************************************************************/
void EWSR(void)
{
	CE_Low();			/* enable device */
	Send_Byte(WRITE_SREG_IST);	/* enable writing to the status register */
	CE_High();			/* disable device */
}

/************************************************************************/
/* PROCEDURE: WRSR							*/
/*									*/
/* This procedure writes a byte to the Status Register.			*/
/*									*/
/* Input:								*/
/*		byte							*/
/*									*/
/* Returns:								*/
/*		Nothing							*/
/************************************************************************/
void WRSR(char unsigned byte)
{
	CE_Low();			/* enable device */
	Send_Byte(0x01);		/* select write to status register */
	Send_Byte(byte);		/* data that will change the status of BPx
					       or BPL (only bits 2,3,4,5,7 can be written) */
	CE_High();			/* disable the device */
}


/************************************************************************/
/* PROCEDURE: WREN							*/
/*									*/
/* This procedure enables the Write Enable Latch.  It can also be used 	*/
/* to Enables Write Status Register.					*/
/*									*/
/* Input:								*/
/*		None							*/
/*									*/
/* Returns:								*/
/*		Nothing							*/
/************************************************************************/
void WREN(void)
{
	CE_Low();			/* enable device */
	Send_Byte(WREN_IST);		/* send WREN command */
	CE_High();			/* disable device */
}

/************************************************************************/
/* PROCEDURE: WRDI							*/
/*									*/
/* This procedure disables the Write Enable Latch.			*/
/*									*/
/* Input:								*/
/*		None							*/
/*									*/
/* Returns:								*/
/*		Nothing							*/
/************************************************************************/
void WRDI(void)
{
	CE_Low();			/* enable device */
	Send_Byte(0x04);		/* send WRDI command */
	CE_High();			/* disable device */
}

/************************************************************************/
/* PROCEDURE: EBSY							*/
/*									*/
/* This procedure enable SO to output RY/BY# status during AAI 		*/
/* programming.								*/
/*									*/
/* Input:								*/
/*		None							*/
/*									*/
/* Returns:								*/
/*		Nothing							*/
/************************************************************************/
void EBSY(void)
{
	CE_Low();			/* enable device */
	Send_Byte(0x70);    /* send EBSY command */
	CE_High();			/* disable device */
}


/************************************************************************/
/* PROCEDURE: DBSY							*/
/*									*/
/* This procedure disable SO as output RY/BY# status signal during AAI	*/
/* programming.								*/
/*									*/
/* Input:								*/
/*		None							*/
/*									*/
/* Returns:								*/
/*		Nothing							*/
/************************************************************************/
void DBSY(void)
{
	CE_Low();			/* enable device */
	Send_Byte(0x80);		/* send DBSY command */
	CE_High();			/* disable device */
}

/************************************************************************/
/* PROCEDURE: Read_ID							*/
/*									*/
/* This procedure Reads the manufacturer's ID and device ID.  It will 	*/
/* use 90h or ABh as the command to read the ID (90h in this sample).   */
/* It is up to the user to give the last byte ID_addr to determine      */
/* whether the device outputs manufacturer's ID first, or device ID 	*/
/* first.  Please see the product datasheet for details.  Returns ID in */
/* variable byte.							*/
/*									*/
/* Input:								*/
/*		ID_addr							*/
/*									*/
/* Returns:								*/
/*		byte:	ID1(Manufacture's ID = BFh or Device ID = 4Ah)	*/
/*									*/
/************************************************************************/
unsigned char Read_ID(unsigned char ID_addr)
{
	unsigned char byte;
	CE_Low();			/* enable device */
	Send_Byte(0x90);		/* send read ID command (90h or ABh) */
   Send_Byte(0x00);		/* send address */
	Send_Byte(0x00);		/* send address */
	Send_Byte(ID_addr);		/* send address - either 00H or 01H */
	byte = Get_Byte();		/* receive byte */
	CE_High();			/* disable device */
	return byte;
}


/************************************************************************/
/* PROCEDURE:	Read							*/
/*									*/
/* This procedure reads one address of the device.  It will return the 	*/
/* byte read in variable byte.						*/
/*									*/
/*									*/
/*									*/
/* Input:								*/
/*		Dst:	Destination Address 000000H - 3FFFFFH		*/
/*      								*/
/*									*/
/* Returns:								*/
/*		byte							*/
/*									*/
/************************************************************************/
unsigned char Read(unsigned long Dst)
{
	unsigned char byte = 0;

	CE_Low();			/* enable device */
	Send_Byte(0x03); 		/* read command */
	Send_Byte(((Dst & 0xFFFFFF) >> 16));	/* send 3 address bytes */
	Send_Byte(((Dst & 0xFFFF) >> 8));
	Send_Byte(Dst & 0xFF);
	byte = Get_Byte();
	CE_High();			/* disable device */
	return byte;			/* return one byte read */
}


/************************************************************************/
/* PROCEDURE:	Read_Cont						*/
/*									*/
/* This procedure reads multiple addresses of the device and stores	*/
/* data into 128 byte buffer. Maximum byte that can be read is 128 bytes*/
/*									*/
/* Input:								*/
/*		Dst:		Destination Address 000000H - 3FFFFFH	*/
/*      	no_bytes	Number of bytes to read	(max = 128)	*/
/*									*/
/* Returns:								*/
/*		Nothing							*/
/*									*/
/************************************************************************/
void Read_Cont(unsigned long Dst, unsigned long no_bytes)
{
	unsigned long i = 0;
	CE_Low();				/* enable device */
	Send_Byte(0x03); 			/* read command */
	Send_Byte(((Dst & 0xFFFFFF) >> 16)); 	/* send 3 address bytes */
	Send_Byte(((Dst & 0xFFFF) >> 8));
	Send_Byte(Dst & 0xFF);
	for (i = 0; i < no_bytes; i++)		/* read until no_bytes is reached */
	{
		upper_128[i] = Get_Byte();	/* receive byte and store at address 80H - FFH */
	}
	CE_High();				/* disable device */

}



/************************************************************************/
/* PROCEDURE:	HighSpeed_Read						*/
/*									*/
/* This procedure reads one address of the device.  It will return the 	*/
/* byte read in variable byte.						*/
/*									*/
/*									*/
/*									*/
/* Input:								*/
/*		Dst:	Destination Address 000000H - 3FFFFFH		*/
/*      								*/
/*									*/
/* Returns:								*/
/*		byte							*/
/*									*/
/************************************************************************/
unsigned char HighSpeed_Read(unsigned long Dst)
{
	unsigned char byte = 0;

	CE_Low();			/* enable device */
	Send_Byte(0x0B); 		/* read command */
	Send_Byte(((Dst & 0xFFFFFF) >> 16));	/* send 3 address bytes */
	Send_Byte(((Dst & 0xFFFF) >> 8));
	Send_Byte(Dst & 0xFF);
	Send_Byte(0xFF);		/*dummy byte*/
	byte = Get_Byte();
	CE_High();			/* disable device */
	return byte;			/* return one byte read */
}

/************************************************************************/
/* PROCEDURE:	HighSpeed_Read_Cont					*/
/*									*/
/* This procedure reads multiple addresses of the device and stores	*/
/* data into 128 byte buffer. Maximum byte that can be read is 128 bytes*/
/*									*/
/* Input:								*/
/*		Dst:		Destination Address 000000H - 3FFFFFH	*/
/*      	no_bytes	Number of bytes to read	(max = 128)	*/
/*									*/
/* Returns:								*/
/*		Nothing							*/
/*									*/
/************************************************************************/
void HighSpeed_Read_Cont(unsigned long Dst, unsigned long no_bytes)
{
	unsigned long i = 0;
	CE_Low();				/* enable device */
	Send_Byte(0x0B); 			/* read command */
	Send_Byte(((Dst & 0xFFFFFF) >> 16)); 	/* send 3 address bytes */
	Send_Byte(((Dst & 0xFFFF) >> 8));
	Send_Byte(Dst & 0xFF);
	Send_Byte(0xFF);			/*dummy byte*/
	for (i = 0; i < no_bytes; i++)		/* read until no_bytes is reached */
	{
		upper_128[i] = Get_Byte();	/* receive byte and store at address 80H - FFH */
	}
	CE_High();				/* disable device */
}


/************************************************************************/
//P33
//写入A23--A0
//指令 0x02
//1到256 byte写入
//最后一个byte要写入 0
/************************************************************************/
void Byte_Program(unsigned long Dst, unsigned char byte)
{
	CE_Low();					/* enable device */
	Send_Byte(PAGE_PRG_IST); 			/* send Byte Program command */
	Send_Byte(((Dst & 0xFFFFFF) >> 16));/* send 3 address bytes */
	Send_Byte(((Dst & 0xFFFF) >> 8));
	Send_Byte(Dst & 0xFF);
	Send_Byte(byte);			/* send byte to be programmed */
	CE_High();				/* disable device */
}



/************************************************************************/
/* PROCEDURE: Chip_Erase						*/
/*									*/
/* This procedure erases the entire Chip.				*/
/*									*/
/* Input:								*/
/*		None							*/
/*									*/
/* Returns:								*/
/*		Nothing							*/
/************************************************************************/
void Chip_Erase(void)
{
	CE_Low();				/* enable device */
	Send_Byte(CHIP_ERASE_IST);			/* send Chip Erase command (60h or C7h) */
	CE_High();				/* disable device */
}

/************************************************************************/
/* PROCEDURE: Sector_Erase						*/
/*									*/
/* This procedure Sector Erases the Chip.				*/
/*									*/
/* Input:								*/
/*		Dst:		Destination Address 000000H - 3FFFFFH	*/
/*									*/
/* Returns:								*/
/*		Nothing							*/
/************************************************************************/
void Sector_Erase(unsigned long Dst)
{


	CE_Low();				/* enable device */
	Send_Byte(0x20);			/* send Sector Erase command */
	Send_Byte(((Dst & 0xFFFFFF) >> 16)); 	/* send 3 address bytes */
	Send_Byte(((Dst & 0xFFFF) >> 8));
	Send_Byte(Dst & 0xFF);
	CE_High();				/* disable device */
}


/************************************************************************/
/* PROCEDURE: Block_Erase_32K						*/
/*									*/
/* This procedure Block Erases 32 KByte of the Chip.			*/
/*									*/
/* Input:								*/
/*		Dst:		Destination Address 000000H - 3FFFFFH	*/
/*									*/
/* Returns:								*/
/*		Nothing							*/
/************************************************************************/
void Block_Erase_32K(unsigned long Dst)
{
	CE_Low();				/* enable device */
	Send_Byte(0x52);			/* send 32 KByte Block Erase command */
	Send_Byte(((Dst & 0xFFFFFF) >> 16)); 	/* send 3 address bytes */
	Send_Byte(((Dst & 0xFFFF) >> 8));
	Send_Byte(Dst & 0xFF);
	CE_High();				/* disable device */
}

/************************************************************************/
/* PROCEDURE: Block_Erase_64K						*/
/*									*/
/* This procedure Block Erases 64 KByte of the Chip.			*/
/*									*/
/* Input:								*/
/*		Dst:		Destination Address 000000H - 3FFFFFH	*/
/*									*/
/* Returns:								*/
/*		Nothing							*/
/************************************************************************/
void Block_Erase_64K(unsigned long Dst)
{
	CE_Low();				/* enable device */
	Send_Byte(0xD8);			/* send 64KByte Block Erase command */
	Send_Byte(((Dst & 0xFFFFFF) >> 16)); 	/* send 3 address bytes */
	Send_Byte(((Dst & 0xFFFF) >> 8));
	Send_Byte(Dst & 0xFF);
	CE_High();				/* disable device */
}

/************************************************************************/
/* PROCEDURE: Wait_Busy							*/
/*									*/
/* This procedure waits until device is no longer busy (can be used by	*/
/* Byte-Program, Sector-Erase, Block-Erase, Chip-Erase).		*/
/*									*/
/* Input:								*/
/*		None							*/
/*									*/
/* Returns:								*/
/*		Nothing							*/
/************************************************************************/
void Wait_Busy(void)
{
	uint32_t waitCnt=0;
	unsigned char Wbusy;
	waitCnt = 600000;
	Wbusy =Read_Status_Register(); //HK P15  Status_Register目录
	while ((Wbusy & 0x01) == 0x01)	/* waste time until not busy */
	{
		Wbusy =Read_Status_Register();

		if(waitCnt==0)
		{
			WREN();
			WrSReg(0x0000);//上点要设置状态，取消写保护才能写入
			printf("\r\n-----!!!!!!!!!!!!!!!!-----------");
			SPIFlash.SReg = Read_Status_2Reg();
			printf("Wait Flash busy,and reWrite REG is %x",SPIFlash.SReg&0x03ff);
			break;
		}

		else
			waitCnt--;
	}
}


/************************************************************************/
/* PROCEDURE: WREN_Check						*/
/*									*/
/* This procedure checks to see if WEL bit set before program/erase.	*/
/*									*/
/* Input:								*/
/*		None							*/
/*									*/
/* Returns:								*/
/*		Nothing							*/
/************************************************************************/
void WREN_Check(void)
{
	unsigned char byte;
	byte = Read_Status_Register();	/* read the status register */
	if (byte != 0x02)		/* verify that WEL bit is set */
	{
		while(1)
		{}
			/* add source code or statements for this file */
			/* to compile   			       */
			/* i.e. option: insert a display to view error on LED? */
		 	/* option: insert a display to view error on LED? */
	}
}


