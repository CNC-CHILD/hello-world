#include "HK_all_include.h"
#include <math.h>

char poitPosition=0;//存放小数点的位置
float ftestData = 0.01;//数码管内容的变化量
char keyValCode[]={S100_CODE,S101_CODE,S102_CODE};

/*
  uint16_t scanTick;    //扫描时间
  SegCS    sltCSn;      //选择的显示位置
  uint8_t  showRam[4];  //数码管显示的内容,最低层显示的内容，  [0] -->SEG4
  float    showNum;     //数码管显示的数字, 可以是浮点数也可以是整数，这里的最大范围 0.999 到 9999
  char     showhex[4];  // 显示优先级高于showNum;范围 0 --- 0x0F,以及SEGHexBLK；不显示放置 SEGHexBLK
  Led      Led595;           //LED 状态结构体
  keySlt   nowScanKey;  //当前发送的按键
  KeyScan  skey[KeyNUM];     //S100 S101按键扫描；S102非595输出扫描按键

  Led_Clour ledState;      //  LED显示内容
  uint16_t   ledONZKB;     //  LED亮的占空比时间； 单位 1ms
  uint16_t   ledTogglePre; //  LED闪烁频率； 单位 1ms
  uint16_t   ledTick;      //  LED计数

  uint16_t  pressDEF;  //  需要按下识别的次数
  uint16_t  pressTimer;//  按下次数时基计数
  uint16_t  ClickCNT;  //  已经按下点击的次数
*/
//运行内容初始化
Scan Running={0,SEGCS1,{0,0,0,0},-1.99,{SEGHexBLK,SEGHexBLK,SEGHexBLK,SEGHexBLK},{BLEUON,100,1000,0},S100,{{5,0,0},{5,0,0},{5,0,0}}};


//-----------------RGB LED----------------------
#ifdef RGB_HIGE
//GPIO7=RED  GPIO8=GRE GPIO9=BLUE
Uint32 RGB_Code[]={0x0380,0x0080,0x0100,0x0200,0x0380};//高电平点亮
#else
//GPIO7=RED GPIO9=GRE  GPIO8=BLUE
Uint32 RGB_Code[]={0x0380,0x0080,0x0200,0x0100,0x0380};  //低电平点亮
#endif

//595输出到按键的编码
//因为输出高有效，ShumaSlt()函数发的是反码，所以这里定义要取反
char *keyStr[]={"S100","S101","S102"};
//-----------------------------------------------
//
////#################################################
////-----------------------------------------------
//// ·￠?íêy??1ü??ê?oˉêy
//// ′ó×óμ?óòêy￡?d1  d2
////  ??′?ê?3?
////-----------------------------------------------
//void Shuma(uint8_t d1,uint8_t d2)
//{
//    Send_One_595(SEG7Table[d2]);
//    Send_One_595(SEG7Table[d1]);
//    LT595_H();
//    LT595_L();
//}



//#################################################
//-----------------------------------------------
// 发送某一位数码管

// 
//----------------------------------------------- 
char TM1650SegVal[]={DIG1,DIG2,DIG3,DIG4};
void ShumaSlt(uint8_t data,SegCS csn,Led_Clour led, keySlt KEY)
{
//    if(led >YELLOWON)
//      led = CLOSE;
	//HIKE A
//    Send_One_595((0xff-(0x01<<csn))&(0xff-ledRam[led])&(0xff- (0x40<<KEY)));//发送片选，LED，按键
//	Send_One_595(data);
	TM1650_Send(TM1650SegVal[0],SEG7Table[data]);    //DIG2
//	TM1650_Send(DIG2,SEG7Table[Tmp/100]);    //DIG2
//	TM1650_Send(DIG3,SEG7Table[Tmp/100]);    //DIG2
//	TM1650_Send(DIG4,SEG7Table[Tmp/100]);    //DIG2
    LT595_H();
    LT595_L();
}


//#################################################
//-----------------------------------------------
// 数码管显示内容获取
// 先计算显示数字，后计算显示字母,
// 显示字母优先级高于显示数字

//----------------------------------------------- 
void segGetShow(Scan * PtrSegTask)
{
	float tempfloatdata;
	float pointData=0;   //小数部分
	int iterData=0;      //正数部分
	char i;
//-------------------------------------showNum转化为showRam显示---------------------------------------------------------
	tempfloatdata = PtrSegTask ->showNum;
	//这里限幅显示
	if(tempfloatdata  > 9999) tempfloatdata  = 9999;
	else if(tempfloatdata < -999)  tempfloatdata = -999;
	//显示正数
	if(tempfloatdata >=0) 
	{
				//获取小数点
		if(tempfloatdata >=1000)
			poitPosition = 0;
		else if(tempfloatdata >=100)
			poitPosition = 1;	
		else if(tempfloatdata >=10)
			poitPosition = 2;	
		else if(tempfloatdata >0)
			poitPosition = 3;		

		//清空显示
		PtrSegTask ->showRam[3] = 0x00;
		PtrSegTask ->showRam[2] = 0x00;
		PtrSegTask ->showRam[1] = 0x00;
		PtrSegTask ->showRam[0] = 0x00;

		pointData = (pow(10,poitPosition) * (tempfloatdata) );
		iterData = (float)pointData;
		//获取显示的内容，整数与小数
		for(i=0;i< 4;i++)
		{
			PtrSegTask ->showRam[i] = SEG7Table[iterData%10];
			iterData = iterData/10;
		}
		//添加小数点
		PtrSegTask ->showRam[poitPosition] |= 0x80;// 共阴极数码管|=0x80; 共阳极数码管 &=0x7F
	}
	//显示负数
	else
	{	
		//获取小数点
		if(tempfloatdata <= (-100))
			poitPosition = 0;	
		else if(tempfloatdata <=(-10))
			poitPosition = 1;	
		else if(tempfloatdata <0)
			poitPosition = 2;	
		
		PtrSegTask ->showRam[3] = 0x40;//显示负号 共阴极数码管，负号为 0x40；共阳极为0xBF
		PtrSegTask ->showRam[2] = 0x00;
		PtrSegTask ->showRam[1] = 0x00;
		PtrSegTask ->showRam[0] = 0x00;

		pointData = (pow(10,poitPosition) * (-(tempfloatdata) ));
		iterData = (float)pointData;
		//获取显示的内容，整数与小数
		for(i=0;i< 3;i++)
		{
			PtrSegTask ->showRam[i] = SEG7Table[iterData%10];
			iterData = iterData/10;
		}
		//添加小数点
		PtrSegTask ->showRam[poitPosition] |= 0x80;	 //共阴极数码管|=0x80; 共阳极数码管 &=0x7F
		
	}

//------------------------------------showhex强制显示，高优先级显示--------------------------------------------------------
	//showhex显示的优先级高于showRam, showhex内容为非SEGHexBLK时，该位数码管显示showhex的内容

	//左侧第一个数码管，若为负数，显示的数字依次向右移动一位
	if(PtrSegTask ->showhex[3]!= SEGHexBLK)
	{
		PtrSegTask ->showRam[0] = PtrSegTask ->showhex[0]!=SEGHexBLK?SEG7Table[PtrSegTask ->showhex[0]]:PtrSegTask ->showRam[1];
		PtrSegTask ->showRam[1] = PtrSegTask ->showhex[1]!=SEGHexBLK?SEG7Table[PtrSegTask ->showhex[1]]:PtrSegTask ->showRam[2];
		PtrSegTask ->showRam[2] = PtrSegTask ->showhex[2]!=SEGHexBLK?SEG7Table[PtrSegTask ->showhex[2]]:PtrSegTask ->showRam[3];
		PtrSegTask ->showRam[3] = SEG7Table[PtrSegTask ->showhex[3]];
	}
	else//showhex显示的优先级高于showRam
	{
		PtrSegTask ->showRam[0] = PtrSegTask ->showhex[0]!=SEGHexBLK?SEG7Table[PtrSegTask ->showhex[0]]:PtrSegTask ->showRam[0];
		PtrSegTask ->showRam[1] = PtrSegTask ->showhex[1]!=SEGHexBLK?SEG7Table[PtrSegTask ->showhex[1]]:PtrSegTask ->showRam[1];
		PtrSegTask ->showRam[2] = PtrSegTask ->showhex[2]!=SEGHexBLK?SEG7Table[PtrSegTask ->showhex[2]]:PtrSegTask ->showRam[2];
		PtrSegTask ->showRam[3] = PtrSegTask ->showhex[3]!=SEGHexBLK?SEG7Table[PtrSegTask ->showhex[3]]:PtrSegTask ->showRam[3];
	}

}

//#################################################
//-----------------------------------------------
// 数码管扫描函数
// LED显示函数
// 定时器提供时基
//----------------------------------------------- 
void SegScanTask(Scan * PtrSegTask)
{
	  static float temp =0 ;
	  static char hexNum[4]={0};
	  if(PtrSegTask ->scanTick > SCANTIMER)
	  {
	    PtrSegTask ->scanTick =0;

	 //----------------------------数码管显示处理--------------------------------------------
	    //若显示内容不同，则刷新显示
	    if((temp!=PtrSegTask ->showNum)
	    		|| (hexNum[0]!=PtrSegTask ->showhex[0])|| (hexNum[1]!=PtrSegTask ->showhex[1])
	    		|| (hexNum[2]!=PtrSegTask ->showhex[2])|| (hexNum[3]!=PtrSegTask ->showhex[3])
	    		)
	    {
	    	temp = PtrSegTask ->showNum;
	        hexNum[0]=PtrSegTask ->showhex[0];
	        hexNum[1]=PtrSegTask ->showhex[1];
	        hexNum[2]=PtrSegTask ->showhex[2];
	        hexNum[3]=PtrSegTask ->showhex[3];

	        segGetShow(PtrSegTask);
	    	TM1650_Send(DIG1,PtrSegTask ->showRam[0]);  //GID1
	    	TM1650_Send(DIG2,PtrSegTask ->showRam[1]);  //GID2
	    	TM1650_Send(DIG3,PtrSegTask ->showRam[2]);  //GID3
	    	TM1650_Send(DIG4,PtrSegTask ->showRam[3]);  //GID4
	    }

 //----------------------------三色LED的处理--------------------------------------------
	if(PtrSegTask ->Led595.ledTick >= PtrSegTask ->Led595.ledTogglePre)
		PtrSegTask ->Led595.ledTick = 0;
	//三色LED常亮或者占空比范围内亮
    if((PtrSegTask ->Led595.ledONZKB == PtrSegTask ->Led595.ledTogglePre)
    		|| (PtrSegTask ->Led595.ledTick <= PtrSegTask ->Led595.ledONZKB)
    		)
	    RGB_ON |= RGB_Code[PtrSegTask ->Led595.ledState];
	else//三色LED占空比以外灭
		RGB_OFF |= RGB_Code[CLOSE];


 //----------------------------扫描按键-------------------------------------------
//	  S100 = 0,
//	  S101 = 1,
//	  S102 = 2
    if( PtrSegTask ->nowScanKey >= S102) PtrSegTask ->nowScanKey =S100;
    else PtrSegTask ->nowScanKey +=1 ;


    keyVal=0xff;//先初始化一个无效的按键值，再做以下按键值的返回

    if(PtrSegTask ->nowScanKey == S100)
    	keyVal = rS100DAT();//独立IO口按键
    else
    	TM1650_Read(CMD_KEY,&keyVal);//读取TM1650，获取扫描按键S101、S102是否有按下

	if(keyVal == keyValCode[PtrSegTask ->nowScanKey])//判断是否按下
	{
		PtrSegTask ->skey[PtrSegTask ->nowScanKey].pressTick++;
		if(PtrSegTask ->skey[PtrSegTask ->nowScanKey].pressTick >= PtrSegTask ->skey[PtrSegTask ->nowScanKey].pressDEF)
			PtrSegTask ->skey[PtrSegTask ->nowScanKey].ClickCNT = 1;
	}
	else
		PtrSegTask ->skey[PtrSegTask ->nowScanKey].pressTick = 0;

  }

}


//#################################################
//-----------------------------------------------
//处理扫描按键
//-----------------------------------------------
float ftest[]={1,0.1,0.01,0.001};
void handleScanKey(Scan * PtrSegTask)
{
	Uint16 i=0;

	//这里做串口显示哪个按键按下处理
	for(i=0;i<KeyNUM;i++)
	{
		if((PtrSegTask ->skey[i].ClickCNT == 1)  && (PtrSegTask ->skey[i].pressTick ==0))
		{
			//PtrSegTask ->skey[i].ClickCNT = 0;
			printf("\r\nscan key is %s",keyStr[i]);
		}
	}

	//这里做数码管内容变化处理
	if((PtrSegTask ->skey[S100].ClickCNT == 1)  && (PtrSegTask ->skey[S100].pressTick ==0))
	{
		PtrSegTask ->skey[S100].ClickCNT = 0;
		ftestData = 0;
		printf("  显示内容为 %0.3f",PtrSegTask ->showNum);
	}

	if((PtrSegTask ->skey[S101].ClickCNT == 1)  && (PtrSegTask ->skey[S101].pressTick ==0))
	{
		PtrSegTask ->skey[S101].ClickCNT = 0;
		ftestData += ftest[poitPosition];

		printf("  数值变化率为 %0.3f",ftestData);
	}

	if((PtrSegTask ->skey[S102].ClickCNT == 1)  && (PtrSegTask ->skey[S102].pressTick ==0))
	{
		PtrSegTask ->skey[S102].ClickCNT = 0;
		ftestData -= ftest[poitPosition];
		printf("  数值变化率为 %0.3f",ftestData);
	}
}


