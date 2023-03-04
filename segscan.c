#include "HK_all_include.h"
#include <math.h>

char poitPosition=0;//���С�����λ��
float ftestData = 0.01;//��������ݵı仯��
char keyValCode[]={S100_CODE,S101_CODE,S102_CODE};

/*
  uint16_t scanTick;    //ɨ��ʱ��
  SegCS    sltCSn;      //ѡ�����ʾλ��
  uint8_t  showRam[4];  //�������ʾ������,��Ͳ���ʾ�����ݣ�  [0] -->SEG4
  float    showNum;     //�������ʾ������, �����Ǹ�����Ҳ��������������������Χ 0.999 �� 9999
  char     showhex[4];  // ��ʾ���ȼ�����showNum;��Χ 0 --- 0x0F,�Լ�SEGHexBLK������ʾ���� SEGHexBLK
  Led      Led595;           //LED ״̬�ṹ��
  keySlt   nowScanKey;  //��ǰ���͵İ���
  KeyScan  skey[KeyNUM];     //S100 S101����ɨ�裻S102��595���ɨ�谴��

  Led_Clour ledState;      //  LED��ʾ����
  uint16_t   ledONZKB;     //  LED����ռ�ձ�ʱ�䣻 ��λ 1ms
  uint16_t   ledTogglePre; //  LED��˸Ƶ�ʣ� ��λ 1ms
  uint16_t   ledTick;      //  LED����

  uint16_t  pressDEF;  //  ��Ҫ����ʶ��Ĵ���
  uint16_t  pressTimer;//  ���´���ʱ������
  uint16_t  ClickCNT;  //  �Ѿ����µ���Ĵ���
*/
//�������ݳ�ʼ��
Scan Running={0,SEGCS1,{0,0,0,0},-1.99,{SEGHexBLK,SEGHexBLK,SEGHexBLK,SEGHexBLK},{BLEUON,100,1000,0},S100,{{5,0,0},{5,0,0},{5,0,0}}};


//-----------------RGB LED----------------------
#ifdef RGB_HIGE
//GPIO7=RED  GPIO8=GRE GPIO9=BLUE
Uint32 RGB_Code[]={0x0380,0x0080,0x0100,0x0200,0x0380};//�ߵ�ƽ����
#else
//GPIO7=RED GPIO9=GRE  GPIO8=BLUE
Uint32 RGB_Code[]={0x0380,0x0080,0x0200,0x0100,0x0380};  //�͵�ƽ����
#endif

//595����������ı���
//��Ϊ�������Ч��ShumaSlt()���������Ƿ��룬�������ﶨ��Ҫȡ��
char *keyStr[]={"S100","S101","S102"};
//-----------------------------------------------
//
////#################################################
////-----------------------------------------------
//// ����?����y??1��??��?o����y
//// �䨮������?������y��?d1  d2
////  ??��?��?3?
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
// ����ĳһλ�����

// 
//----------------------------------------------- 
char TM1650SegVal[]={DIG1,DIG2,DIG3,DIG4};
void ShumaSlt(uint8_t data,SegCS csn,Led_Clour led, keySlt KEY)
{
//    if(led >YELLOWON)
//      led = CLOSE;
	//HIKE A
//    Send_One_595((0xff-(0x01<<csn))&(0xff-ledRam[led])&(0xff- (0x40<<KEY)));//����Ƭѡ��LED������
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
// �������ʾ���ݻ�ȡ
// �ȼ�����ʾ���֣��������ʾ��ĸ,
// ��ʾ��ĸ���ȼ�������ʾ����

//----------------------------------------------- 
void segGetShow(Scan * PtrSegTask)
{
	float tempfloatdata;
	float pointData=0;   //С������
	int iterData=0;      //��������
	char i;
//-------------------------------------showNumת��ΪshowRam��ʾ---------------------------------------------------------
	tempfloatdata = PtrSegTask ->showNum;
	//�����޷���ʾ
	if(tempfloatdata  > 9999) tempfloatdata  = 9999;
	else if(tempfloatdata < -999)  tempfloatdata = -999;
	//��ʾ����
	if(tempfloatdata >=0) 
	{
				//��ȡС����
		if(tempfloatdata >=1000)
			poitPosition = 0;
		else if(tempfloatdata >=100)
			poitPosition = 1;	
		else if(tempfloatdata >=10)
			poitPosition = 2;	
		else if(tempfloatdata >0)
			poitPosition = 3;		

		//�����ʾ
		PtrSegTask ->showRam[3] = 0x00;
		PtrSegTask ->showRam[2] = 0x00;
		PtrSegTask ->showRam[1] = 0x00;
		PtrSegTask ->showRam[0] = 0x00;

		pointData = (pow(10,poitPosition) * (tempfloatdata) );
		iterData = (float)pointData;
		//��ȡ��ʾ�����ݣ�������С��
		for(i=0;i< 4;i++)
		{
			PtrSegTask ->showRam[i] = SEG7Table[iterData%10];
			iterData = iterData/10;
		}
		//���С����
		PtrSegTask ->showRam[poitPosition] |= 0x80;// �����������|=0x80; ����������� &=0x7F
	}
	//��ʾ����
	else
	{	
		//��ȡС����
		if(tempfloatdata <= (-100))
			poitPosition = 0;	
		else if(tempfloatdata <=(-10))
			poitPosition = 1;	
		else if(tempfloatdata <0)
			poitPosition = 2;	
		
		PtrSegTask ->showRam[3] = 0x40;//��ʾ���� ����������ܣ�����Ϊ 0x40��������Ϊ0xBF
		PtrSegTask ->showRam[2] = 0x00;
		PtrSegTask ->showRam[1] = 0x00;
		PtrSegTask ->showRam[0] = 0x00;

		pointData = (pow(10,poitPosition) * (-(tempfloatdata) ));
		iterData = (float)pointData;
		//��ȡ��ʾ�����ݣ�������С��
		for(i=0;i< 3;i++)
		{
			PtrSegTask ->showRam[i] = SEG7Table[iterData%10];
			iterData = iterData/10;
		}
		//���С����
		PtrSegTask ->showRam[poitPosition] |= 0x80;	 //�����������|=0x80; ����������� &=0x7F
		
	}

//------------------------------------showhexǿ����ʾ�������ȼ���ʾ--------------------------------------------------------
	//showhex��ʾ�����ȼ�����showRam, showhex����Ϊ��SEGHexBLKʱ����λ�������ʾshowhex������

	//����һ������ܣ���Ϊ��������ʾ���������������ƶ�һλ
	if(PtrSegTask ->showhex[3]!= SEGHexBLK)
	{
		PtrSegTask ->showRam[0] = PtrSegTask ->showhex[0]!=SEGHexBLK?SEG7Table[PtrSegTask ->showhex[0]]:PtrSegTask ->showRam[1];
		PtrSegTask ->showRam[1] = PtrSegTask ->showhex[1]!=SEGHexBLK?SEG7Table[PtrSegTask ->showhex[1]]:PtrSegTask ->showRam[2];
		PtrSegTask ->showRam[2] = PtrSegTask ->showhex[2]!=SEGHexBLK?SEG7Table[PtrSegTask ->showhex[2]]:PtrSegTask ->showRam[3];
		PtrSegTask ->showRam[3] = SEG7Table[PtrSegTask ->showhex[3]];
	}
	else//showhex��ʾ�����ȼ�����showRam
	{
		PtrSegTask ->showRam[0] = PtrSegTask ->showhex[0]!=SEGHexBLK?SEG7Table[PtrSegTask ->showhex[0]]:PtrSegTask ->showRam[0];
		PtrSegTask ->showRam[1] = PtrSegTask ->showhex[1]!=SEGHexBLK?SEG7Table[PtrSegTask ->showhex[1]]:PtrSegTask ->showRam[1];
		PtrSegTask ->showRam[2] = PtrSegTask ->showhex[2]!=SEGHexBLK?SEG7Table[PtrSegTask ->showhex[2]]:PtrSegTask ->showRam[2];
		PtrSegTask ->showRam[3] = PtrSegTask ->showhex[3]!=SEGHexBLK?SEG7Table[PtrSegTask ->showhex[3]]:PtrSegTask ->showRam[3];
	}

}

//#################################################
//-----------------------------------------------
// �����ɨ�躯��
// LED��ʾ����
// ��ʱ���ṩʱ��
//----------------------------------------------- 
void SegScanTask(Scan * PtrSegTask)
{
	  static float temp =0 ;
	  static char hexNum[4]={0};
	  if(PtrSegTask ->scanTick > SCANTIMER)
	  {
	    PtrSegTask ->scanTick =0;

	 //----------------------------�������ʾ����--------------------------------------------
	    //����ʾ���ݲ�ͬ����ˢ����ʾ
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

 //----------------------------��ɫLED�Ĵ���--------------------------------------------
	if(PtrSegTask ->Led595.ledTick >= PtrSegTask ->Led595.ledTogglePre)
		PtrSegTask ->Led595.ledTick = 0;
	//��ɫLED��������ռ�ձȷ�Χ����
    if((PtrSegTask ->Led595.ledONZKB == PtrSegTask ->Led595.ledTogglePre)
    		|| (PtrSegTask ->Led595.ledTick <= PtrSegTask ->Led595.ledONZKB)
    		)
	    RGB_ON |= RGB_Code[PtrSegTask ->Led595.ledState];
	else//��ɫLEDռ�ձ�������
		RGB_OFF |= RGB_Code[CLOSE];


 //----------------------------ɨ�谴��-------------------------------------------
//	  S100 = 0,
//	  S101 = 1,
//	  S102 = 2
    if( PtrSegTask ->nowScanKey >= S102) PtrSegTask ->nowScanKey =S100;
    else PtrSegTask ->nowScanKey +=1 ;


    keyVal=0xff;//�ȳ�ʼ��һ����Ч�İ���ֵ���������°���ֵ�ķ���

    if(PtrSegTask ->nowScanKey == S100)
    	keyVal = rS100DAT();//����IO�ڰ���
    else
    	TM1650_Read(CMD_KEY,&keyVal);//��ȡTM1650����ȡɨ�谴��S101��S102�Ƿ��а���

	if(keyVal == keyValCode[PtrSegTask ->nowScanKey])//�ж��Ƿ���
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
//����ɨ�谴��
//-----------------------------------------------
float ftest[]={1,0.1,0.01,0.001};
void handleScanKey(Scan * PtrSegTask)
{
	Uint16 i=0;

	//������������ʾ�ĸ��������´���
	for(i=0;i<KeyNUM;i++)
	{
		if((PtrSegTask ->skey[i].ClickCNT == 1)  && (PtrSegTask ->skey[i].pressTick ==0))
		{
			//PtrSegTask ->skey[i].ClickCNT = 0;
			printf("\r\nscan key is %s",keyStr[i]);
		}
	}

	//��������������ݱ仯����
	if((PtrSegTask ->skey[S100].ClickCNT == 1)  && (PtrSegTask ->skey[S100].pressTick ==0))
	{
		PtrSegTask ->skey[S100].ClickCNT = 0;
		ftestData = 0;
		printf("  ��ʾ����Ϊ %0.3f",PtrSegTask ->showNum);
	}

	if((PtrSegTask ->skey[S101].ClickCNT == 1)  && (PtrSegTask ->skey[S101].pressTick ==0))
	{
		PtrSegTask ->skey[S101].ClickCNT = 0;
		ftestData += ftest[poitPosition];

		printf("  ��ֵ�仯��Ϊ %0.3f",ftestData);
	}

	if((PtrSegTask ->skey[S102].ClickCNT == 1)  && (PtrSegTask ->skey[S102].pressTick ==0))
	{
		PtrSegTask ->skey[S102].ClickCNT = 0;
		ftestData -= ftest[poitPosition];
		printf("  ��ֵ�仯��Ϊ %0.3f",ftestData);
	}
}


