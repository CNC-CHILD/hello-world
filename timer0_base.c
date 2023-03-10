/*
 * timer0_base.c
 *
 *  Created on: 2013-10-22
 *      Author: cotto
 */
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "HK_all_include.h"


timer0 timer0Base={0,0};

__interrupt void cpu_timer0_isr(void);


//---------------------定时器0初始化------------------------
//1ms 中断
void Timer0_init()
{
	InitCpuTimers();

	//中断配置步骤-----1,开启模块中断使能，位于 Timer->RegsAddr->TCR.bit.TIE = 1;
	ConfigCpuTimer(&CpuTimer0, 60, 1000);//60MHz CPU Freq, 1 millisecond Period (in uSeconds)
	CpuTimer0Regs.TCR.all = 0x4001;		   // Use write-only instruction to set TSS bit = 0

	//中断配置步骤-----2，重映射中断服务函数
	// Interrupts that are used in this example are re-mapped to
	// ISR functions found within this file.
	EALLOW;
	PieVectTable.TINT0 = &cpu_timer0_isr;
	EDIS;
	//中断配置步骤-----3，连接CPU中断Y
	IER |= M_INT1;
	//中断配置步骤-----4，连接Y中断里的第几位
	PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
}


__interrupt void cpu_timer0_isr(void)
{
   char i;
   timer0Base.msCounter++;
   timer0Base.Mark_Para.Status_Bits.OnemsdFlag = 1;

   Running.scanTick++;//数码管芯片TM1650扫描计时
   Running.Led595.ledTick++;//RGB LED的计时

   //串口通讯处理
   if(SCI_Msg.Mark_Para.Status_Bits.rFifoDataflag == 1)
   {
	   SCI_Msg.timerOut++;
   }

   //IIC判断超时的计时操作
   for(i=0;i<IIC_NODE_NUM;i++)
   {
	   PtrMsg[i]->IIC_TimerOUT  = (PtrMsg[i]->MasterStatus == IIC_IDLE)?0:(PtrMsg[i]->IIC_TimerOUT+1);
   }

   // Acknowledge this interrupt to receive more interrupts from group 1
	EALLOW;
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
	EDIS;

}


