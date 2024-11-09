//############################################################
// FILE: ThreeHall.c
// Created on: 2017年1月18日
// Author: XQ
// summary: ThreeHall
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//DSP/STM32电机控制开发板
//硕历电子
//网址: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//电机控制QQ群：314306105

//decision  Hall_Three.Speed_RPM    feedback speeed /  Hall_Three.ele_angleIQ   electric angle  for dq calculate


//############################################################
#include "ThreeHall.h"
#include "Tim1_PWM.h"
#include "Task_function.h"

extern   Hall   Hall_Three;
extern  logic   logicContr;

uint16_t   HallK1=355; 		// 一阶低通滤波系数 K1:  
uint16_t   HallK2=669 ;  	// 一阶低通滤波系数 K2: K1+K2=1=2…^10=1024
//u16 degree30IQ15=0;    // 182*30  =5462
u16 degree30IQ15=5462;    // 182*30  =5462=>  30*(65536/360)=5462


  u32   init_angle=5462 ;  // 30 degree     4对级 42PM电机   霍尔的初始角度  0---65535  0---360度

// 电机零位 通过反电动势过零比较与霍尔状态定位计算插值角度 
// GPIO在PWM读取与计算三霍尔PMSM位置.pdf
// 仪器测量电机零位：零位检测仪三相永磁电机，三霍尔传感器，旋转变压器电机，正交编码器电机
// 见硕历电子官方网站: 
// https://item.taobao.com/item.htm?spm=a1z10.1-c-s.w4004-18047096558.4.63fa4eb3jjz7EB&id=566491697412
void  ThreeHallPara_init(void )
{
	// 假设零位点电机6 4 5 1 3 2是0度 60度 120度 180 240 300  0(360)
	
	 Hall_Three.Hall_num[0]=5;
   Hall_Three.Hall_num[1]=1;
   Hall_Three.Hall_num[2]=3;
   Hall_Three.Hall_num[3]=2;
   Hall_Three.Hall_num[4]=6;
   Hall_Three.Hall_num[5]=4;
	
	
	
	/*
	
   Hall_Three.Hall_num[0]=5;
   Hall_Three.Hall_num[1]=1;
   Hall_Three.Hall_num[2]=3;
   Hall_Three.Hall_num[3]=2;
   Hall_Three.Hall_num[4]=6;
   Hall_Three.Hall_num[5]=4;
	
	
	*/

   Hall_Three.Hall_angle[0]=65535;  // 1
   Hall_Three.Hall_angle[1]=10923;  // 1/6    60*182
   Hall_Three.Hall_angle[2]=21845;  // 1/3
   Hall_Three.Hall_angle[3]=32678;  // 1/2
   Hall_Three.Hall_angle[4]=43691;  // 2/3
   Hall_Three.Hall_angle[5]=54613;  // 5/6
   	
   Hall_Three.step_angle_error =0;  //   
   Hall_Three.Poles=4;   
   Hall_Three.initial_angle=init_angle;   
	 Hall_Three.speed_coeff=(1000*60)/(2*Hall_Three.Poles ); // 2毫秒计算一次角度差值 1000/2ms=500   =7500
	 //Hall_Three.speed_coeff= 120000/Hall_Three.Poles;
}


void ThreeHallanglecale(void)  // 一个PWM周期执行一次   found ele_angleIQ
{
  //在PWM中读取霍尔的IO状态，在二个相邻状态不一致的时候刚好是整60度的霍尔换向时刻。
  // 即是程序中判断  if ( Hall_Three.Hall_State!=Hall_Three.OldHall_State )	 
	
	   if (GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13)==1) 
             Hall_Three.HallUVW[0]=1;
	   else
		     Hall_Three.HallUVW[0]=0; 
   	   if (GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_14)==1) 
             Hall_Three.HallUVW[1]=1;
	   else
		     Hall_Three.HallUVW[1]=0; 
	   if (GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_15)==1) 
             Hall_Three.HallUVW[2]=1;
	   else
		     Hall_Three.HallUVW[2]=0; 
	 
	  Hall_Three.Hall_State = Hall_Three.HallUVW[0] +(Hall_Three.HallUVW[1]<<1) +(Hall_Three.HallUVW[2]<<2);

    if ( Hall_Three.Hall_State!=Hall_Three.OldHall_State )
      {
    	  Hall_Three.HallLX_State=Hall_Three.Hall_State + (Hall_Three.OldHall_State<<4) ;   //  ex: 0x45:  //    4 to  5
  
 // 以正转0x45状态为准，有霍尔4变为5，每一个状态角度变化范围为60度，二每一个状态真实角度变化是又误差  //三霍尔计算转子位置是通过把二个相邻霍尔状态内速度看作为恒转速，但是电机实际不是恒定转速
 //所以在使 用计算转子位置角度时，每个状态之间的角度变化量可能大于60度也可能小于60度，实际变化量  //Hall_Three.step_angle_error = 60度+ 此时刻角值-实际控制应用电角。区间步进角度值//（Hall_Three.step_angle_error=60度+ Hall_Three.Hall_angle[0]-实际电角度）
//在一周期换的时候可能会出现大于65535或者小于0时候（此时减一周期）

    	  switch (Hall_Three.HallLX_State )
    	        {
								
								
													
									 case 0x45:  //    4 to  5
										{  // [Hall_Three.step_angle_error ]actual delta angle = 60 degree [Hall_Three.Hall_angle[1]=10923]  +   now angle  -  actual control ele angle   [sida e]
											 Hall_Three.step_angle_error=Hall_Three.Hall_angle[1]+Hall_Three.Hall_angle[0]-Hall_Three.angleIQ;    	           
											 if( Hall_Three.step_angle_error>65536 )
											 Hall_Three.step_angle_error-=65536;
											 if( Hall_Three.step_angle_error<0 )
											 Hall_Three.step_angle_error+=65536;    	            
											 Hall_Three.Move_State=1;   //CW
										 }
									 break;
								 
							
								 
						
									 case 0x15:    //    1 to  5
										 {
												Hall_Three.step_angle_error=  Hall_Three.Hall_angle[1]+Hall_Three.angleIQ-Hall_Three.Hall_angle[0];
												if( Hall_Three.step_angle_error>65536 )
												Hall_Three.step_angle_error-=65536;
												if( Hall_Three.step_angle_error<0 )
												Hall_Three.step_angle_error+=65536;
												Hall_Three.Move_State=2;   //CCW
										 }
										 
										 
    	             break;
									


													 
									 case 0x51:    //    5 to  1
										 {
												Hall_Three.step_angle_error=  Hall_Three.Hall_angle[1]+Hall_Three.Hall_angle[1]- Hall_Three.angleIQ;
												if( Hall_Three.step_angle_error>65536 )
												Hall_Three.step_angle_error-=65536;
												if( Hall_Three.step_angle_error<0 )
												Hall_Three.step_angle_error+=65536;    	           
												Hall_Three.Move_State=1;
										 }
									 break;
								 
							
									 case  0x31:
											{
												Hall_Three.step_angle_error= Hall_Three.Hall_angle[1]+Hall_Three.angleIQ- Hall_Three.Hall_angle[1]; 
												if( Hall_Three.step_angle_error>65536 )
												Hall_Three.step_angle_error-=65536;
												if( Hall_Three.step_angle_error<0 )
												Hall_Three.step_angle_error+=65536;    	             
												Hall_Three.Move_State=2;
											}
									 break;
							 	
									
									
																
									 case 0x13:
											{
												Hall_Three.step_angle_error= Hall_Three.Hall_angle[1]+Hall_Three.Hall_angle[2]- Hall_Three.angleIQ ;
												if( Hall_Three.step_angle_error>65536 )
												Hall_Three.step_angle_error-=65536;
												if( Hall_Three.step_angle_error<0 )
												Hall_Three.step_angle_error+=65536; 
												Hall_Three.Move_State=1;
											}
									 break;
								
									 case 0x23:
											{
												Hall_Three.step_angle_error= Hall_Three.Hall_angle[1]+Hall_Three.angleIQ- Hall_Three.Hall_angle[2];
												if( Hall_Three.step_angle_error>65536 )
												Hall_Three.step_angle_error-=65536;
												if( Hall_Three.step_angle_error<0 )
												Hall_Three.step_angle_error+=65536;
												Hall_Three.Move_State=2;
											}
										break;
																	
									 case 0x32:
											{
												Hall_Three.step_angle_error= Hall_Three.Hall_angle[1]+Hall_Three.Hall_angle[3]- Hall_Three.angleIQ ;
												if( Hall_Three.step_angle_error>65536 )
												Hall_Three.step_angle_error-=65536;
												if( Hall_Three.step_angle_error<0 )
												Hall_Three.step_angle_error+=65536;
												Hall_Three.Move_State=1;
											}
									 break;
								
									 case 0x62:
											{
												Hall_Three.step_angle_error= Hall_Three.Hall_angle[1]+Hall_Three.angleIQ- Hall_Three.Hall_angle[3] ;
												if( Hall_Three.step_angle_error>65536 )
												Hall_Three.step_angle_error-=65536;
												if( Hall_Three.step_angle_error<0 )
												Hall_Three.step_angle_error+=65536; 
												Hall_Three.Move_State=2;
											}
										break;
																		
									 case 0x26:
											{
												Hall_Three.step_angle_error= Hall_Three.Hall_angle[1]+Hall_Three.Hall_angle[4]- Hall_Three.angleIQ;
												if( Hall_Three.step_angle_error>65536 )
												Hall_Three.step_angle_error-=65536;
												if( Hall_Three.step_angle_error<0 )
												Hall_Three.step_angle_error+=65536; 
												Hall_Three.Move_State=1;
											}
									 break;
									
								
									 case 0x46:
											{
												Hall_Three.step_angle_error= Hall_Three.Hall_angle[1]+Hall_Three.angleIQ- Hall_Three.Hall_angle[4] ;
												if( Hall_Three.step_angle_error>65536 )
												Hall_Three.step_angle_error-=65536;
												if( Hall_Three.step_angle_error<0 )
												Hall_Three.step_angle_error+=65536;
												Hall_Three.Move_State=2;
											}
										break;		
																	
									 case 0x64:
											{
												Hall_Three.step_angle_error= Hall_Three.Hall_angle[1]+Hall_Three.Hall_angle[5]- Hall_Three.angleIQ;
												if( Hall_Three.step_angle_error>65536 )
												Hall_Three.step_angle_error-=65536;
												if( Hall_Three.step_angle_error<0 )
												Hall_Three.step_angle_error+=65536;
												Hall_Three.Move_State=1;
											}
									 break;
						  		
									 case 0x54:
											{
												Hall_Three.step_angle_error= Hall_Three.Hall_angle[1]+Hall_Three.angleIQ- Hall_Three.Hall_angle[5];
												if( Hall_Three.step_angle_error>65536 )
												Hall_Three.step_angle_error-=65536;
												if( Hall_Three.step_angle_error<0 )
												Hall_Three.step_angle_error+=65536;
												Hall_Three.Move_State=2;
											}
										break;
									

    	           default:
    	          	{
    	          		Hall_Three.ele_angleIQ=0;
										Hall_Three.Move_State=0;
    	          	}
    	           break;
    	         } 
  // 在每一次PWM中计算，Hall_Three.Speed_count++ 
  // 每一个霍尔变换时候读取计算每个PWM中断的角度步进值，同时读取后清0.

		  Hall_Three.Speed_countFitter= _IQ10mpy(HallK2, Hall_Three.Speed_countFitter)+_IQ10mpy(HallK1,  Hall_Three.Speed_count);
		 // Hall_Three.Speed_count	电机控制中的PWM计数							  
			Hall_Three.Speed_count_old=Hall_Three.Speed_countFitter; //电机控制中的PWM计数	存储历史	
      Hall_Three.step_angle = Hall_Three.step_angle_error/Hall_Three.Speed_countFitter; // 电机60度一个步进角计算
     // 一阶数字低通把二个霍尔状态之间时间的计数平滑滤波，HallK1+ HallK2=1024，用y=a*y0+(1-a)*x;
		  Hall_Three.step_angleFitter= _IQ10mpy(HallK2, Hall_Three.step_angleFitter)+_IQ10mpy(HallK1,  Hall_Three.step_angle);	
  		Hall_Three.Speed_count= 0;        
     }

   else  if ( Hall_Three.Hall_State==Hall_Three.OldHall_State )
     {
    	 Hall_Three.Speed_count++;   
       if( Hall_Three.Speed_count>=8000 )  // 判断停车，和启动控制 并清除参数
    	 {
    		 Hall_Three.Speed_count=0;   
    		 Hall_Three.Speed_RPM= 0 ;
    		 Hall_Three.step_angleFitter=0;	
				 Hall_Three.Move_State=0;
						switch (Hall_Three.Hall_State )
						{
							case 0x5:
							{
							Hall_Three.angleIQ = Hall_Three.Hall_angle[0]+degree30IQ15;   // ele angle
							// 5462=30度在启动的几个霍尔状态中速度较低在二个霍尔状态中间角度控制 
							}
						  break;
						  case 0x1:
							{
							Hall_Three.angleIQ = Hall_Three.Hall_angle[1]+degree30IQ15;
							}
						  break;
						  case 0x3:
							{
							Hall_Three.angleIQ = Hall_Three.Hall_angle[2]+degree30IQ15;
							}
						  break;
						  case 0x2:
							{
							Hall_Three.angleIQ = Hall_Three.Hall_angle[3]+degree30IQ15;
							}
						  break;
						  case 0x6:
							{
							Hall_Three.angleIQ = Hall_Three.Hall_angle[4]+degree30IQ15;
							}
						  break;
						  case 0x4:
							{
								Hall_Three.angleIQ = Hall_Three.Hall_angle[5]+degree30IQ15;
							}
						  break;

						  default:
							{
								Hall_Three.ele_angleIQ=0;
							}
						  break;
						 }
       }
   }		 
      if ( Hall_Three.Move_State==1 )
       {
        Hall_Three.angleIQ = Hall_Three.angleIQ + Hall_Three.step_angleFitter ;
       }
       else if ( Hall_Three.Move_State==2 )
       {
        Hall_Three.angleIQ = Hall_Three.angleIQ - Hall_Three.step_angleFitter ;
       }

     if( Hall_Three.angleIQ> 65536)
  	  Hall_Three.angleIQ-=65536;
     else if( Hall_Three.angleIQ<0)
  	  Hall_Three.angleIQ+=65536;
     // d轴电机磁极位置和电机位置,A相霍尔上升沿与A相反电动势下降0点的时间角度差
     Hall_Three.ele_angleIQ = Hall_Three.angleIQ-Hall_Three.initial_angle ;  ////    // speed ele angle

     if( Hall_Three.ele_angleIQ> 65536)
  	  Hall_Three.ele_angleIQ-=65536;
     else if( Hall_Three.ele_angleIQ<0)
  	  Hall_Three.ele_angleIQ+=65536;

      Hall_Three.OldHall_State=Hall_Three.Hall_State ;
}

 // 电机在2ms时间计算角度变化量。即是公式：
 // Hall_Three.Speed_ele_angleIQ =Hall_Three.ele_angleIQ -Hall_Three.old_ele_angleIQ  
 // 防止超过65535和小于0，把差值一阶滤波，插值变化量乘系数就可以得到速度。
 // 速度信号的计算可以简单根据转位置的步进角计算或者直接根据角度在一定周期内的变化量计算
 // 其中移位16是把角度变化量归1的一个系数，变化角度/360度，在乘一个系数得到速度，
 // 可以通过示波器测量一个霍尔周期时间来计算。
 // 假设一个霍尔周期时间15ms，电机极对数为4，速度RPM=1/T*60/p=1000/15*60/4=1000rpm 
 // 然后看在线看角度变化量（或者通讯发出来），速度RPM=变化量/65535*K=1000，求得系数K。 
 // 系数 :Hall_Three.speed_coeff   

void Hall_Three_Speedcale(void)  // 2ms执行一次  result-> Hall_Three.Speed_RPM 
{
  if( Hall_Three.Move_State==1) // 根据正反转
  {
	   Hall_Three.Speed_ele_angleIQ =Hall_Three.ele_angleIQ -Hall_Three.old_ele_angleIQ ;      //  Hall_Three.Speed_ele_angleIQ  -> 0 ~ 22000
	   if( Hall_Three.Speed_ele_angleIQ <0)
		     Hall_Three.Speed_ele_angleIQ+= 65536;
  }
  else  if( Hall_Three.Move_State==2)
  {
  	 Hall_Three.Speed_ele_angleIQ =Hall_Three.old_ele_angleIQ -Hall_Three.ele_angleIQ;
  	 if( Hall_Three.Speed_ele_angleIQ < 0)
  	     Hall_Three.Speed_ele_angleIQ+=65536; 	 
  }
	else if( Hall_Three.Move_State==0)   // Hall_Three.Move_State=0 
	{
		 Hall_Three.Speed_ele_angleIQ=0;
	}
 	 Hall_Three.Speed_ele_angleIQFitter= _IQ10mpy(HallK1, Hall_Three.Speed_ele_angleIQFitter)+_IQ10mpy(HallK2,  Hall_Three.Speed_ele_angleIQ);
   Hall_Three.Speed_RPM = (Hall_Three.Speed_ele_angleIQ*Hall_Three.speed_coeff)>>16; // 最大角度 2pi是一圈 65536
	 Hall_Three.old_ele_angleIQ = Hall_Three.ele_angleIQ ;
}
 


//===========================================================================
// No more.
//===========================================================================
