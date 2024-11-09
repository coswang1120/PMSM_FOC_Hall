//############################################################
// FILE: ThreeHall.c
// Created on: 2017��1��18��
// Author: XQ
// summary: ThreeHall
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//��Ȩ���У�����ؾ�
//DSP/STM32������ƿ�����
//˶������
//��ַ: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//�������QQȺ��314306105

//decision  Hall_Three.Speed_RPM    feedback speeed /  Hall_Three.ele_angleIQ   electric angle  for dq calculate


//############################################################
#include "ThreeHall.h"
#include "Tim1_PWM.h"
#include "Task_function.h"

extern   Hall   Hall_Three;
extern  logic   logicContr;

uint16_t   HallK1=355; 		// һ�׵�ͨ�˲�ϵ�� K1:  
uint16_t   HallK2=669 ;  	// һ�׵�ͨ�˲�ϵ�� K2: K1+K2=1=2��^10=1024
//u16 degree30IQ15=0;    // 182*30  =5462
u16 degree30IQ15=5462;    // 182*30  =5462=>  30*(65536/360)=5462


  u32   init_angle=5462 ;  // 30 degree     4�Լ� 42PM���   �����ĳ�ʼ�Ƕ�  0---65535  0---360��

// �����λ ͨ�����綯�ƹ���Ƚ������״̬��λ�����ֵ�Ƕ� 
// GPIO��PWM��ȡ�����������PMSMλ��.pdf
// �������������λ����λ������������ŵ��������������������ת��ѹ��������������������
// ��˶�����ӹٷ���վ: 
// https://item.taobao.com/item.htm?spm=a1z10.1-c-s.w4004-18047096558.4.63fa4eb3jjz7EB&id=566491697412
void  ThreeHallPara_init(void )
{
	// ������λ����6 4 5 1 3 2��0�� 60�� 120�� 180 240 300  0(360)
	
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
	 Hall_Three.speed_coeff=(1000*60)/(2*Hall_Three.Poles ); // 2�������һ�νǶȲ�ֵ 1000/2ms=500   =7500
	 //Hall_Three.speed_coeff= 120000/Hall_Three.Poles;
}


void ThreeHallanglecale(void)  // һ��PWM����ִ��һ��   found ele_angleIQ
{
  //��PWM�ж�ȡ������IO״̬���ڶ�������״̬��һ�µ�ʱ��պ�����60�ȵĻ�������ʱ�̡�
  // ���ǳ������ж�  if ( Hall_Three.Hall_State!=Hall_Three.OldHall_State )	 
	
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
  
 // ����ת0x45״̬Ϊ׼���л���4��Ϊ5��ÿһ��״̬�Ƕȱ仯��ΧΪ60�ȣ���ÿһ��״̬��ʵ�Ƕȱ仯�������  //����������ת��λ����ͨ���Ѷ������ڻ���״̬���ٶȿ���Ϊ��ת�٣����ǵ��ʵ�ʲ��Ǻ㶨ת��
 //������ʹ �ü���ת��λ�ýǶ�ʱ��ÿ��״̬֮��ĽǶȱ仯�����ܴ���60��Ҳ����С��60�ȣ�ʵ�ʱ仯��  //Hall_Three.step_angle_error = 60��+ ��ʱ�̽�ֵ-ʵ�ʿ���Ӧ�õ�ǡ����䲽���Ƕ�ֵ//��Hall_Three.step_angle_error=60��+ Hall_Three.Hall_angle[0]-ʵ�ʵ�Ƕȣ�
//��һ���ڻ���ʱ����ܻ���ִ���65535����С��0ʱ�򣨴�ʱ��һ���ڣ�

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
  // ��ÿһ��PWM�м��㣬Hall_Three.Speed_count++ 
  // ÿһ�������任ʱ���ȡ����ÿ��PWM�жϵĽǶȲ���ֵ��ͬʱ��ȡ����0.

		  Hall_Three.Speed_countFitter= _IQ10mpy(HallK2, Hall_Three.Speed_countFitter)+_IQ10mpy(HallK1,  Hall_Three.Speed_count);
		 // Hall_Three.Speed_count	��������е�PWM����							  
			Hall_Three.Speed_count_old=Hall_Three.Speed_countFitter; //��������е�PWM����	�洢��ʷ	
      Hall_Three.step_angle = Hall_Three.step_angle_error/Hall_Three.Speed_countFitter; // ���60��һ�������Ǽ���
     // һ�����ֵ�ͨ�Ѷ�������״̬֮��ʱ��ļ���ƽ���˲���HallK1+ HallK2=1024����y=a*y0+(1-a)*x;
		  Hall_Three.step_angleFitter= _IQ10mpy(HallK2, Hall_Three.step_angleFitter)+_IQ10mpy(HallK1,  Hall_Three.step_angle);	
  		Hall_Three.Speed_count= 0;        
     }

   else  if ( Hall_Three.Hall_State==Hall_Three.OldHall_State )
     {
    	 Hall_Three.Speed_count++;   
       if( Hall_Three.Speed_count>=8000 )  // �ж�ͣ�������������� ���������
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
							// 5462=30���������ļ�������״̬���ٶȽϵ��ڶ�������״̬�м�Ƕȿ��� 
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
     // d�����ż�λ�ú͵��λ��,A�������������A�෴�綯���½�0���ʱ��ǶȲ�
     Hall_Three.ele_angleIQ = Hall_Three.angleIQ-Hall_Three.initial_angle ;  ////    // speed ele angle

     if( Hall_Three.ele_angleIQ> 65536)
  	  Hall_Three.ele_angleIQ-=65536;
     else if( Hall_Three.ele_angleIQ<0)
  	  Hall_Three.ele_angleIQ+=65536;

      Hall_Three.OldHall_State=Hall_Three.Hall_State ;
}

 // �����2msʱ�����Ƕȱ仯�������ǹ�ʽ��
 // Hall_Three.Speed_ele_angleIQ =Hall_Three.ele_angleIQ -Hall_Three.old_ele_angleIQ  
 // ��ֹ����65535��С��0���Ѳ�ֵһ���˲�����ֵ�仯����ϵ���Ϳ��Եõ��ٶȡ�
 // �ٶ��źŵļ�����Լ򵥸���תλ�õĲ����Ǽ������ֱ�Ӹ��ݽǶ���һ�������ڵı仯������
 // ������λ16�ǰѽǶȱ仯����1��һ��ϵ�����仯�Ƕ�/360�ȣ��ڳ�һ��ϵ���õ��ٶȣ�
 // ����ͨ��ʾ��������һ����������ʱ�������㡣
 // ����һ����������ʱ��15ms�����������Ϊ4���ٶ�RPM=1/T*60/p=1000/15*60/4=1000rpm 
 // Ȼ�����߿��Ƕȱ仯��������ͨѶ�����������ٶ�RPM=�仯��/65535*K=1000�����ϵ��K�� 
 // ϵ�� :Hall_Three.speed_coeff   

void Hall_Three_Speedcale(void)  // 2msִ��һ��  result-> Hall_Three.Speed_RPM 
{
  if( Hall_Three.Move_State==1) // ��������ת
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
   Hall_Three.Speed_RPM = (Hall_Three.Speed_ele_angleIQ*Hall_Three.speed_coeff)>>16; // ���Ƕ� 2pi��һȦ 65536
	 Hall_Three.old_ele_angleIQ = Hall_Three.ele_angleIQ ;
}
 


//===========================================================================
// No more.
//===========================================================================
