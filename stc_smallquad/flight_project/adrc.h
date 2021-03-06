#ifndef _ADRC_H_
#define _ADRC_H_

#if 0
/*----------------------------------------------------------------------------------------------------------------------/
		*               ? � � ? � � 买� �??使� ??? ? ? � ? � � ? ? � ? ?��  � 
		*               ? ? � ? ?��  � � ?��?� � ? ? � � � 买� � 
		*               � 买� � 为�  ? � ? ?��  ? � � ?��?
		*               ? � � ? 许�  � � � � � 代�  ? � � � �?
		*               � � � � 代�  ?�� ��?� � � 人�  费�?载�?
		*               ?��??�以此� �? ? ?��?� ? ?��?述�?为�?
		*               ? ? � ? ?��  � � � 以�?� 解� ��?� � 
-----------------------------------------------------------------------------------------------------------------------/
		*               ? ?��?? ? � ? � �??? 人� ��  � ? 人�?? � � � 
		*               开� � ? � � �??? ? � � ?�� �� � ? � ?��  ? � � � 
-----------------------------------------------------------------------------------------------------------------------/
	*		? ? � ? 开� � ??V1.1	�??� ?�大� ? By.YuYi
	*		CSDN? �? http://blog.csdn.net/u011992534
	*               � ??D� NamelessCotrun? ? � ??
	*               ? ? � ? 开� � ??Q群�?540707961
		*               https://shop348646912.taobao.com/?spm=2013.1.1000126.2.5ce78a88ht1sO2
		*               ?�度贴� ?? ? � ? 开� � ??
		*               � ?�� ��  :2017/10/30
		*               ? ? � V1.1
		*               ? ? ?�? � ? ? � 究� 
		*               Copyright(C) �??� ?�大� ?  ? � ? ?��   2017-2019
		*               All rights reserved
----------------------------------------------------------------------------------------------------------------------*/
typedef struct
{
		/*****� */
		float x1; //� � � ? ? ?�� ? 
		float x2; //� � � ? ? ?�� ? � ? �?
		float r; //?�� �尺�?
		float h; //ADRC系�?� ? ?�� ?
		int N0; //� � � ? ?�解?�� �??� h0=N*h

		float h0;
		float fh; //?�? � ? ? ? �??� ? 
		/*****?��??�� */
		/******已系� */
		float z1;
		float z2;
		float z3; //?��  ?�� �� �象� ?��?� ?��?? ? ? ?�� �信? 
		float e; //系�??�� � � 
		float y; //系�?� ?��  
		float fe;
		float fe1;
		float beta_01;
		float beta_02;
		float beta_03;
		float b;


		/**********系�??�� */
		float e0; //?�� � � � ? �?
		float e1; //?�� ? � 
		float e2; //?�� ? � ? �?
		float u0; //? 线性�?? 系�?� ??
		float u; //�? �� �补?��  ? � ??
		float b0; //?�� �补??

		/*********� */
		float beta_0; //线�?
		float beta_1; //? 线性�?? ? ??
		float beta_2; //u0=beta_1*e1+beta_2*e2+(beta_0*e0);
		/*********� */
		float alpha1; //u0=beta_1*fal(e1,alpha1,zeta)+beta_2*fal(e2,alpha2,zeta)
		float alpha2; //0<alpha1<1<alpha2
		float zeta; //线性� ��  ?�� �� �度
		/*********� */
		float h1; //u0=-fhan(e1,e2,r,h1);
		int N1; //� � � ? ?�解?�� �??� h0=N*h
		/*********� */
		float c; //u0=-fhan(e1,c*e2*e2,r,h1);


}Fhan_Data;



void ADRC_Init(Fhan_Data *fhan_Input1, Fhan_Data *fhan_Input2);
void Fhan_ADRC(Fhan_Data *fhan_Input, float expect_ADRC);
void ADRC_Control(Fhan_Data *fhan_Input, float expect_ADRC, float feedback);
float Fal_ADRC(float e, float alpha, float zeta);
float Constrain_Float(float amt, float low, float high);


extern Fhan_Data ADRC_Pitch_Controller,ADRC_Roll_Controller;
#endif
#endif
