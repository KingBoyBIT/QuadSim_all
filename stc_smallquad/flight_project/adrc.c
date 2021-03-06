#include <math.h>
#include "ADRC.h"
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
Fhan_Data ADRC_Pitch_Controller;
Fhan_Data ADRC_Roll_Controller;
const float ADRC_Unit[3][16] =
{
	/*TD� � � ? ??  ?��??�? TD,h0=N*h      ?��??�� � � ??SO           ?�� �补??    ? 线性�?? */
	/*  r     h      N                  beta_01   beta_02    beta_03     b0     beta_0  beta_1     beta_2     N1     C    alpha1  alpha2  zeta  b*/
	{ 300000, 0.005, 3, 300, 4000, 10000, 0.001, 0.002, 2.0, 0.0010, 5, 5, 0.8, 1.5, 50, 0 },
	{ 300000, 0.005, 3, 300, 4000, 10000, 0.001, 0.002, 2.0, 0.0010, 5, 5, 0.8, 1.5, 50, 0 },
	{ 300000, 0.005, 3, 300, 4000, 10000, 0.001, 0.002, 1.2, 0.0005, 5, 5, 0.8, 1.5, 50, 0 },
};

/*边�  �?  */
float Constrain_Float(float amt, float low, float high)
{
	return ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)));
}

int Sign_ADRC(float Input)
{
	int output = 0;
	if (Input > 1E-6)
		output = 1;
	else if (Input < -1E-6)
		output = -1;
	else
		output = 0;
	return (output);
}

int Fsg_ADRC(float x, float d)
{
	int output = 0;
	output = (Sign_ADRC(x + d) - Sign_ADRC(x - d)) / 2;
	return (output);
}


void ADRC_Init(Fhan_Data *fhan_Input1, Fhan_Data *fhan_Input2)
{
	fhan_Input1->r = ADRC_Unit[0][0];
	fhan_Input1->h = ADRC_Unit[0][1];
	fhan_Input1->N0 = (int)(ADRC_Unit[0][2]);
	fhan_Input1->beta_01 = ADRC_Unit[0][3];
	fhan_Input1->beta_02 = ADRC_Unit[0][4];
	fhan_Input1->beta_03 = ADRC_Unit[0][5];
	fhan_Input1->b0 = ADRC_Unit[0][6];
	fhan_Input1->beta_0 = ADRC_Unit[0][7];
	fhan_Input1->beta_1 = ADRC_Unit[0][8];
	fhan_Input1->beta_2 = ADRC_Unit[0][9];
	fhan_Input1->N1 = (int)(ADRC_Unit[0][10]);
	fhan_Input1->c = ADRC_Unit[0][11];

	fhan_Input1->alpha1 = ADRC_Unit[0][12];
	fhan_Input1->alpha2 = ADRC_Unit[0][13];
	fhan_Input1->zeta = ADRC_Unit[0][14];
	fhan_Input1->b = ADRC_Unit[0][15];

	fhan_Input2->r = ADRC_Unit[1][0];
	fhan_Input2->h = ADRC_Unit[1][1];
	fhan_Input2->N0 = (int)(ADRC_Unit[1][2]);
	fhan_Input2->beta_01 = ADRC_Unit[1][3];
	fhan_Input2->beta_02 = ADRC_Unit[1][4];
	fhan_Input2->beta_03 = ADRC_Unit[1][5];
	fhan_Input2->b0 = ADRC_Unit[1][6];
	fhan_Input2->beta_0 = ADRC_Unit[1][7];
	fhan_Input2->beta_1 = ADRC_Unit[1][8];
	fhan_Input2->beta_2 = ADRC_Unit[1][9];
	fhan_Input2->N1 = (int)(ADRC_Unit[1][10]);
	fhan_Input2->c = ADRC_Unit[1][11];

	fhan_Input2->alpha1 = ADRC_Unit[1][12];
	fhan_Input2->alpha2 = ADRC_Unit[1][13];
	fhan_Input2->zeta = ADRC_Unit[1][14];
	fhan_Input2->b = ADRC_Unit[1][15];
}



//ADRC?�? � � � ? ??D� ?��?? � � fhan
void Fhan_ADRC(Fhan_Data *fhan_Input, float expect_ADRC) //� ? ADRC� �??� 
{
	float d = 0, a0 = 0, y = 0, a1 = 0, a2 = 0, a = 0;
	float x1_delta = 0; //ADRC?�� � � � � �?
	x1_delta = fhan_Input->x1 - expect_ADRC; //??1-v(k)?�代x1� ?��??�� �� ��  � 
	fhan_Input->h0 = fhan_Input->N0 * fhan_Input->h; //??0?�代h� 解� �� �? � � � ? ?�� �??� ? � 
	d = fhan_Input->r * fhan_Input->h0 * fhan_Input->h0; //d=rh^2;
	a0 = fhan_Input->h0 * fhan_Input->x2; //a0=h*x2
	y = x1_delta + a0; //y=x1+a0
	a1 = sqrt(d * (d + 8 * abs(y))); //a1=sqrt(d*(d+8*ABS(y))])
	a2 = a0 + Sign_ADRC(y) * (a1 - d) / 2; //a2=a0+sign(y)*(a1-d)/2;
	a = (a0 + y) * Fsg_ADRC(y, d) + a2 * (1 - Fsg_ADRC(y, d));
	fhan_Input->fh = -fhan_Input->r * (a / d) * Fsg_ADRC(a, d)
					 - fhan_Input->r * Sign_ADRC(a) * (1 - Fsg_ADRC(a, d)); //� ?�� �? � ? ? ? �??� ? 
	fhan_Input->x1 += fhan_Input->h * fhan_Input->x2; //� ?�� �? � � ?�� ? x1
	fhan_Input->x2 += fhan_Input->h * fhan_Input->fh; //� ?�� �? � � ?�� ? � ? x2
}


//? ?��  � ? � 线性� ��  � � � 次� �� ?
float Fal_ADRC(float e, float alpha, float zeta)
{
	int s = 0;
	float fal_output = 0;
	s = (Sign_ADRC(e + zeta) - Sign_ADRC(e - zeta)) / 2;
	fal_output = e * s / (pow(zeta, 1 - alpha)) + pow(abs(e), alpha) * Sign_ADRC(e) * (1 - s);
	return (fal_output);
}




/************?��??�� */
//?�� � � ?��  ??eta01=1/h  beta02=1/(3*h^2)  beta03=2/(8^2*h^3) ...
void ESO_ADRC(Fhan_Data *fhan_Input)
{
	fhan_Input->e = fhan_Input->z1 - fhan_Input->y; //?�� � � 

	fhan_Input->fe = Fal_ADRC(fhan_Input->e, 0.5, fhan_Input->h); //? 线性� �� ��?? ? � � ?�� � � ? ?�� � � 
	fhan_Input->fe1 = Fal_ADRC(fhan_Input->e, 0.25, fhan_Input->h);

	/*************?��??�� */
	fhan_Input->z1 += fhan_Input->h * (fhan_Input->z2 - fhan_Input->beta_01 * fhan_Input->e);
	fhan_Input->z2 += fhan_Input->h * (fhan_Input->z3
					  - fhan_Input->beta_02 * fhan_Input->fe
					  + fhan_Input->b * fhan_Input->u);
	//ESO估� �� �� ? ? �?��?��?� � ?�� �补?��?� � MEMS?�?�仪� 移�?大�?估� ��?产�  � �?
	fhan_Input->z3 += fhan_Input->h * (-fhan_Input->beta_03 * fhan_Input->fe1);
}


/************? */
/*
void Nolinear_Conbination_ADRC(Fhan_Data *fhan_Input)
{
  float d=0,a0=0,y=0,a1=0,a2=0,a=0;
  float Sy=0,Sa=0;//ADRC?�� � � � � �?
  fhan_Input->h1=fhan_Input->N1*fhan_Input->h;
  d=fhan_Input->r*fhan_Input->h1*fhan_Input->h1;
  a0=fhan_Input->h1*fhan_Input->c*fhan_Input->e2;
  y=fhan_Input->e1+a0;
  a1=sqrt(d*(d+8*ABS(y)));
  a2=a0+Sign_ADRC(y)*(a1-d)/2;
  Sy=Fsg_ADRC(y,d);
  a=(a0+y-a2)*Sy+a2;
  Sa=Fsg_ADRC(a,d);
  fhan_Input->u0=-fhan_Input->r*((a/d)-Sign_ADRC(a))*Sa-fhan_Input->r*Sign_ADRC(a);
  //a=(a0+y)*Fsg_ADRC(y,d)+a2*(1-Fsg_ADRC(y,d));
  //fhan_Input->fh=-fhan_Input->r*(a/d)*Fsg_ADRC(a,d)
  //                -fhan_Input->r*Sign_ADRC(a)*(1-Fsg_ADRC(a,d));//� ?�� �? � ? ? ? �??� ? 
}
*/
void Nolinear_Conbination_ADRC(Fhan_Data *fhan_Input)
{
	float temp_e2 = 0;
	temp_e2 = Constrain_Float(fhan_Input->e2, -3000, 3000);
	fhan_Input->u0 = fhan_Input->beta_1 * Fal_ADRC(fhan_Input->e1, fhan_Input->alpha1, fhan_Input->zeta)
					 + fhan_Input->beta_2 * Fal_ADRC(temp_e2, fhan_Input->alpha2, fhan_Input->zeta);

}


void ADRC_Control(Fhan_Data *fhan_Input, float expect_ADRC, float feedback_ADRC)
{
	/*? ? ?�� �� �� ��  1�?/
	/********
		**
		**
		**
		**
		**
	 ********/
	/*****
	 * � */
	Fhan_ADRC(fhan_Input, expect_ADRC);

	/*? ? ?�� �� �� ��  2�?/
	/********
			*
			*
	   ****
	 *
	 *
	 ********/
	/************系�?� */
	fhan_Input->y = feedback_ADRC;
	/*****
	 * ?��??�� */
	ESO_ADRC(fhan_Input); //� ? ? MEMS� 产�  � 移�??��??�� ��  z3此项� � 移�?? ? ? ?��  ?�� ��  � 解� ��?? ?�� ?3
	/*? ? ?�� �� �� ��  3�?/
	/********
		   **
		 **
	   **
		 **
		   **
	 ********/
	/********?�� */
	fhan_Input->e0 += fhan_Input->e1 * fhan_Input->h; //?�� � ? �?
	fhan_Input->e1 = fhan_Input->x1 - fhan_Input->z1; //?�� ? � �?
	fhan_Input->e2 = fhan_Input->x2 - fhan_Input->z2; //?�� � ? 项�?
	/********线性�?? */
	/*
	 fhan_Input->u0=//fhan_Input->beta_0*fhan_Input->e0
				   +fhan_Input->beta_1*fhan_Input->e1
				   +fhan_Input->beta_2*fhan_Input->e2;
	*/
	Nolinear_Conbination_ADRC(fhan_Input);
	/**********?�� */
	//fhan_Input->u=fhan_Input->u0
	//             -fhan_Input->z3/fhan_Input->b0;
	//?��?MEMS� ? ?��?移�  � 严�  � � beta_03? ?��  � 大� ��??�� �� ?3� 移�  � 大�?? ? � ? ?�� �� �补?�� �� ��  
	fhan_Input->u = Constrain_Float(fhan_Input->u0, -200, 200);
}
#endif
