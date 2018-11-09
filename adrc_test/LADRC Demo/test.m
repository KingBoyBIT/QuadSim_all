clear,clc,close all

ADRC_Unit=...
	[[300000 ,0.005 , 3,               300,      4000,      10000,     0.001,    0.002,   2.0,      0.0010,    5,    5,    0.8,   1.5,    50,    0];
	[300000 ,0.005 , 3,               300,      4000,      10000,     0.001,    0.002,   2.0,      0.0010,    5,    5,    0.8,   1.5,    50,    0];
	[300000 ,0.005 , 3,               300,      4000,      10000,     0.001,    0.002,   1.2,      0.0005,    5,    5,    0.8,   1.5,    50,    0]];
ADRC_Pitch_Controller.x1 = 0;
ADRC_Pitch_Controller.x2 = 0;
ADRC_Pitch_Controller.z1 = 0;
ADRC_Pitch_Controller.z2 = 0;
ADRC_Pitch_Controller.z3 = 0;
ADRC_Pitch_Controller.u = 0;
ADRC_Pitch_Controller.u0 = 0;
ADRC_Pitch_Controller.e0 = 0;
ADRC_Pitch_Controller.e1 = 0;
ADRC_Pitch_Controller.e2 = 0;
ADRC_Pitch_Controller.r=ADRC_Unit(1,1);
ADRC_Pitch_Controller.h=ADRC_Unit(1,2);
ADRC_Pitch_Controller.N0=(ADRC_Unit(1,3));
ADRC_Pitch_Controller.beta_01=ADRC_Unit(1,4);
ADRC_Pitch_Controller.beta_02=ADRC_Unit(1,5);
ADRC_Pitch_Controller.beta_03=ADRC_Unit(1,6);
ADRC_Pitch_Controller.b0=ADRC_Unit(1,7);
ADRC_Pitch_Controller.beta_0=ADRC_Unit(1,8);
ADRC_Pitch_Controller.beta_1=ADRC_Unit(1,9);
ADRC_Pitch_Controller.beta_2=ADRC_Unit(1,10);
ADRC_Pitch_Controller.N1=(ADRC_Unit(1,11));
ADRC_Pitch_Controller.c=ADRC_Unit(1,12);
ADRC_Pitch_Controller.alpha1=ADRC_Unit(1,13);
ADRC_Pitch_Controller.alpha2=ADRC_Unit(1,14);
ADRC_Pitch_Controller.zeta=ADRC_Unit(1,15);
ADRC_Pitch_Controller.b=ADRC_Unit(1,16);

ADRC_Roll_Controller.r=ADRC_Unit(2,1);
ADRC_Roll_Controller.h=ADRC_Unit(2,2);
ADRC_Roll_Controller.N0=(ADRC_Unit(2,3));
ADRC_Roll_Controller.beta_01=ADRC_Unit(2,4);
ADRC_Roll_Controller.beta_02=ADRC_Unit(2,5);
ADRC_Roll_Controller.beta_03=ADRC_Unit(2,6);
ADRC_Roll_Controller.b0=ADRC_Unit(2,7);
ADRC_Roll_Controller.beta_0=ADRC_Unit(2,8);
ADRC_Roll_Controller.beta_1=ADRC_Unit(2,9);
ADRC_Roll_Controller.beta_2=ADRC_Unit(2,10);
ADRC_Roll_Controller.N1=(ADRC_Unit(2,11));
ADRC_Roll_Controller.c=ADRC_Unit(2,12);
ADRC_Roll_Controller.alpha1=ADRC_Unit(2,13);
ADRC_Roll_Controller.alpha2=ADRC_Unit(2,14);
ADRC_Roll_Controller.zeta=ADRC_Unit(2,15);
ADRC_Roll_Controller.b=ADRC_Unit(2,16);

out = ADRC_Control(ADRC_Pitch_Controller,10/360*pi,3)


% /*�߽�Լ��*/
function out = Constrain_Float(amt, low, high)
if(amt>high)
	out = high;
else
	if amt<low
		out = low;
	else
		out = amt;
	end
end

end

function output = Sign_ADRC(Input)
if(Input>1E-6)
	output=1;
elseif(Input<-1E-6)
	output=-1;
else
	output=0;
end
end
function output = Fsg_ADRC( x, d)
output=(Sign_ADRC(x+d)-Sign_ADRC(x-d))/2;
end


% %ADRC���ٸ���΢����TD���Ľ����㷨fhan
function fhan_Output = Fhan_ADRC(fhan_Input,expect_ADRC)%����ADRC���ȹ���

% d=0,a0=0,y=0,a1=0,a2=0,a=0;
% x1_delta=0;%ADRC״̬���������
x1_delta=fhan_Input.x1-expect_ADRC;%��x1-v(k)���x1�õ���ɢ���¹�ʽ
fhan_Input.h0=fhan_Input.N0*fhan_Input.h;%��h0���h��������ٸ���΢�����ٶȳ�������
d=fhan_Input.r*fhan_Input.h0*fhan_Input.h0;%d=rh^2;
a0=fhan_Input.h0*fhan_Input.x2;%a0=h*x2
y=x1_delta+a0;%y=x1+a0
a1=sqrt(d*(d+8*abs(y)));%a1=sqrt(d*(d+8*ABS(y))])
a2=a0+Sign_ADRC(y)*(a1-d)/2;%a2=a0+sign(y)*(a1-d)/2;
a=(a0+y)*Fsg_ADRC(y,d)+a2*(1-Fsg_ADRC(y,d));
fhan_Input.fh=-fhan_Input.r*(a/d)*Fsg_ADRC(a,d)...
	-fhan_Input.r*Sign_ADRC(a)*(1-Fsg_ADRC(a,d));%�õ�����΢�ּ��ٶȸ�����
fhan_Input.x1=fhan_Input.x1+fhan_Input.h*fhan_Input.x2;%�������ٸ���״̬��x1
fhan_Input.x2=fhan_Input.x2+fhan_Input.h*fhan_Input.fh;%�������ٸ���״̬��΢��x2

fhan_Output = fhan_Input;
end

% %ԭ�㸽���������Զε������ݴκ���
function fal_output = Fal_ADRC(e,alpha,zeta)
s=(Sign_ADRC(e+zeta)-Sign_ADRC(e-zeta))/2;
fal_output=e*s/(zeta^(1-alpha))+(abs(e)^alpha)*Sign_ADRC(e)*(1-s);
end



% /************����״̬�۲���********************/
% %״̬�۲�������beta01=1/h  beta02=1/(3*h^2)  beta03=2/(8^2*h^3) ...
function fhan_Output = ESO_ADRC(fhan_Input)
fhan_Input.e=fhan_Input.z1-fhan_Input.y;%״̬���

fhan_Input.fe=Fal_ADRC(fhan_Input.e,0.5,fhan_Input.h);%�����Ժ�������ȡ����״̬�뵱ǰ״̬���
fhan_Input.fe1=Fal_ADRC(fhan_Input.e,0.25,fhan_Input.h);

%   /*************��չ״̬������**********/
fhan_Input.z1=fhan_Input.z1+fhan_Input.h*(fhan_Input.z2-fhan_Input.beta_01*fhan_Input.e);
fhan_Input.z2=fhan_Input.z2+fhan_Input.h*(fhan_Input.z3...
	-fhan_Input.beta_02*fhan_Input.fe...
	+fhan_Input.b*fhan_Input.u);
%ESO����״̬���ٶ��źţ������Ŷ���������ͳMEMS������Ư�ƽϴ󣬹��ƻ����Ư��
fhan_Input.z3=fhan_Input.z3+fhan_Input.h*(-fhan_Input.beta_03*fhan_Input.fe1);
fhan_Output = fhan_Input;
end

% /************���������****************/
function output = Nolinear_Conbination_ADRC(fhan_Input)
temp_e2=Constrain_Float(fhan_Input.e2,-3000,3000);
fhan_Input.u0=fhan_Input.beta_1*Fal_ADRC(fhan_Input.e1,fhan_Input.alpha1,fhan_Input.zeta)...
	+fhan_Input.beta_2*Fal_ADRC(temp_e2,fhan_Input.alpha2,fhan_Input.zeta);
output = fhan_Input;
end

function fhan_Output = ADRC_Control(fhan_Input,expect_ADRC,feedback_ADRC)
%     /*�Կ��ſ�������1��*/
%       ���Ź��ȹ��̣�����Ϊ����������
%       ��TD����΢�����õ���
%       ���������ź�x1����������΢���ź�x2
      Fhan_ADRC(fhan_Input,expect_ADRC);
%     /*�Կ��ſ�������2��*/
%       /************ϵͳ���ֵΪ��������״̬������ESO����״̬�۲���������*********/
      fhan_Input.y=feedback_ADRC;
%       /*****
%       ����״̬�۲������õ������źŵ�����״̬��
%       1��״̬�ź�z1��
%       2��״̬�ٶ��ź�z2��
%       3��״̬���ٶ��ź�z3��
%       ����z1��z2������Ϊ״̬������TD΢�ָ������õ���x1,x2�����
%       ���������Ժ���ӳ�䣬����betaϵ����
%       ��ϵõ�δ����״̬���ٶȹ����Ŷ�������ԭʼ������u
%       *********/
      ESO_ADRC(fhan_Input);%�ͳɱ�MEMS�����Ư�ƣ���չ������z3�����Ư�ƣ�Ŀǰ��ʱδ�뵽�취�����δ�õ�z3
%     /*�Կ��ſ�������3��*/
%       /********״̬������***/
      fhan_Input.e0=fhan_Input.e0+fhan_Input.e1*fhan_Input.h;%״̬������
      fhan_Input.e1=fhan_Input.x1-fhan_Input.z1;%״̬ƫ����
      fhan_Input.e2=fhan_Input.x2-fhan_Input.z2;%״̬΢���
%       /********�������*******/
      Nolinear_Conbination_ADRC(fhan_Input);
%       /**********�Ŷ�����*******/
%       fhan_Input.u=fhan_Input.u0
%                   -fhan_Input.z3/fhan_Input.b0;
      %����MEMS������Ư�ƱȽ����أ���beta_03ȡֵ�Ƚϴ�ʱ����ʱ��z3Ư�ƱȽϴ�Ŀǰ�������Ŷ�����������
      fhan_Input.u=Constrain_Float(fhan_Input.u0,-200,200);
	  fhan_Output = fhan_Input;
end
