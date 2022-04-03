%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%                    �����������
%
%  �������:
%  T          ���沽��
%  Fb         ����ϵ���ٶȼ���� ����λ����/��/�룩
%  attiN      ��������������򣨵�λ���ȣ�
%  veloN      �ɻ��˶��ٶȡ���X����Y����Z���򣨵�λ����/�룩
%  posiN      ���ȡ�γ�ȡ��߶ȣ���λ���ȡ��ȡ��ף�
%  ���������
%  veloN      �ɻ��˶��ٶȡ���X����Y����Z���򣨵�λ����/�룩
%      
%                           ������ƣ�����  ���ڣ�2002/8/9
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [veloN]=velo_cal(T,Fb,attiN,veloN,posiN)

  Re=6378137.0;       %����뾶���ף� 
  f=1/298.257;        %�������Բ��
  Wie=7.292115147e-5; %������ת���ٶ�
  g=9.7803698;        %�������ٶ�

  long=posiN(1,1)*pi/180.0;lati=posiN(2,1)*pi/180.0;heig=posiN(3,1);
    %������λ��

  Rm=Re*(1-2*f+3*f*sin(lati)*sin(lati));
  Rn=Re*(1+f*sin(lati)*sin(lati));
    %�������ʰ뾶���

  roll=attiN(1,1)*pi/180.0;pitch=attiN(2,1)*pi/180.0;head=attiN(3,1)*pi/180.0;

  %����ϵN-->B
  Cbn=[cos(roll)*cos(head)+sin(roll)*sin(pitch)*sin(head), -cos(roll)*sin(head)+sin(roll)*sin(pitch)*cos(head), -sin(roll)*cos(pitch);
       cos(pitch)*sin(head),                               cos(pitch)*cos(head),                                sin(pitch);
       sin(roll)*cos(head)-cos(roll)*sin(pitch)*sin(head), -sin(roll)*sin(head)-cos(roll)*sin(pitch)*cos(head), cos(roll)*cos(pitch)];

  Fn=Cbn'*Fb;

  Ve=veloN(1,1);Vn=veloN(2,1);Vu=veloN(3,1);

  Wenn=[-Vn/(Rm+heig); Ve/(Rn+heig);  Ve/(Rn+heig)*tan(lati)];
  Wien=[0;             Wie*cos(lati); Wie*sin(lati)];
  
  %һ�������������΢�ַ���
  veloN=veloN+T*(Fn-cross((2*Wien+Wenn),veloN)+[0.0;0.0;-1.0*g]);





