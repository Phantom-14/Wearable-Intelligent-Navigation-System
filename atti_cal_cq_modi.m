%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%                    ��̬����������Ԫ��������Чת��ʸ��������
%
%  T          ���沽�����룩
%  Wibb       ����ϵ���������   ����λ����/�룩
%  attiN      ��������������򣨵�λ���ȣ�
%  veloN      �ɻ��˶��ٶȡ���X����Y����Z���򣨵�λ����/�룩
%  posiN      ���ȡ�γ�ȡ��߶ȣ���λ���ȡ��ȡ��ף�
%  WnbbA_old  ���ٶȻ����������λ�����ȣ�
%
%  ���������
%  attiN      ��������������򣨵�λ���ȣ�  
%  WnbbA_old  ���ٶȻ����������λ�����ȣ�
%      
%                           ������ƣ�����  ���ڣ�2002/8/9
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [attiN,WnbbA_old]=atti_cal_cq_modi(T,Wibb,attiN,veloN,posiN,WnbbA_old)

  Re=6378137.0;       %����뾶���ף� 
  f=1/298.257;        %�������Բ��
  Wie=7.292115147e-5; %������ת���ٶ�

  long=posiN(1,1)*pi/180.0;lati=posiN(2,1)*pi/180.0;heig=posiN(3,1);
    %������λ��

  Rm=Re*(1-2*f+3*f*sin(lati)*sin(lati));
  Rn=Re*(1+f*sin(lati)*sin(lati));
    %�������ʰ뾶���

  roll=attiN(1,1)*pi/180.0;pitch=attiN(2,1)*pi/180.0;head=attiN(3,1)*pi/180.0;

  Cbn=[cos(roll)*cos(head)+sin(roll)*sin(pitch)*sin(head), -cos(roll)*sin(head)+sin(roll)*sin(pitch)*cos(head), -sin(roll)*cos(pitch);
       cos(pitch)*sin(head),                               cos(pitch)*cos(head),                                sin(pitch);
       sin(roll)*cos(head)-cos(roll)*sin(pitch)*sin(head), -sin(roll)*sin(head)-cos(roll)*sin(pitch)*cos(head), cos(roll)*cos(pitch)];
    %����ϵN-->B

  Ve=veloN(1,1);Vn=veloN(2,1);Vu=veloN(3,1);

  Wenn=[-Vn/(Rm+heig); Ve/(Rn+heig);  Ve/(Rn+heig)*tan(lati)];
  Wien=[0;             Wie*cos(lati); Wie*sin(lati)];
    %��λ������/��

  Wnbb=Wibb*pi/180.0-Cbn*(Wien+Wenn);  %��λ������/��

  %%%%%%%%%%%%%%��Ԫ��������̬����%%%%%%%%%%%%%%%
  Q=[cos(head/2)*cos(pitch/2)*cos(roll/2)+sin(head/2)*sin(pitch/2)*sin(roll/2);
     cos(head/2)*sin(pitch/2)*cos(roll/2)+sin(head/2)*cos(pitch/2)*sin(roll/2);
     cos(head/2)*cos(pitch/2)*sin(roll/2)-sin(head/2)*sin(pitch/2)*cos(roll/2);
     -1.0*sin(head/2)*cos(pitch/2)*cos(roll/2)+cos(head/2)*sin(pitch/2)*sin(roll/2)];
 
  WnbbA=Wnbb*T;
  WnbbA0=sqrt(WnbbA(1,1)^2+WnbbA(2,1)^2+WnbbA(3,1)^2);
  
  WnbbX=[0,          -WnbbA(1,1), -WnbbA(2,1), -WnbbA(3,1);
         WnbbA(1,1),  0,           WnbbA(3,1), -WnbbA(2,1);
         WnbbA(2,1), -WnbbA(3,1),   0,          WnbbA(1,1);
         WnbbA(3,1),  WnbbA(2,1),  -WnbbA(1,1),   0         ];

  c_q=cos(WnbbA0/2);
  if( WnbbA0<=1.0e-15 ) d_q=0.5; else  d_q=sin(WnbbA0/2)/WnbbA0;  end

  %%%%%%%%%%%��Чת��ʸ�������㷨%%%%%%%%%%%%%%%
  WnbbA_e=cross(WnbbA_old,WnbbA);
  WnbbX_e=[0,            -WnbbA_e(1,1), -WnbbA_e(2,1), -WnbbA_e(3,1);
           WnbbA_e(1,1),  0,             WnbbA_e(3,1), -WnbbA_e(2,1);
           WnbbA_e(2,1), -WnbbA_e(3,1),    0,           WnbbA_e(1,1);
           WnbbA_e(3,1),  WnbbA_e(2,1),  -WnbbA_e(1,1),   0         ];

  
  Q=( c_q*eye(4)+d_q*(WnbbX+1/12*WnbbX_e) )*Q;%�����㷨
  
  WnbbA_old=WnbbA;

  %%%%%%%%%%��Ԫ���淶��%%%%%%%%%
  tmp_Q=sqrt(Q(1,1)^2+Q(2,1)^2+Q(3,1)^2+Q(4,1)^2);
  for kc=1:4
    Q(kc,1)=Q(kc,1)/tmp_Q;    
  end
  
  %%%%%%%%%%��ȡ��̬����%%%%%%%%%
  Cbn=[Q(2,1)^2+Q(1,1)^2-Q(4,1)^2-Q(3,1)^2, 2*(Q(2,1)*Q(3,1)+Q(1,1)*Q(4,1)), 2*(Q(2,1)*Q(4,1)-Q(1,1)*Q(3,1));
       2*(Q(2,1)*Q(3,1)-Q(1,1)*Q(4,1)), Q(3,1)^2-Q(4,1)^2+Q(1,1)^2-Q(2,1)^2,  2*(Q(3,1)*Q(4,1)+Q(1,1)*Q(2,1));
       2*(Q(2,1)*Q(4,1)+Q(1,1)*Q(3,1)), 2*(Q(3,1)*Q(4,1)-Q(1,1)*Q(2,1)), Q(4,1)^2-Q(3,1)^2-Q(2,1)^2+Q(1,1)^2]; 
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %����̬(���������������
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  attiN(1,1)=atan(-Cbn(1,3)/Cbn(3,3));
  attiN(2,1)=atan(Cbn(2,3)/sqrt(Cbn(2,1)*Cbn(2,1)+Cbn(2,2)*Cbn(2,2)));
  attiN(3,1)=atan(Cbn(2,1)/Cbn(2,2));
    %��λ������

  %�����ж�
  attiN(1,1)=attiN(1,1)*180.0/pi;
  attiN(2,1)=attiN(2,1)*180.0/pi;
  attiN(3,1)=attiN(3,1)*180.0/pi;
    % ��λ����

  if(Cbn(2,2)<0 ) 
   attiN(3,1)=180.0+attiN(3,1);
  else 
   if(Cbn(2,1)<0) attiN(3,1)=360.0+attiN(3,1); end
  end
    %����Ƕȣ���λ���ȣ�

  if(Cbn(3,3)<0)
   if(Cbn(1,3)>0) attiN(1,1)=180.0-attiN(1,1); end
   if(Cbn(1,3)<0) attiN(1,1)=-(180.0+attiN(1,1)); end
  end
    %����Ƕȣ���λ���ȣ�


