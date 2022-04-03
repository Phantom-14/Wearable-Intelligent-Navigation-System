%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%                   �������˲�����(����GPS����λ���ٶ����)
%
%  �������:t-����ʱ�䣬T_D-��ɢ���ڣ�
%           Fb-���ٶȼ������
%           attiN-�����������̬�ǶȺ�������������򣨶ȣ��ȣ��ȣ���
%           veloN-��������Ի���ϵ���˶��ٶȶ��򡢱���������/�룩��
%           posiG-GPS����ķ�����λ�ã����ȣ��ȣ���γ�ȣ��ȣ����߶ȣ��ף����� 
%           veloG-GPS����ķ������ٶȣ�������/�룩��������/�룩��������/�룩��
%           Xc-�ۺ�ģ��״̬����PK-Э������
%           Xerr-״̬�����������ֵ����¼����ʱ�̵ģ�
%           kflag-GPS��Ϣ��Ч��־λ��1����Ч��
%  ���������Xc-�ۺ�ģ��״̬����PK-Э������Xerr-״̬�����������ֵ����¼ĳһʱ�̵ģ�         
%
%                           ������ƣ�����  ���ڣ�2003/9/29
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [Xc,PK,Xerr]=kalm_gps(t,T_D,Fb,attiN,veloN,posiN,posiG,veloG,Xc,PK,Xerr,kflag)
 
  Re=6378137.0;                                      %����뾶���ף� 
  f=1/298.257;                                        %�������Բ��
  Wie=7.292115147e-5;                          %������ת���ٶ�
  g=9.7803698;                                      %�������ٶ�

  long=posiN(1,1)*pi/180.0;lati=posiN(2,1)*pi/180.0;heig=posiN(3,1);
    %������λ��

  %�������ʰ뾶���
  Rm=Re*(1-2*f+3*f*sin(lati)*sin(lati));
  Rn=Re*(1+f*sin(lati)*sin(lati));
      
  roll=attiN(1,1)*pi/180.0;pitch=attiN(2,1)*pi/180.0;head=attiN(3,1)*pi/180.0;
    
  Cbn=[cos(roll)*cos(head)+sin(roll)*sin(pitch)*sin(head), -cos(roll)*sin(head)+sin(roll)*sin(pitch)*cos(head), -sin(roll)*cos(pitch);
       cos(pitch)*sin(head),                               cos(pitch)*cos(head),                                sin(pitch);
       sin(roll)*cos(head)-cos(roll)*sin(pitch)*sin(head), -sin(roll)*sin(head)-cos(roll)*sin(pitch)*cos(head), cos(roll)*cos(pitch)];
  %����ϵN-->B
  
  Fn=Cbn'*Fb;

  Ve=veloN(1,1);Vn=veloN(2,1);Vu=veloN(3,1);

  %�ۺ�ϵͳģ��
  FN=zeros(9,9);
  FN(1,2)=Wie*sin(lati)+Ve/(Rn+heig)*tan(lati);
  FN(1,3)=-(Wie*cos(lati)+Ve/(Rn+heig));
  FN(1,5)=-1/(Rm+heig);
  FN(2,1)=-(Wie*sin(lati)+Ve/(Rn+heig)*tan(lati));
  FN(2,3)=-Vn/(Rm+heig);
  FN(2,4)=1/(Rn+heig);
  FN(2,7)=-Wie*sin(lati);
  FN(3,1)=Wie*cos(lati)+Ve/(Rn+heig);
  FN(3,2)=Vn/(Rm+heig);
  FN(3,4)=1/(Rn+heig)*tan(lati);
  FN(3,7)=Wie*cos(lati)+Ve/(Rn+heig)*sec(lati)*sec(lati);
  FN(4,2)=-Fn(3,1);
  FN(4,3)=Fn(2,1);
  FN(4,4)=Vn/(Rm+heig)*tan(lati)-Vu/(Rm+heig);
  FN(4,5)=2*Wie*sin(lati)+Ve/(Rn+heig)*tan(lati);
  FN(4,6)=-(2*Wie*cos(lati)+Ve/(Rn+heig));
  FN(4,7)=2*Wie*cos(lati)*Vn+Ve*Vn/(Rn+heig)*sec(lati)*sec(lati)+2*Wie*sin(lati)*Vu;
  FN(5,1)=Fn(3,1);
  FN(5,3)=-Fn(1,1);
  FN(5,4)=-2*(Wie*sin(lati)+Ve/(Rn+heig)*tan(lati));
  FN(5,5)=-Vu/(Rm+heig);
  FN(5,6)=-Vn/(Rm+heig);
  FN(5,7)=-(2*Wie*cos(lati)+Ve/(Rn+heig)*sec(lati)*sec(lati))*Ve;
  FN(6,1)=-Fn(2,1);
  FN(6,2)=Fn(1,1);
  FN(6,4)=2*(Wie*cos(lati)+Ve/(Rn+heig));
  FN(6,5)=2*Vn/(Rm+heig);
  FN(6,7)=-2*Ve*Wie*sin(lati);
  FN(7,5)=1/(Rm+heig);
  FN(8,4)=sec(lati)/(Rn+heig);
  FN(8,7)=Ve/(Rn+heig)*sec(lati)*tan(lati);
  FN(9,6)=1;

  FS=[-Cbn',zeros(3,3);
      zeros(3,3),Cbn';
      zeros(3,6)];
  
  Tgx=3600.0; Tgy=3600.0;  Tgz=3600.0; 
  Tax=1800.0; Tay=1800.0;  Taz=1800.0; 
    %���ݺͼ��ٶȼƵ�һ������ɷ����ʱ�䣨��IMU����ͬ��

  %FM=diag([0,0,0,-1.0/Tgx,-1.0/Tgy,-1.0/Tgz,-1.0/Tax,-1.0/Tay,-1.0/Taz]);
  FM=diag([0,0,0,0.0,0.0,0.0]);

  
  %disp('������A��');
  
  FI=[FN,        FS;
      zeros(6,9),FM];
    %ϵͳ����
   
 %disp('������B��');
    
    GI=[Cbn', zeros(3,3);
      zeros(3,3), Cbn';
      zeros(9,6)];
    %�������
    
  I=eye(size(FI));
  
  %disp('��ɢ��A��');
  FL=I+FI*T_D+FI*FI/2.0*T_D*T_D;
  
  %disp('��ɢ��B��');
  GL=(I+FI/2.0*T_D+FI*FI/6.0*T_D*T_D)*GI*T_D;
    %��ɢ���ۺ�ģ��
  
   

    
    
    % W=[0.01*pi/(3600*180),0.01*pi/(3600*180),0.01*pi/(3600*180), ...
  %   sqrt(2*T_D/Tgx)*0.01*pi/(3600*180),sqrt(2*T_D/Tgy)*0.01*pi/(3600*180),sqrt(2*T_D/Tgz)*0.01*pi/(3600*180), ...
   %  sqrt(2*T_D/Tax)*(1e-4)*g,sqrt(2*T_D/Tay)*(1e-4)*g,sqrt(2*T_D/Taz)*(1e-4)*g]';  
      
 W=[20.01*pi/(3600*180),20.01*pi/(3600*180),20.01*pi/(3600*180), ...
    8e-4*g,8e-4*g,8e-4*g]';  
 
 
 %ϵͳ��������IMU����ͬ����λ��rad/s,rad/s,rad/s,rad/s,rad/s,rad/s,m/s/s,m/s/s,m/s/s

  %GPS/INSλ���������
%   HG=[zeros(3,6),diag([Rm,Rn*cos(lati),1]),zeros(3,6)];
%   VG=[0.01;0.01;0.01];  % ��Ҫ��GPS���澫����ͬ

%   %GPS/INSλ�ã��ٶ����ⷽ��
%   HG=[HG;zeros(3,3),diag([1,1,1]),zeros(3,9)];
%   VG=[VG;0.001;0.001;0.001]; % ��Ҫ��GPS���澫����ͬ

HG=[zeros(3,3),diag([1,1,1]),zeros(3,9)];
VG=[0.001;0.001;0.001]; % ��Ҫ��GPS���澫����ͬ 


  Q=diag((W.^2)');
  RG=diag((VG.^2)');
  
  PK=FL*PK*FL'+GL*Q*GL';  
    %ϵͳ״̬���������
    
%   FI_total=[[1:18];FI];
%   save FI.dat  FI_total -ASCII;
    
  if( kflag == 1 )
    KK=PK*HG'*inv(HG*PK*HG'+RG);
    PK=(I-KK*HG)*PK*(I-KK*HG)'+KK*RG*KK';
    
    %����������    
    
%     Yc=[(posiN(2,1)-posiG(2,1))*pi/180.0*(Rm+heig);
%         (posiN(1,1)-posiG(1,1))*pi/180.0*(Rn+heig)*cos(lati);
%         posiN(3,1)-posiG(3,1)]; %�������Ϊγ�ȡ����ȡ��߶�
%     Yc=[Yc;veloN-veloG]; %��λ���ף���/�룩

 Yc=[veloN-veloG]; %��λ���ף���/�� )   
    Xc=FL*Xc;
    Xc=Xc+KK*(Yc-HG*Xc);    
  
  end

  %%%%%%%%%%%%%%%%%�˲����ƾ���%%%%%%%%%%%%
  Xerr(1,1)=sqrt(PK(1,1))*180.0*3600.0/pi;    %sec
  Xerr(1,2)=sqrt(PK(2,2))*180.0*3600.0/pi;    %sec
  Xerr(1,3)=sqrt(PK(3,3))*180.0*3600.0/pi;    %sec
  Xerr(1,4)=sqrt(PK(4,4));                    %m/s
  Xerr(1,5)=sqrt(PK(5,5));                    %m/s
  Xerr(1,6)=sqrt(PK(6,6));                    %m/s
  Xerr(1,7)=sqrt(PK(7,7))*(Rm+heig);          %m
  Xerr(1,8)=sqrt(PK(8,8))*(Rn+heig)*cos(lati);%m
  Xerr(1,9)=sqrt(PK(9,9));                    %m
       %INS��9�����������
         
  Xerr(1,10)=sqrt(PK(10,10))*180.0*3600.0/pi;   %deg/h
  Xerr(1,11)=sqrt(PK(11,11))*180.0*3600.0/pi;   %deg/h
  Xerr(1,12)=sqrt(PK(12,12))*180.0*3600.0/pi;   %deg/h
  
  Xerr(1,13)=sqrt(PK(13,13))/g;                 %g
  Xerr(1,14)=sqrt(PK(14,14))/g;                 %g
  Xerr(1,15)=sqrt(PK(15,15))/g;                 %g
      %IMU��9�������



