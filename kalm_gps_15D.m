%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%                   卡尔曼滤波程序(利用GPS进行位置速度组合)
%
%  输入参数:t-仿真时间，T_D-离散周期；
%           Fb-加速度计输出；
%           attiN-导航计算的姿态角度横滚，俯仰，航向（度，度，度）；
%           veloN-飞行器相对机体系的运动速度东向、北向、天向（米/秒）；
%           posiG-GPS输出的飞行器位置（经度（度）、纬度（度）、高度（米））； 
%           veloG-GPS输出的飞行器速度（东向（米/秒），北向（米/秒），天向（米/秒））
%           Xc-综合模型状态量；PK-协方差阵；
%           Xerr-状态估计量的误差值（记录所有时刻的）
%           kflag-GPS信息有效标志位（1－有效）
%  输出参数：Xc-综合模型状态量；PK-协方差阵；Xerr-状态估计量的误差值（记录某一时刻的）         
%
%                           程序设计：熊智  日期：2003/9/29
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [Xc,PK,Xerr]=kalm_gps(t,T_D,Fb,attiN,veloN,posiN,posiG,veloG,Xc,PK,Xerr,kflag)
 
  Re=6378137.0;                                      %地球半径（米） 
  f=1/298.257;                                        %地球的椭圆率
  Wie=7.292115147e-5;                          %地球自转角速度
  g=9.7803698;                                      %重力加速度

  long=posiN(1,1)*pi/180.0;lati=posiN(2,1)*pi/180.0;heig=posiN(3,1);
    %飞行器位置

  %地球曲率半径求解
  Rm=Re*(1-2*f+3*f*sin(lati)*sin(lati));
  Rn=Re*(1+f*sin(lati)*sin(lati));
      
  roll=attiN(1,1)*pi/180.0;pitch=attiN(2,1)*pi/180.0;head=attiN(3,1)*pi/180.0;
    
  Cbn=[cos(roll)*cos(head)+sin(roll)*sin(pitch)*sin(head), -cos(roll)*sin(head)+sin(roll)*sin(pitch)*cos(head), -sin(roll)*cos(pitch);
       cos(pitch)*sin(head),                               cos(pitch)*cos(head),                                sin(pitch);
       sin(roll)*cos(head)-cos(roll)*sin(pitch)*sin(head), -sin(roll)*sin(head)-cos(roll)*sin(pitch)*cos(head), cos(roll)*cos(pitch)];
  %坐标系N-->B
  
  Fn=Cbn'*Fb;

  Ve=veloN(1,1);Vn=veloN(2,1);Vu=veloN(3,1);

  %综合系统模型
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
    %陀螺和加速度计的一阶马尔可夫相关时间（与IMU仿真同）

  %FM=diag([0,0,0,-1.0/Tgx,-1.0/Tgy,-1.0/Tgz,-1.0/Tax,-1.0/Tay,-1.0/Taz]);
  FM=diag([0,0,0,0.0,0.0,0.0]);

  
  %disp('连续的A阵');
  
  FI=[FN,        FS;
      zeros(6,9),FM];
    %系统矩阵
   
 %disp('连续的B阵');
    
    GI=[Cbn', zeros(3,3);
      zeros(3,3), Cbn';
      zeros(9,6)];
    %量测矩阵
    
  I=eye(size(FI));
  
  %disp('离散的A阵');
  FL=I+FI*T_D+FI*FI/2.0*T_D*T_D;
  
  %disp('离散的B阵');
  GL=(I+FI/2.0*T_D+FI*FI/6.0*T_D*T_D)*GI*T_D;
    %离散化综合模型
  
   

    
    
    % W=[0.01*pi/(3600*180),0.01*pi/(3600*180),0.01*pi/(3600*180), ...
  %   sqrt(2*T_D/Tgx)*0.01*pi/(3600*180),sqrt(2*T_D/Tgy)*0.01*pi/(3600*180),sqrt(2*T_D/Tgz)*0.01*pi/(3600*180), ...
   %  sqrt(2*T_D/Tax)*(1e-4)*g,sqrt(2*T_D/Tay)*(1e-4)*g,sqrt(2*T_D/Taz)*(1e-4)*g]';  
      
 W=[20.01*pi/(3600*180),20.01*pi/(3600*180),20.01*pi/(3600*180), ...
    8e-4*g,8e-4*g,8e-4*g]';  
 
 
 %系统噪声阵（与IMU仿真同）单位：rad/s,rad/s,rad/s,rad/s,rad/s,rad/s,m/s/s,m/s/s,m/s/s

  %GPS/INS位置量测矩阵
%   HG=[zeros(3,6),diag([Rm,Rn*cos(lati),1]),zeros(3,6)];
%   VG=[0.01;0.01;0.01];  % 需要与GPS仿真精度相同

%   %GPS/INS位置＋速度量测方程
%   HG=[HG;zeros(3,3),diag([1,1,1]),zeros(3,9)];
%   VG=[VG;0.001;0.001;0.001]; % 需要与GPS仿真精度相同

HG=[zeros(3,3),diag([1,1,1]),zeros(3,9)];
VG=[0.001;0.001;0.001]; % 需要与GPS仿真精度相同 


  Q=diag((W.^2)');
  RG=diag((VG.^2)');
  
  PK=FL*PK*FL'+GL*Q*GL';  
    %系统状态量误差方差计算
    
%   FI_total=[[1:18];FI];
%   save FI.dat  FI_total -ASCII;
    
  if( kflag == 1 )
    KK=PK*HG'*inv(HG*PK*HG'+RG);
    PK=(I-KK*HG)*PK*(I-KK*HG)'+KK*RG*KK';
    
    %修正量计算    
    
%     Yc=[(posiN(2,1)-posiG(2,1))*pi/180.0*(Rm+heig);
%         (posiN(1,1)-posiG(1,1))*pi/180.0*(Rn+heig)*cos(lati);
%         posiN(3,1)-posiG(3,1)]; %量测次序为纬度、经度、高度
%     Yc=[Yc;veloN-veloG]; %单位（米，米/秒）

 Yc=[veloN-veloG]; %单位（米，米/秒 )   
    Xc=FL*Xc;
    Xc=Xc+KK*(Yc-HG*Xc);    
  
  end

  %%%%%%%%%%%%%%%%%滤波估计精度%%%%%%%%%%%%
  Xerr(1,1)=sqrt(PK(1,1))*180.0*3600.0/pi;    %sec
  Xerr(1,2)=sqrt(PK(2,2))*180.0*3600.0/pi;    %sec
  Xerr(1,3)=sqrt(PK(3,3))*180.0*3600.0/pi;    %sec
  Xerr(1,4)=sqrt(PK(4,4));                    %m/s
  Xerr(1,5)=sqrt(PK(5,5));                    %m/s
  Xerr(1,6)=sqrt(PK(6,6));                    %m/s
  Xerr(1,7)=sqrt(PK(7,7))*(Rm+heig);          %m
  Xerr(1,8)=sqrt(PK(8,8))*(Rn+heig)*cos(lati);%m
  Xerr(1,9)=sqrt(PK(9,9));                    %m
       %INS的9个导航量误差
         
  Xerr(1,10)=sqrt(PK(10,10))*180.0*3600.0/pi;   %deg/h
  Xerr(1,11)=sqrt(PK(11,11))*180.0*3600.0/pi;   %deg/h
  Xerr(1,12)=sqrt(PK(12,12))*180.0*3600.0/pi;   %deg/h
  
  Xerr(1,13)=sqrt(PK(13,13))/g;                 %g
  Xerr(1,14)=sqrt(PK(14,14))/g;                 %g
  Xerr(1,15)=sqrt(PK(15,15))/g;                 %g
      %IMU的9个误差量



