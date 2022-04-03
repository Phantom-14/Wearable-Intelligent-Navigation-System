%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%                   卡尔曼滤波初始化程序(利用GPS进行位置速度组合)
%
%  输入参数:
%           posiN-初始的飞行器位置（经度（度）、纬度（度）、高度（米））； 
%           Xc-综合模型状态量；PK-协方差阵；
%           Xerr-状态估计量的误差值（记录所有时刻的）
%  输出参数：Xc-综合模型状态量；PK-协方差阵；Xerr-状态估计量的误差值（记录某一时刻的）         
%
%                           程序设计：熊智  日期：2003/10/04
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [Xc,PK,Xerr]=kalm_gps_init_15D(posiN,Xc,PK,Xerr)
 
  Re=6378137.0;                                      %地球半径（米） 
  f=1/298.257;                                        %地球的椭圆率
  Wie=7.292115147e-5;                          %地球自转角速度
  g=9.7803698;                                      %重力加速度

  long=posiN(1,1)*pi/180.0;lati=posiN(2,1)*pi/180.0;heig=posiN(3,1);
    %飞行器位置

  %地球曲率半径求解
  Rm=Re*(1-2*f+3*f*sin(lati)*sin(lati));
  Rn=Re*(1+f*sin(lati)*sin(lati));


    Xerr0=[3600.0*pi/(3600.0*180.0),3600.0*pi/(3600.0*180.0),3600.0*pi/(3600.0*180.0),0.6,0.6,0.6,3.0/(Rm+heig),3.0/(Rn+heig)/cos(lati),3.0, ...
     100.0*pi/(3600.0*180.0),100.0*pi/(3600.0*180.0),100.0*pi/(3600.0*180.0),(4e-3)*g,(4e-3)*g,(4e-3)*g];
      %估计误差初值（与IMU仿真同）单位：rad,rad,rad,m/s,m/s,m/s,rad,rad,m,rad/s,rad/s,rad/s,rad/s,rad/s,rad/s,m/s/s,m/s/s,m/s/s
          
    PK=diag((Xerr0.^2));
    Xc=zeros(15,1);
    
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
        %IMU的6个误差量
        


      
