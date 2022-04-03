%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%                    位置计算程序
%
%  输入参数:
%  T          仿真步长
%  veloN      飞机运动速度——X东向、Y北向、Z天向（单位：米/秒）
%  posiN      经度、纬度、高度（单位：度、度、米）
%  输出参数：
%  posiN      经度、纬度、高度（单位：度、度、米）
%      
%                           程序设计：熊智  日期：2002/4/15
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [posiN]=posi_cal(T,veloN,posiN)

  Re=6378137.0;  %地球半径（米） 
  f=1/298.257;   %地球的椭圆率

  long=posiN(1,1)*pi/180.0;lati=posiN(2,1)*pi/180.0;heig=posiN(3,1);
    %飞行器位置

  Rm=Re*(1-2*f+3*f*sin(lati)*sin(lati));
  Rn=Re*(1+f*sin(lati)*sin(lati));
    %地球曲率半径求解

  Ve=veloN(1,1);Vn=veloN(2,1);Vu=veloN(3,1);

  heig=heig+T*Vu;
  lati=lati+T*(Vn/(Rm+heig));
  long=long+T*(Ve/((Rn+heig)*cos(lati)));

  posiN(1,1)=long*180.0/pi;       %单位：度
  posiN(2,1)=lati*180.0/pi;       %单位：度
  posiN(3,1)=heig;





