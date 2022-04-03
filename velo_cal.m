%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%                    比力计算程序
%
%  输入参数:
%  T          仿真步长
%  Fb         机体系加速度计输出 （单位：米/秒/秒）
%  attiN      横滚、俯仰、航向（单位：度）
%  veloN      飞机运动速度——X东向、Y北向、Z天向（单位：米/秒）
%  posiN      经度、纬度、高度（单位：度、度、米）
%  输出参数：
%  veloN      飞机运动速度——X东向、Y北向、Z天向（单位：米/秒）
%      
%                           程序设计：熊智  日期：2002/8/9
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [veloN]=velo_cal(T,Fb,attiN,veloN,posiN)

  Re=6378137.0;       %地球半径（米） 
  f=1/298.257;        %地球的椭圆率
  Wie=7.292115147e-5; %地球自转角速度
  g=9.7803698;        %重力加速度

  long=posiN(1,1)*pi/180.0;lati=posiN(2,1)*pi/180.0;heig=posiN(3,1);
    %飞行器位置

  Rm=Re*(1-2*f+3*f*sin(lati)*sin(lati));
  Rn=Re*(1+f*sin(lati)*sin(lati));
    %地球曲率半径求解

  roll=attiN(1,1)*pi/180.0;pitch=attiN(2,1)*pi/180.0;head=attiN(3,1)*pi/180.0;

  %坐标系N-->B
  Cbn=[cos(roll)*cos(head)+sin(roll)*sin(pitch)*sin(head), -cos(roll)*sin(head)+sin(roll)*sin(pitch)*cos(head), -sin(roll)*cos(pitch);
       cos(pitch)*sin(head),                               cos(pitch)*cos(head),                                sin(pitch);
       sin(roll)*cos(head)-cos(roll)*sin(pitch)*sin(head), -sin(roll)*sin(head)-cos(roll)*sin(pitch)*cos(head), cos(roll)*cos(pitch)];

  Fn=Cbn'*Fb;

  Ve=veloN(1,1);Vn=veloN(2,1);Vu=veloN(3,1);

  Wenn=[-Vn/(Rm+heig); Ve/(Rn+heig);  Ve/(Rn+heig)*tan(lati)];
  Wien=[0;             Wie*cos(lati); Wie*sin(lati)];
  
  %一阶龙格库塔法解微分方程
  veloN=veloN+T*(Fn-cross((2*Wien+Wenn),veloN)+[0.0;0.0;-1.0*g]);





