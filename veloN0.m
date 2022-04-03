%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%                    初始速度计算程序
%
%  输入参数:
%  atti     %横滚、俯仰、航向（单位：度）
%  veloB    %飞机运动速度——X右翼、Y机头、Z天向（单位：米/秒）
%  输出参数：
%  veloN    %飞行器初始速度
%      
%                           程序设计：熊智  日期：2002/4/15
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [veloN]=veloN0(atti,veloB)

  roll=atti(1,1)*pi/180.0;pitch=atti(2,1)*pi/180.0;head=atti(3,1)*pi/180.0;

  %坐标系N-->B
  Cbn=[cos(roll)*cos(head)+sin(roll)*sin(pitch)*sin(head), -cos(roll)*sin(head)+sin(roll)*sin(pitch)*cos(head), -sin(roll)*cos(pitch);
       cos(pitch)*sin(head),                               cos(pitch)*cos(head),                                sin(pitch);
       sin(roll)*cos(head)-cos(roll)*sin(pitch)*sin(head), -sin(roll)*sin(head)-cos(roll)*sin(pitch)*cos(head), cos(roll)*cos(pitch)];


  veloN=Cbn'*veloB;



