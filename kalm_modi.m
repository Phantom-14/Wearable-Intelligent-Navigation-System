%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%                   卡尔曼滤波修正程序(利用GPS进行位置速度组合)
%
%  输入参数:
%           attiN-导航计算的姿态角度横滚，俯仰，航向（度，度，度）；
%           veloN-飞行器相对机体系的运动速度东向、北向、天向（米/秒）；
%           posiN-导航计算输出的飞行器位置（经度（度）、纬度（度）、高度（米））； 
%           Xc-综合模型状态量；误差修正量
%  输出参数：         
%           attiN-修正后的导航计算的姿态角度横滚，俯仰，航向（度，度，度）；
%           veloN-修正后的飞行器相对机体系的运动速度东向、北向、天向（米/秒）；
%           posiN-修正后的导航计算输出的飞行器位置（经度（度）、纬度（度）、高度（米））； 
%
%                           程序设计：熊智  日期：2003/10/05
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [attiN,veloN,posiN]=kalm_modi(attiN,veloN,posiN,Xc)



  veloN(1,1) = veloN(1,1)-Xc(4,1);  % m/s
  veloN(2,1) = veloN(2,1)-Xc(5,1);
  veloN(3,1) = veloN(3,1)-Xc(6,1);  
    %飞行器速度修正

  posiN(1,1) = posiN(1,1) - Xc(8,1)*180.0/pi; % deg
  posiN(2,1) = posiN(2,1) - Xc(7,1)*180.0/pi; % deg
  posiN(3,1) = posiN(3,1) - Xc(9,1);          % m
    %飞行器位置修正
  
  %return;
  
  roll=attiN(1,1)*pi/180.0;pitch=attiN(2,1)*pi/180.0;head=attiN(3,1)*pi/180.0;
    
  Cbc=[cos(roll)*cos(head)+sin(roll)*sin(pitch)*sin(head), -cos(roll)*sin(head)+sin(roll)*sin(pitch)*cos(head), -sin(roll)*cos(pitch);
       cos(pitch)*sin(head),                               cos(pitch)*cos(head),                                sin(pitch);
       sin(roll)*cos(head)-cos(roll)*sin(pitch)*sin(head), -sin(roll)*sin(head)-cos(roll)*sin(pitch)*cos(head), cos(roll)*cos(pitch)];
  %坐标系C-->B(机体系－－>计算系)
  
  Ccn=[ 1,       Xc(3,1),  -Xc(2,1);
       -Xc(3,1), 1          Xc(1,1);
        Xc(2,1), -Xc(1,1), 1        ];
    %修正矩阵N-->C(导航坐标系－－>计算坐标系)
  
  Cbn =( Ccn'*Cbc' )';
    %姿态矩阵修正N--->B

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %求姿态(横滚、俯仰、航向）
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  attiN(1,1)=atan(-Cbn(1,3)/Cbn(3,3));
  attiN(2,1)=atan(Cbn(2,3)/sqrt(Cbn(2,1)*Cbn(2,1)+Cbn(2,2)*Cbn(2,2)));
  attiN(3,1)=atan(Cbn(2,1)/Cbn(2,2));
    %单位：弧度

  %象限判断
  attiN(1,1)=attiN(1,1)*180.0/pi;
  attiN(2,1)=attiN(2,1)*180.0/pi;
  attiN(3,1)=attiN(3,1)*180.0/pi;
    % 单位：度

  if(Cbn(2,2)<0 ) 
   attiN(3,1)=180.0+attiN(3,1);
  else 
   if(Cbn(2,1)<0) attiN(3,1)=360.0+attiN(3,1); end
  end
    %航向角度（单位：度）

  if(Cbn(3,3)<0)
   if(Cbn(1,3)>0) attiN(1,1)=180.0-attiN(1,1); end
   if(Cbn(1,3)<0) attiN(1,1)=-(180.0+attiN(1,1)); end
  end
    %横滚角度（单位：度）


