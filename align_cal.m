%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%                    初始对准程序（粗对准）
%
%  输入参数:
%  Wibb       机体系陀螺仪输出   （单位：度/秒）
%  Fb         机体系加速度计输出 （单位：米/秒/秒）
%  posiN      经度、纬度、高度（单位：度、度、米）
%  输出参数：
%  attiN      横滚、俯仰、航向（单位：度）   
%      
%                           程序设计：熊智  日期：2002/8/9
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [attiN]=align_cal(Wibb,Fb,posiN)

  Wie=7.292115147e-5; %地球自转角速度
  g=9.7803698;        %重力加速度
  
  long=posiN(1,1)*pi/180.0;lati=posiN(2,1)*pi/180.0;heig=posiN(3,1);
    %飞行器位置

  Wien=[0;  Wie*cos(lati); Wie*sin(lati)];
  Gn  =[0;              0; g            ];
  Gman=cross(Gn,Wien);
    %东北天地理坐标系输出
    
  Wieb=Wibb*pi/180.0;    %弧度/秒
  Gmab=cross(Fb,Wieb);
    %IMU输出
      
  Cnb=inv([Gn';Wien';Gman'])*[Fb';Wieb';Gmab'];
  
  Cbn=Cnb';
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %求姿态(横滚、俯仰、航向）
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   attiN(1,1)=atan(-Cbn(1,3)/Cbn(3,3));
%   attiN(2,1)=atan(Cbn(2,3)/sqrt(Cbn(2,1)*Cbn(2,1)+Cbn(2,2)*Cbn(2,2)));
   attiN(3,1)=atan(Cbn(2,1)/Cbn(2,2));
%     %单位：弧度
  
  attiN(2,1)=  asin(Fb(2,1)/g);  %低精度下对粗对准算法的调整，避免陀螺误差对于水平姿态的影响
  attiN(1,1)= asin(-Fb(1,1)/(g*cos(attiN(2,1))));%单位：弧度%同上
  
    
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

  
  
  
  