%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%                    姿态阵计算程序（四元数法含等效转动矢量修正）
%
%  T          仿真步长（秒）
%  Wibb       机体系陀螺仪输出   （单位：度/秒）
%  attiN      横滚、俯仰、航向（单位：度）
%  veloN      飞机运动速度——X东向、Y北向、Z天向（单位：米/秒）
%  posiN      经度、纬度、高度（单位：度、度、米）
%  WnbbA_old  角速度积分输出（单位：弧度）
%
%  输出参数：
%  attiN      横滚、俯仰、航向（单位：度）  
%  WnbbA_old  角速度积分输出（单位：弧度）
%      
%                           程序设计：熊智  日期：2002/8/9
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [attiN,WnbbA_old]=atti_cal_cq_modi(T,Wibb,attiN,veloN,posiN,WnbbA_old)

  Re=6378137.0;       %地球半径（米） 
  f=1/298.257;        %地球的椭圆率
  Wie=7.292115147e-5; %地球自转角速度

  long=posiN(1,1)*pi/180.0;lati=posiN(2,1)*pi/180.0;heig=posiN(3,1);
    %飞行器位置

  Rm=Re*(1-2*f+3*f*sin(lati)*sin(lati));
  Rn=Re*(1+f*sin(lati)*sin(lati));
    %地球曲率半径求解

  roll=attiN(1,1)*pi/180.0;pitch=attiN(2,1)*pi/180.0;head=attiN(3,1)*pi/180.0;

  Cbn=[cos(roll)*cos(head)+sin(roll)*sin(pitch)*sin(head), -cos(roll)*sin(head)+sin(roll)*sin(pitch)*cos(head), -sin(roll)*cos(pitch);
       cos(pitch)*sin(head),                               cos(pitch)*cos(head),                                sin(pitch);
       sin(roll)*cos(head)-cos(roll)*sin(pitch)*sin(head), -sin(roll)*sin(head)-cos(roll)*sin(pitch)*cos(head), cos(roll)*cos(pitch)];
    %坐标系N-->B

  Ve=veloN(1,1);Vn=veloN(2,1);Vu=veloN(3,1);

  Wenn=[-Vn/(Rm+heig); Ve/(Rn+heig);  Ve/(Rn+heig)*tan(lati)];
  Wien=[0;             Wie*cos(lati); Wie*sin(lati)];
    %单位：弧度/秒

  Wnbb=Wibb*pi/180.0-Cbn*(Wien+Wenn);  %单位：弧度/秒

  %%%%%%%%%%%%%%四元数法求姿态矩阵%%%%%%%%%%%%%%%
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

  %%%%%%%%%%%等效转动矢量修正算法%%%%%%%%%%%%%%%
  WnbbA_e=cross(WnbbA_old,WnbbA);
  WnbbX_e=[0,            -WnbbA_e(1,1), -WnbbA_e(2,1), -WnbbA_e(3,1);
           WnbbA_e(1,1),  0,             WnbbA_e(3,1), -WnbbA_e(2,1);
           WnbbA_e(2,1), -WnbbA_e(3,1),    0,           WnbbA_e(1,1);
           WnbbA_e(3,1),  WnbbA_e(2,1),  -WnbbA_e(1,1),   0         ];

  
  Q=( c_q*eye(4)+d_q*(WnbbX+1/12*WnbbX_e) )*Q;%修正算法
  
  WnbbA_old=WnbbA;

  %%%%%%%%%%四元数规范化%%%%%%%%%
  tmp_Q=sqrt(Q(1,1)^2+Q(2,1)^2+Q(3,1)^2+Q(4,1)^2);
  for kc=1:4
    Q(kc,1)=Q(kc,1)/tmp_Q;    
  end
  
  %%%%%%%%%%获取姿态矩阵%%%%%%%%%
  Cbn=[Q(2,1)^2+Q(1,1)^2-Q(4,1)^2-Q(3,1)^2, 2*(Q(2,1)*Q(3,1)+Q(1,1)*Q(4,1)), 2*(Q(2,1)*Q(4,1)-Q(1,1)*Q(3,1));
       2*(Q(2,1)*Q(3,1)-Q(1,1)*Q(4,1)), Q(3,1)^2-Q(4,1)^2+Q(1,1)^2-Q(2,1)^2,  2*(Q(3,1)*Q(4,1)+Q(1,1)*Q(2,1));
       2*(Q(2,1)*Q(4,1)+Q(1,1)*Q(3,1)), 2*(Q(3,1)*Q(4,1)-Q(1,1)*Q(2,1)), Q(4,1)^2-Q(3,1)^2-Q(2,1)^2+Q(1,1)^2]; 
  
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


