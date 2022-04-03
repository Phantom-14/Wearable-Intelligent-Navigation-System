%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%          SINS协方差仿真
%                        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;
%%%%%%%%常数设置%%%%%%%%%%%
deg_rad=0.01745329252e0;% Transfer from angle degree to rad
g=9.7803698;         %重力加速度    （单位：米/秒/秒）

%%%%%%%%仿真时间设置%%%%%%
t=0;
T=0.01;
t_stop=300.0;

%IMU_Data=load('IMU_MOD_1.txt');
% IMU_Data=load('326Rec00-calibrated.txt');
% IMU_Data=load('MT_01500291_007-000.dat');
% Atti_MTiG=load('MT_01500291_007-001.txt');
%GPS_Data=load('GPS data 12-28.txt');

% IMU_Data=load('MT_01500291_007-000.dat');
DATA1=load('imu_data._dz.dat');
DATA2=load('imu_data.dat');
IMU_Data=[DATA1;DATA2];

%t_stop=length(IMU_Data(:,1))*T;
%%%%%%%%%%%%%%%%航迹发生器%%%%%%%%%%%%%%%%%
atti=zeros(3,1);     %横滚、俯仰、航向（单位：度）
atti_rate=zeros(3,1);%横滚速率、俯仰速率、航向速率（单位：度/秒）
veloB=zeros(3,1);    %飞机运动速度——X右翼、Y机头、Z天向（单位：米/秒）
acceB=zeros(3,1);    %飞机运动加速度——X右翼、Y机头、Z天向（单位：米/秒/秒）
posi=zeros(3,1);     %航迹发生器初始位置经度、纬度、高度（单位：度、度、米）
posi=[118.855015;32.028929;50.0];  

atti(1,1)=0.0;
atti(2,1)=0.0;
atti(3,1)=0.0;  %初始航向角度（单位：度）

veloB(2,1)=0.0; %飞机初始运动速度——机头（单位：米/秒）

%%%%%%%%%%%%%%%%%%%%IMU输出%%%%%%%%%%
Wibb=zeros(3,1);    %机体系陀螺仪输出   （单位：度/秒）
Fb=zeros(3,1);      %机体系加速度计输出 （单位：米/秒/秒）

Gyro_fix=zeros(3,1);%机体系陀螺仪固定误差输出   （单位：弧度/秒）
Acc_fix=zeros(3,1); %机体系加速度计固定误差输出 （单位：米/秒/秒）
Gyro_b=zeros(3,1);  % 陀螺随机常数（弧度/秒）
Gyro_r=zeros(3,1);  % 陀螺一阶马尔可夫过程（弧度/秒）
Gyro_wg=zeros(3,1); %陀螺白噪声（弧度/秒）
Acc_r =zeros(3,1);  % 加速度一阶马尔可夫过程（米/秒/秒）

Wibb_mean=zeros(3,1); 

%%%%%%%%%%%%%%%%%%GPS仿真输出%%%%%%%%%%%%%%
posiG = zeros(3,1); %GPS输出的飞行器位置（经度（度）、纬度（度）、高度（米））； 
veloG = zeros(3,1); %GPS输出的飞行器速度（东向（米/秒），北向（米/秒），天向（米/秒））


%%%%%%%%%%%%%%%%%%%%%捷联惯导仿真%%%%%%%%%%%%% 
attiN=zeros(3,1);        %飞行器初始姿态
veloN=zeros(3,1);        %飞行器初始速度（相对于导航系）
posiN=zeros(3,1);        %飞行器初始位置
WnbbA_old=zeros(3,1);    %角速度积分输出（单位：弧度）

posiN=posi;              %初始位置与航迹位置一致
attiN=atti;              %初始姿态与航迹姿态一致（可以用初始对准函数替换）

%%%%%%%%%%%%%%%%%%%%%%%KALMAN滤波输出%%%%%%%%%%%%%%%%%
T_D = T; %离散周期；
T_D2 = 0.5;
T_M = 0;    %滤波量测产生时间（秒）；
Xc = zeros(15,1);  %综合模型状态量；(可用于误差修正)全部为国际单位制
PK = zeros(15,15); %协方差阵；
Xerr =zeros(1,15); %状态估计量的误差值（记录某个时刻的）
kflag=0;           %GPS信息有效标志位（1－有效）

Acc_modi = zeros(3,1); %加速度计误差修正值（米/秒/秒）（X,Y,Z）
Gyro_modi= zeros(3,1); %陀螺误差修正值(弧度/秒)(X,Y,Z)

 Acc_modi_2= zeros(3,1); 
 Gyro_modi_2= zeros(3,1);
 mag_sig_b=zeros(3,1);

%%%%%%%%%%%%%%%%%%%初始对准%%%%%%%%%%%%%%%%%%%
kc=0;
tmp_Fb=zeros(3,1);
tmp_Wibb=zeros(3,1);
t_alig  = 0;

%%%%%%%%%%%%%%%%%数据记录%%%%%%%%%%%
DATA1=[];
DATA2=[];
TraceData=[];
IMUData=[];
GPSData=[];
SinsData=[];
KALData=[];
ErrData=[];
IMUData_MOD=[];
count=1;
count_GPS=1;
t=0;   %对准与导航开始


% [Xc,PK,Xerr]=kalm_gps_init(posiN,Xc,PK,Xerr);
       %卡尔曼滤波初始化
%        
while t<=35  %0-20s进行粗对准
%     Fb=[-IMU_Data(count,6);-IMU_Data(count,5);IMU_Data(count,7)]; %m/s/s  采用读文件数据形式进行捷联解算
%     Wibb=[IMU_Data(count,3);IMU_Data(count,2);-IMU_Data(count,4)];  %°/s
    Fb=[IMU_Data(count,3);-IMU_Data(count,2);IMU_Data(count,4)]; %m/s/s  2011-11-12  采用读文件数据形式进行捷联解算
    Wibb=[IMU_Data(count,6);-IMU_Data(count,5);IMU_Data(count,7)]/deg_rad; % deg/s/deg_rad
    
%     attiN(3,1)=Atti_MTiG(count,4);
%     if (attiN(3,1)<0)
%       attiN(3,1)=attiN(3,1)+360;
%     end  
 
%       if(rem(t,50)<0.1)    %每隔50秒使用1次地磁数据
%    mag_sig_b=[IMU_Data(count,8);-IMU_Data(count,7);IMU_Data(count,9)];
%    [attiN]=mag_head(attiN,mag_sig_b); 
%       end      
 
    tmp_Fb = tmp_Fb+Fb;
    tmp_Wibb = tmp_Wibb+Wibb;
    kc=kc+1;
    t=t+T;
    count=count+1;  

end

Fb=tmp_Fb/kc;
Wibb=tmp_Wibb/kc;
Wibb_mean=zeros(3,1); 
Wibb_mean=Wibb;%对于自制IMU陀螺零偏较大的情况下使用，直接扣除零偏

[attiN]=align_cal(Wibb,Fb,posiN); %初始对准计算

[veloN]=veloN0(attiN,veloB);%计算初始速度


%attiN(3,1)=atti(3,1)+0.0;
%attiN(1,1)=atti(1,1)+0.3;
%attiN(2,1)=atti(2,1)+0.3;

%Wibb_const=Wibb;%保存陀螺仪常值偏置

[Xc,PK,Xerr]=kalm_gps_init_15D(posiN,Xc,PK,Xerr);
       %卡尔曼滤波重新初始化 
       
while t<=70   %20-40s进行精对准
  
     
%     Fb=[-IMU_Data(count,6);-IMU_Data(count,5);IMU_Data(count,7)]; %m/s/s  2011-01-08  采用读文件数据形式进行捷联解算
%     Wibb=[IMU_Data(count,3)-Wibb_mean(1);IMU_Data(count,2)-Wibb_mean(2);-IMU_Data(count,4)-Wibb_mean(3)];  % deg/s
    Fb=[IMU_Data(count,3);-IMU_Data(count,2);IMU_Data(count,4)]; %m/s/s  2011-11-12  采用读文件数据形式进行捷联解算
    Wibb=[IMU_Data(count,6);-IMU_Data(count,5);IMU_Data(count,7)]/deg_rad-Wibb_mean;  % deg/s/deg_rad
    %Wibb=Wibb-Wibb_const;
   
%     attiN(3,1)=Atti_MTiG(count,4);
%    if (attiN(3,1)<0)
%       attiN(3,1)=attiN(3,1)+360;
%    end  
    
   [attiN,WnbbA_old]=atti_cal_cq_modi(T,Wibb,attiN,veloN,posiN,WnbbA_old);% 四元数算法(含等效转动矢量修正）%2010-11-25
      %姿态角度求解   
      
%   if(rem(t,50)<0.1)    %每隔50秒使用1次地磁数据
%    mag_sig_b=[IMU_Data(count,8);-IMU_Data(count,7);IMU_Data(count,9)];
%    [attiN]=mag_head(attiN,mag_sig_b); 
%   end      
%  
 
  [veloN]=velo_cal(T,Fb,attiN,veloN,posiN);%2010-11-25
      %比力变换
   
  [posiN]=posi_cal(T,veloN,posiN);
      %定位计算
  
  t=t+T;
 
  T_M = T_M + T; kflag = 0; 
        
  
  if( T_M >= T_D )  
      [posiG,veloG]=simu_gps(t,posi,atti,veloB);
        %GPS输出
        
      T_M = 0.0; kflag = 1; 
      
      %Xc = zeros(18,1); %闭环反馈校正，控制修正量初值为0
      
      [Xc,PK,Xerr]=kalm_gps_15D(t,T_D,Fb,attiN,veloN,posiN,posiG,veloG,Xc,PK,Xerr,kflag);
         %卡尔曼滤波   
     
%      [attiN,veloN,posiN]=kalm_modi(attiN,veloN,posiN,Xc);
%         %进行滤波修正
%       

        
  GPSData=[GPSData;t,posiG',veloG'];
  KALData = [KALData;t,Xerr];
  ErrData = [ErrData;t,Xc'];
         %保存仿真数据   
                 
  TraceData=[TraceData;t,posi',veloB(2,1),atti'];
  IMUData=[IMUData;t,3600.0*Wibb',1/g*Fb'];
  IMUData_MOD=[IMUData_MOD;t,3600.0*(Wibb-Gyro_modi)',1/g*(Fb-Acc_modi)'];%校正后的IMU输出
  SinsData=[SinsData;t,attiN',veloN',posiN'];
      %保存仿真数据
  count=count+1;  
  
  end
end

[attiN,veloN,posiN]=kalm_modi(attiN,veloN,posiN,Xc);%%对准后姿态、速度、位置误差一次性校正，完成精对准

% attiN(3,1)=Atti_MTiG(count,4);%采用传感器组件输出的航向作为初始航向
% if (attiN(3,1)<0)
%       attiN(3,1)=attiN(3,1)+360;
%     end  

Gyro_modi(1,1) = Xc(10,1)/deg_rad; %陀螺仪误差校正量，单位：°/s
Gyro_modi(2,1) = Xc(11,1)/deg_rad; %单位：°/s
%Gyro_modi(3,1) = Xc(12,1)/deg_rad; %单位：°/s
         %陀螺修正量
         
 %Acc_modi(1,1) = Xc(13,1);%加速度计误差校正量，单位：m/s/s
 %Acc_modi(2,1) = Xc(14,1);%单位：m/s/s
 Acc_modi(3,1) = Xc(15,1);%单位：m/s/s     
          %加速度计修正量


% disp('*******初始非随机性误差（单位：度\小时,米/秒/秒）*********');
% disp('陀螺刻度系数和安装误差 | 加速度计刻度系数和安装误差');
% disp([Gyro_fix/deg_rad*3600.0,Acc_fix]);
% 
% disp('*******初始随机性误差（单位：度\小时,米/秒/秒）*********');
% disp('陀螺随机常数，陀螺一阶马儿可夫，陀螺白噪声  | 加速度一阶马儿可夫');
% disp([Gyro_b/deg_rad*3600.0,Gyro_r/deg_rad*3600.0,Gyro_wg/deg_rad*3600.0,Acc_r]);
% disp('*******');

%  attiN(3,1)=250; attiN(2,1)=-20; attiN(1,1)=-2.5;
%  attiN(3,1)=275; attiN(2,1)=-19; attiN(1,1)=0;
%  
[Xc,PK,Xerr]=kalm_gps_init_15D(posiN,Xc,PK,Xerr);%卡尔曼滤波器再次初始化

while t<=t_stop  %捷联解算与组合导航过程开始
         
%     Fb=[-IMU_Data(count,6);-IMU_Data(count,5);IMU_Data(count,7)]; %m/s/s  采用读文件数据形式进行捷联解算
%     Wibb=[IMU_Data(count,3)-Wibb_mean(1);IMU_Data(count,2)-Wibb_mean(2);-IMU_Data(count,4)-Wibb_mean(3)];  % deg/s
    Fb=[IMU_Data(count,3);-IMU_Data(count,2);IMU_Data(count,4)]-Acc_modi; %m/s/s  2011-11-12  采用读文件数据形式进行捷联解算
%     Wibb=[IMU_Data(count,6);-IMU_Data(count,5);IMU_Data(count,7)]/deg_rad-Wibb_mean-Gyro_modi;  % deg/s
    %Wibb=Wibb-Wibb_const;
%     Wibb_mean(1)=-3.04241; Wibb_mean(2)=0.83652; Wibb_mean(3)=1.26051; 
     Wibb_mean(1)=0; Wibb_mean(2)=0; Wibb_mean(3)=0; 
     Wibb=[IMU_Data(count,6);-IMU_Data(count,5);IMU_Data(count,7)]/deg_rad-Gyro_modi-Wibb_mean;  % deg/s
   
  %[attiN]=atti_cal(T,Wibb,attiN,veloN,posiN);    % 方向余弦算法
  %[attiN]=atti_cal_cq(T,Wibb,attiN,veloN,posiN); % 四元数算法
  [attiN,WnbbA_old]=atti_cal_cq_modi(T,Wibb-Gyro_modi_2,attiN,veloN,posiN,WnbbA_old);% 四元数算法(含等效转动矢量修正）
      %姿态角度求解，解算过程中采用对准中估计得到的陀螺仪误差进行校正
%      
%       attiN(3,1)=-Atti_MTiG(count,4);
%       if (attiN(3,1)<0)
%          attiN(3,1)=attiN(3,1)+360;
%       end   
       
  %[attiN,WnbbA_old]=atti_cal_cq_modi(T,Wibb,attiN,veloN,posiN,WnbbA_old);% 四元数算法(含等效转动矢量修正）%2010-11-25
      %姿态角度求解   
%  
%   if(rem(t,75)<0.1)    %每隔50秒使用1次地磁数据
%    mag_sig_b=[IMU_Data(count,9);-IMU_Data(count,8);IMU_Data(count,10)];
%    [attiN]=mag_head(attiN,mag_sig_b); 
%  end      

[veloN]=velo_cal(T,Fb-Acc_modi-Acc_modi_2,attiN,veloN,posiN);%位置解算，解算过程采用初始对准估计得到的加速度计误差进行校正
  %[veloN]=velo_cal(T,Fb,attiN,veloN,posiN);%2010-11-25
      %比力变换

  [posiN]=posi_cal(T,veloN,posiN);
      %定位计算
  
  t=t+T;
 
  T_M = T_M + T; kflag = 0;  
  Mag=sqrt(Fb(1)^2+Fb(2)^2+Fb(3)^2)-g; 
  
  %if( T_M >= T_D2 )  %2010-11-14修改 %只进行捷联解算，暂不进行组合导航
  if( max(Wibb)<= 4.5&&Mag<=0.15)  %2011-11-15修改 %静态时进行零速修正
      [posiG,veloG]=simu_gps(t,posi,atti,veloB);
%       posiG=[GPS_Data(count_GPS,4);GPS_Data(count_GPS,3);GPS_Data(count_GPS,5)];
%       veloG=[GPS_Data(count_GPS,7);GPS_Data(count_GPS,6);-GPS_Data(count_GPS,8)];
      count_GPS=count_GPS+1;
        %GPS输出
        
      T_M = 0.0; kflag = 1; 
      
%       if(t_stop<=62||(t_stop>=74&& t_stop<=90)||(t_stop>=106&& t_stop<=125)||(t_stop>=140&& t_stop<=150)||(t_stop>=170&& t_stop<=184)||(t_stop>=196&& t_stop<=208)||(t_stop>=224&& t_stop<=240))     
%          mag_sig_b=[-IMU_Data(count,9);IMU_Data(count,8);IMU_Data(count,10)];
%          [attiN]=mag_head(attiN,mag_sig_b); 
%       end  
      
     
      Xc = zeros(15,1); %闭环反馈校正，控制修正量初值为0
      
      [Xc,PK,Xerr]=kalm_gps_15D(t,T_D2,Fb,attiN,veloN,posiN,posiG,veloG,Xc,PK,Xerr,kflag);
         %卡尔曼滤波   
      
%       if(t<=66)  
%         [veloN,posiN]=kalm_modi2(veloN,posiN,Xc);%不校正姿态、只校正速度与位置
        %进行滤波修正
    
%       else 
      [attiN,veloN,posiN]=kalm_modi(attiN,veloN,posiN,Xc);%同时校正速度与位置
%       end
       
      
%      attiN(3,1)=-Atti_MTiG(count,4);
%       if (attiN(3,1)<0)
%          attiN(3,1)=attiN(3,1)+360;
%       end     
      
     %[attiN]=align_cal(Wibb,Fb,posiN);  %2011-11-15 采用加速度计计算水平姿态
     % attiN(3,1)=atti(3,1)+0.5;
      
      Gyro_modi_2(1,1) = Xc(10,1)/deg_rad;
      Gyro_modi_2(2,1) = Xc(11,1)/deg_rad;
      %Gyro_modi_2(3,1) = Xc(12,1)/deg_rad;
         %陀螺修正量
         
      %Acc_modi_2(1,1) = Xc(13,1);
     % Acc_modi_2(2,1) = Xc(14,1);
      Acc_modi_2(3,1) = Xc(15,1);      
         %加速度计修正量
         
      GPSData=[GPSData;t,posiG',veloG'];
      KALData = [KALData;t,Xerr];
      ErrData = [ErrData;t,Xc'];
         %保存仿真数据       
  end
       
  TraceData=[TraceData;t,posi',veloB(2,1),atti'];
  IMUData=[IMUData;t,3600.0*Wibb',1/g*Fb'];
  IMUData_MOD=[IMUData_MOD;t,3600.0*(Wibb-Gyro_modi-Gyro_modi_2)',1/g*(Fb-Acc_modi)'];%校正
  %后的IMU输出
  SinsData=[SinsData;t,attiN',veloN',posiN'];
      %保存仿真数据
 count=count+1;
end

%%%%%%%%%%%%%%绘制曲线%%%%%%%%%
fig_num=0;

% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(TraceData(:,1),TraceData(:,2));
% subplot(3,1,2);plot(TraceData(:,1),TraceData(:,3));
% subplot(3,1,3);plot(TraceData(:,1),TraceData(:,4));
% xlabel('飞行航迹仿真（经度（度），纬度（度），高度（米））');

% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(TraceData(:,1),TraceData(:,6));ylabel('横滚角（°）');
% subplot(3,1,2);plot(TraceData(:,1),TraceData(:,7));ylabel('俯仰角（°）');
% subplot(3,1,3);plot(TraceData(:,1),TraceData(:,8));ylabel('航向角度（°）');
% %xlabel('时间（t）')
% xlabel('飞行航迹仿真（横滚角度（度），俯仰角度（度），航向角度（度））');
%    % 航迹数据

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(IMUData(:,1),IMUData(:,2),IMUData_MOD(:,1),IMUData_MOD(:,2),'g');
subplot(3,1,2);plot(IMUData(:,1),IMUData(:,3),IMUData_MOD(:,1),IMUData_MOD(:,3),'g');
subplot(3,1,3);plot(IMUData(:,1),IMUData(:,4),IMUData_MOD(:,1),IMUData_MOD(:,4),'g');
xlabel('IMU输出（陀螺X轴（度/小时），陀螺Y轴（度/小时），陀螺Z轴（度/小时））');

% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(IMUData_MOD(:,1),IMUData_MOD(:,2),'r');
% subplot(3,1,2);plot(IMUData_MOD(:,1),IMUData_MOD(:,3),'r');
% subplot(3,1,3);plot(IMUData_MOD(:,1),IMUData_MOD(:,4),'r');
% xlabel('IMU输出（陀螺X轴（度/小时），陀螺Y轴（度/小时），陀螺Z轴（度/小时））');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(IMUData(:,1),IMUData(:,5),IMUData_MOD(:,1),IMUData_MOD(:,5),'g');
subplot(3,1,2);plot(IMUData(:,1),IMUData(:,6),IMUData_MOD(:,1),IMUData_MOD(:,6),'g');
subplot(3,1,3);plot(IMUData(:,1),IMUData(:,7),IMUData_MOD(:,1),IMUData_MOD(:,7),'g');
xlabel('IMU输出（加表X轴g），加表Y轴（g），加表Z轴（g））');
   %IMU数据

% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(GPSData(:,1),GPSData(:,2));
% subplot(3,1,2);plot(GPSData(:,1),GPSData(:,3));
% subplot(3,1,3);plot(GPSData(:,1),GPSData(:,4));
% xlabel('GPS仿真（经度（度），纬度（度），高度（米））');

% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(GPSData(:,1),GPSData(:,5));
% subplot(3,1,2);plot(GPSData(:,1),GPSData(:,6));
% subplot(3,1,3);plot(GPSData(:,1),GPSData(:,7));
% xlabel('GPS仿真（东向速度（米/秒），北向速度（米/秒），天向速度（米/秒））');
%    %GPS仿真

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(SinsData(:,1),SinsData(:,2));ylabel('横滚角（°）');
subplot(3,1,2);plot(SinsData(:,1),SinsData(:,3));ylabel('俯仰角（°）');
subplot(3,1,3);plot(SinsData(:,1),SinsData(:,4));ylabel('航向角度（°）');
xlabel('时间（t）')
%xlabel('导航姿态角输出（横滚角度（度），俯仰角度（度），航向角度（度））');
   %姿态角度数据
   
fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(SinsData(:,1),SinsData(:,5));
subplot(3,1,2);plot(SinsData(:,1),SinsData(:,6));
subplot(3,1,3);plot(SinsData(:,1),SinsData(:,7));
xlabel('导航速度输出（东向（米/秒），北向（米/秒），天向（米/秒））');
   %速度数据

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(SinsData(:,1),SinsData(:,8));
subplot(3,1,2);plot(SinsData(:,1),SinsData(:,9));
subplot(3,1,3);plot(SinsData(:,1),SinsData(:,10));
xlabel('导航航迹（经度（度），纬度（度），高度（米））');
   %位置数据

   
figure(14); plot(SinsData(:,8),SinsData(:,9));
figure(13);plot3(SinsData(:,8),SinsData(:,9),SinsData(:,10)) ;
grid on;
% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(TraceData(:,1),SinsData(:,2)-TraceData(:,6));
% subplot(3,1,2);plot(TraceData(:,1),SinsData(:,3)-TraceData(:,7));
% subplot(3,1,3);plot(TraceData(:,1),SinsData(:,4)-TraceData(:,8));
% xlabel('姿态角度误差（横滚角度误差（度），俯仰角度误差（度），航向角度误差（度））');
%    %飞行姿态角度误差曲线    
   
% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(TraceData(:,2),TraceData(:,3),'r',SinsData(:,8),SinsData(:,9));
% subplot(3,1,2);plot(TraceData(:,1),SinsData(:,8)-TraceData(:,2));
% subplot(3,1,3);plot(TraceData(:,1),SinsData(:,9)-TraceData(:,3));
% xlabel('惯导经纬度位置对比及其误差曲线（经纬度位置（度），经度误差（度），纬度误差（度））');
%    %飞行位置轨迹及误差曲线
   
fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(TraceData(:,1),SinsData(:,8)-TraceData(:,2));
subplot(3,1,2);plot(TraceData(:,1),SinsData(:,9)-TraceData(:,3));
subplot(3,1,3);plot(TraceData(:,1),SinsData(:,10)-TraceData(:,4));
 xlabel('惯导经纬度位置对比及其误差曲线（经度误差（度），纬度误差（度））,高度误差（米）');
   %飞行位置轨迹及误差曲线

% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(TraceData(:,2),TraceData(:,3),'r');
% subplot(3,1,2);plot(GPSData(:,2),GPSData(:,3),'g');
% subplot(3,1,3);plot(SinsData(:,8),SinsData(:,9));
% %xlabel('GPS经纬度位置对比及其误差曲线（经纬度位置（度），经度误差（度），纬度误差（度））');
%    %飞行位置轨迹及误差曲线  
%    
% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(GPSData(:,1),GPSData(:,2),'r',SinsData(:,1),SinsData(:,8));
% subplot(3,1,2);plot(GPSData(:,1),GPSData(:,3),'r',SinsData(:,1),SinsData(:,9));
% subplot(3,1,3);plot(GPSData(:,1),GPSData(:,4),'r',SinsData(:,1),SinsData(:,10));
% xlabel('组合导航与GPS对比（经度（度），纬度（度），高度（米））');
% 
% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(GPSData(:,1),GPSData(:,5),'r',SinsData(:,1),SinsData(:,5));
% subplot(3,1,2);plot(GPSData(:,1),GPSData(:,6),'r',SinsData(:,1),SinsData(:,6));
% subplot(3,1,3);plot(GPSData(:,1),GPSData(:,7),'r',SinsData(:,1),SinsData(:,7));
% xlabel('组合导航与GPS对比（东向速度（米/秒），北向速度（米/秒），天向速度（米/秒））');
%     %组合导航与GPS对比输出
% 
fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(KALData(:,1),KALData(:,2));
subplot(3,1,2);plot(KALData(:,1),KALData(:,3));
subplot(3,1,3);plot(KALData(:,1),KALData(:,4));
xlabel('卡尔曼滤波输出（平台误差角（秒））');
% 
fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(KALData(:,1),KALData(:,5));
subplot(3,1,2);plot(KALData(:,1),KALData(:,6));
subplot(3,1,3);plot(KALData(:,1),KALData(:,7));
xlabel('卡尔曼滤波输出（速度误差（米/秒））');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(KALData(:,1),KALData(:,9));
subplot(3,1,2);plot(KALData(:,1),KALData(:,8));
subplot(3,1,3);plot(KALData(:,1),KALData(:,10));
xlabel('卡尔曼滤波输出（位置误差（米））');
% 
fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(KALData(:,1),KALData(:,11));
subplot(3,1,2);plot(KALData(:,1),KALData(:,12));
subplot(3,1,3);plot(KALData(:,1),KALData(:,13));
xlabel('陀螺随机常数误差（度/小时）');
% 
% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(KALData(:,1),KALData(:,14));
% subplot(3,1,2);plot(KALData(:,1),KALData(:,15));
% subplot(3,1,3);plot(KALData(:,1),KALData(:,16));
% xlabel('陀螺一阶马尔可夫误差（度/小时）');
% 
fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(KALData(:,1),KALData(:,14));
subplot(3,1,2);plot(KALData(:,1),KALData(:,15));
subplot(3,1,3);plot(KALData(:,1),KALData(:,16));
xlabel('加速度计一阶马尔可夫误差（g）');
%    %%%%%%%%%%卡尔曼估计误差%%%%%%%%%%%
%    
% %return;
% 
% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(ErrData(:,1),ErrData(:,2)*180.0/pi*3600); %sec
% subplot(3,1,2);plot(ErrData(:,1),ErrData(:,3)*180.0/pi*3600); %sec
% subplot(3,1,3);plot(ErrData(:,1),ErrData(:,4)*180.0/pi*3600); %sec
% xlabel('平台误差补偿量（角秒）');
% 
% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(ErrData(:,1),ErrData(:,5)); %米/秒
% subplot(3,1,2);plot(ErrData(:,1),ErrData(:,6)); %米/秒
% subplot(3,1,3);plot(ErrData(:,1),ErrData(:,7)); %米/秒
% xlabel('速度误差补偿量（米/秒）');
% 
% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(ErrData(:,1),ErrData(:,9)*180.0/pi); % deg
% subplot(3,1,2);plot(ErrData(:,1),ErrData(:,8)*180.0/pi); % deg
% subplot(3,1,3);plot(ErrData(:,1),ErrData(:,10));         % m
% xlabel('位置误差补偿量(经度（度），纬度（度），高度（米）)');
% 
% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(ErrData(:,1),ErrData(:,11)*180.0/pi*3600); %deg/h
% subplot(3,1,2);plot(ErrData(:,1),ErrData(:,12)*180.0/pi*3600); %deg/h
% subplot(3,1,3);plot(ErrData(:,1),ErrData(:,13)*180.0/pi*3600); %deg/h
% xlabel('陀螺随机常数误差补偿量（度/小时）');
% 
% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(ErrData(:,1),ErrData(:,14)*180.0/pi*3600); %deg/h
% subplot(3,1,2);plot(ErrData(:,1),ErrData(:,15)*180.0/pi*3600); %deg/h
% subplot(3,1,3);plot(ErrData(:,1),ErrData(:,16)*180.0/pi*3600); %deg/h
% xlabel('陀螺一阶马尔可夫误差补偿量（度/小时）');
% 
% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(ErrData(:,1),ErrData(:,17)/g); %g
% subplot(3,1,2);plot(ErrData(:,1),ErrData(:,18)/g); %g
% subplot(3,1,3);plot(ErrData(:,1),ErrData(:,19)/g); %g
% xlabel('加速度计一阶马尔可夫误差补偿量（g）');
%   %卡尔曼滤波修正数据

  Re=6378137.0;  %地球半径（米） 
  f=1/298.257;   %地球的椭圆率
  long=posiN(1,1)*pi/180.0;lati=posiN(2,1)*pi/180.0;heig=posiN(3,1);%飞行器位置
  Rm=Re*(1-2*f+3*f*sin(lati)*sin(lati));
  Rn=Re*(1+f*sin(lati)*sin(lati)); %地球曲率半径求解
    
fig_num = fig_num+1;figure(fig_num);
plot((SinsData(:,8)-118.855015)*pi/180*(Rn+posi(3,1))*cos(lati),(SinsData(:,9)-32.028929)*pi/180*(Rm+posi(3,1)));
% plot(SinsData(:,8),SinsData(:,9));
% %%%%%%%%%%%%%存储仿真数据%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% save trace.dat TraceData -ASCII;    %存储航迹数据
% save imu.dat   IMUData   -ASCII;    %存储仿真产生的IMU数据
% save sins.dat  SinsData  -ASCII;    %存储导航输出数据
% save kal.dat   KALData   -ASCII;    %存储卡尔曼滤波些方差阵
% save err.dat   ErrData   -ASCII;    %存储卡尔曼滤波估计的误差修正值



Re=6378137.0;                                      %地球半径（米） 
f=1/298.257;                                        %地球的椭圆率
long=posi(1,1)*pi/180.0;lati=posi(2,1)*pi/180.0;

Rm=Re*(1-2*f+3*f*sin(lati)*sin(lati));
Rn=Re*(1+f*sin(lati)*sin(lati));

% clc;
% disp('经度误差： ')
% disp('    10s ')
% (SinsData(5000,8)-TraceData(5000,2))*pi/180*(Rn+posi(3,1))*cos(lati)
% disp('    20s ')
% (SinsData(6000,8)-TraceData(6000,2))*pi/180*(Rn+posi(3,1))*cos(lati)
% disp('    30s ')
% (SinsData(7000,8)-TraceData(7000,2))*pi/180*(Rn+posi(3,1))*cos(lati)
% disp('    60s ')
% (SinsData(10000,8)-TraceData(10000,2))*pi/180*(Rn+posi(3,1))*cos(lati)
% disp('    120s ')
% (SinsData(16000,8)-TraceData(16000,2))*pi/180*(Rn+posi(3,1))*cos(lati)
% 
% disp('纬度误差： ')
% disp('    10s ')
% (SinsData(5000,9)-TraceData(5000,3))*pi/180*(Rm+posi(3,1))
% disp('    20s ')
% (SinsData(6000,9)-TraceData(6000,3))*pi/180*(Rm+posi(3,1))
% disp('    30s ')
% (SinsData(7000,9)-TraceData(7000,3))*pi/180*(Rm+posi(3,1))
% disp('    60s ')
% (SinsData(10000,9)-TraceData(10000,3))*pi/180*(Rm+posi(3,1))
% disp('    120s ')
% (SinsData(16000,9)-TraceData(16000,3))*pi/180*(Rm+posi(3,1))