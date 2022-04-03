%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%          SINSЭ�������
%                        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;
%%%%%%%%��������%%%%%%%%%%%
deg_rad=0.01745329252e0;% Transfer from angle degree to rad
g=9.7803698;         %�������ٶ�    ����λ����/��/�룩

%%%%%%%%����ʱ������%%%%%%
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
%%%%%%%%%%%%%%%%����������%%%%%%%%%%%%%%%%%
atti=zeros(3,1);     %��������������򣨵�λ���ȣ�
atti_rate=zeros(3,1);%������ʡ��������ʡ��������ʣ���λ����/�룩
veloB=zeros(3,1);    %�ɻ��˶��ٶȡ���X����Y��ͷ��Z���򣨵�λ����/�룩
acceB=zeros(3,1);    %�ɻ��˶����ٶȡ���X����Y��ͷ��Z���򣨵�λ����/��/�룩
posi=zeros(3,1);     %������������ʼλ�þ��ȡ�γ�ȡ��߶ȣ���λ���ȡ��ȡ��ף�
posi=[118.855015;32.028929;50.0];  

atti(1,1)=0.0;
atti(2,1)=0.0;
atti(3,1)=0.0;  %��ʼ����Ƕȣ���λ���ȣ�

veloB(2,1)=0.0; %�ɻ���ʼ�˶��ٶȡ�����ͷ����λ����/�룩

%%%%%%%%%%%%%%%%%%%%IMU���%%%%%%%%%%
Wibb=zeros(3,1);    %����ϵ���������   ����λ����/�룩
Fb=zeros(3,1);      %����ϵ���ٶȼ���� ����λ����/��/�룩

Gyro_fix=zeros(3,1);%����ϵ�����ǹ̶�������   ����λ������/�룩
Acc_fix=zeros(3,1); %����ϵ���ٶȼƹ̶������� ����λ����/��/�룩
Gyro_b=zeros(3,1);  % �����������������/�룩
Gyro_r=zeros(3,1);  % ����һ������ɷ���̣�����/�룩
Gyro_wg=zeros(3,1); %���ݰ�����������/�룩
Acc_r =zeros(3,1);  % ���ٶ�һ������ɷ���̣���/��/�룩

Wibb_mean=zeros(3,1); 

%%%%%%%%%%%%%%%%%%GPS�������%%%%%%%%%%%%%%
posiG = zeros(3,1); %GPS����ķ�����λ�ã����ȣ��ȣ���γ�ȣ��ȣ����߶ȣ��ף����� 
veloG = zeros(3,1); %GPS����ķ������ٶȣ�������/�룩��������/�룩��������/�룩��


%%%%%%%%%%%%%%%%%%%%%�����ߵ�����%%%%%%%%%%%%% 
attiN=zeros(3,1);        %��������ʼ��̬
veloN=zeros(3,1);        %��������ʼ�ٶȣ�����ڵ���ϵ��
posiN=zeros(3,1);        %��������ʼλ��
WnbbA_old=zeros(3,1);    %���ٶȻ����������λ�����ȣ�

posiN=posi;              %��ʼλ���뺽��λ��һ��
attiN=atti;              %��ʼ��̬�뺽����̬һ�£������ó�ʼ��׼�����滻��

%%%%%%%%%%%%%%%%%%%%%%%KALMAN�˲����%%%%%%%%%%%%%%%%%
T_D = T; %��ɢ���ڣ�
T_D2 = 0.5;
T_M = 0;    %�˲��������ʱ�䣨�룩��
Xc = zeros(15,1);  %�ۺ�ģ��״̬����(�������������)ȫ��Ϊ���ʵ�λ��
PK = zeros(15,15); %Э������
Xerr =zeros(1,15); %״̬�����������ֵ����¼ĳ��ʱ�̵ģ�
kflag=0;           %GPS��Ϣ��Ч��־λ��1����Ч��

Acc_modi = zeros(3,1); %���ٶȼ��������ֵ����/��/�룩��X,Y,Z��
Gyro_modi= zeros(3,1); %�����������ֵ(����/��)(X,Y,Z)

 Acc_modi_2= zeros(3,1); 
 Gyro_modi_2= zeros(3,1);
 mag_sig_b=zeros(3,1);

%%%%%%%%%%%%%%%%%%%��ʼ��׼%%%%%%%%%%%%%%%%%%%
kc=0;
tmp_Fb=zeros(3,1);
tmp_Wibb=zeros(3,1);
t_alig  = 0;

%%%%%%%%%%%%%%%%%���ݼ�¼%%%%%%%%%%%
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
t=0;   %��׼�뵼����ʼ


% [Xc,PK,Xerr]=kalm_gps_init(posiN,Xc,PK,Xerr);
       %�������˲���ʼ��
%        
while t<=35  %0-20s���дֶ�׼
%     Fb=[-IMU_Data(count,6);-IMU_Data(count,5);IMU_Data(count,7)]; %m/s/s  ���ö��ļ�������ʽ���н�������
%     Wibb=[IMU_Data(count,3);IMU_Data(count,2);-IMU_Data(count,4)];  %��/s
    Fb=[IMU_Data(count,3);-IMU_Data(count,2);IMU_Data(count,4)]; %m/s/s  2011-11-12  ���ö��ļ�������ʽ���н�������
    Wibb=[IMU_Data(count,6);-IMU_Data(count,5);IMU_Data(count,7)]/deg_rad; % deg/s/deg_rad
    
%     attiN(3,1)=Atti_MTiG(count,4);
%     if (attiN(3,1)<0)
%       attiN(3,1)=attiN(3,1)+360;
%     end  
 
%       if(rem(t,50)<0.1)    %ÿ��50��ʹ��1�εش�����
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
Wibb_mean=Wibb;%��������IMU������ƫ�ϴ�������ʹ�ã�ֱ�ӿ۳���ƫ

[attiN]=align_cal(Wibb,Fb,posiN); %��ʼ��׼����

[veloN]=veloN0(attiN,veloB);%�����ʼ�ٶ�


%attiN(3,1)=atti(3,1)+0.0;
%attiN(1,1)=atti(1,1)+0.3;
%attiN(2,1)=atti(2,1)+0.3;

%Wibb_const=Wibb;%���������ǳ�ֵƫ��

[Xc,PK,Xerr]=kalm_gps_init_15D(posiN,Xc,PK,Xerr);
       %�������˲����³�ʼ�� 
       
while t<=70   %20-40s���о���׼
  
     
%     Fb=[-IMU_Data(count,6);-IMU_Data(count,5);IMU_Data(count,7)]; %m/s/s  2011-01-08  ���ö��ļ�������ʽ���н�������
%     Wibb=[IMU_Data(count,3)-Wibb_mean(1);IMU_Data(count,2)-Wibb_mean(2);-IMU_Data(count,4)-Wibb_mean(3)];  % deg/s
    Fb=[IMU_Data(count,3);-IMU_Data(count,2);IMU_Data(count,4)]; %m/s/s  2011-11-12  ���ö��ļ�������ʽ���н�������
    Wibb=[IMU_Data(count,6);-IMU_Data(count,5);IMU_Data(count,7)]/deg_rad-Wibb_mean;  % deg/s/deg_rad
    %Wibb=Wibb-Wibb_const;
   
%     attiN(3,1)=Atti_MTiG(count,4);
%    if (attiN(3,1)<0)
%       attiN(3,1)=attiN(3,1)+360;
%    end  
    
   [attiN,WnbbA_old]=atti_cal_cq_modi(T,Wibb,attiN,veloN,posiN,WnbbA_old);% ��Ԫ���㷨(����Чת��ʸ��������%2010-11-25
      %��̬�Ƕ����   
      
%   if(rem(t,50)<0.1)    %ÿ��50��ʹ��1�εش�����
%    mag_sig_b=[IMU_Data(count,8);-IMU_Data(count,7);IMU_Data(count,9)];
%    [attiN]=mag_head(attiN,mag_sig_b); 
%   end      
%  
 
  [veloN]=velo_cal(T,Fb,attiN,veloN,posiN);%2010-11-25
      %�����任
   
  [posiN]=posi_cal(T,veloN,posiN);
      %��λ����
  
  t=t+T;
 
  T_M = T_M + T; kflag = 0; 
        
  
  if( T_M >= T_D )  
      [posiG,veloG]=simu_gps(t,posi,atti,veloB);
        %GPS���
        
      T_M = 0.0; kflag = 1; 
      
      %Xc = zeros(18,1); %�ջ�����У����������������ֵΪ0
      
      [Xc,PK,Xerr]=kalm_gps_15D(t,T_D,Fb,attiN,veloN,posiN,posiG,veloG,Xc,PK,Xerr,kflag);
         %�������˲�   
     
%      [attiN,veloN,posiN]=kalm_modi(attiN,veloN,posiN,Xc);
%         %�����˲�����
%       

        
  GPSData=[GPSData;t,posiG',veloG'];
  KALData = [KALData;t,Xerr];
  ErrData = [ErrData;t,Xc'];
         %�����������   
                 
  TraceData=[TraceData;t,posi',veloB(2,1),atti'];
  IMUData=[IMUData;t,3600.0*Wibb',1/g*Fb'];
  IMUData_MOD=[IMUData_MOD;t,3600.0*(Wibb-Gyro_modi)',1/g*(Fb-Acc_modi)'];%У�����IMU���
  SinsData=[SinsData;t,attiN',veloN',posiN'];
      %�����������
  count=count+1;  
  
  end
end

[attiN,veloN,posiN]=kalm_modi(attiN,veloN,posiN,Xc);%%��׼����̬���ٶȡ�λ�����һ����У������ɾ���׼

% attiN(3,1)=Atti_MTiG(count,4);%���ô������������ĺ�����Ϊ��ʼ����
% if (attiN(3,1)<0)
%       attiN(3,1)=attiN(3,1)+360;
%     end  

Gyro_modi(1,1) = Xc(10,1)/deg_rad; %���������У��������λ����/s
Gyro_modi(2,1) = Xc(11,1)/deg_rad; %��λ����/s
%Gyro_modi(3,1) = Xc(12,1)/deg_rad; %��λ����/s
         %����������
         
 %Acc_modi(1,1) = Xc(13,1);%���ٶȼ����У��������λ��m/s/s
 %Acc_modi(2,1) = Xc(14,1);%��λ��m/s/s
 Acc_modi(3,1) = Xc(15,1);%��λ��m/s/s     
          %���ٶȼ�������


% disp('*******��ʼ�����������λ����\Сʱ,��/��/�룩*********');
% disp('���ݿ̶�ϵ���Ͱ�װ��� | ���ٶȼƿ̶�ϵ���Ͱ�װ���');
% disp([Gyro_fix/deg_rad*3600.0,Acc_fix]);
% 
% disp('*******��ʼ���������λ����\Сʱ,��/��/�룩*********');
% disp('�����������������һ������ɷ����ݰ�����  | ���ٶ�һ������ɷ�');
% disp([Gyro_b/deg_rad*3600.0,Gyro_r/deg_rad*3600.0,Gyro_wg/deg_rad*3600.0,Acc_r]);
% disp('*******');

%  attiN(3,1)=250; attiN(2,1)=-20; attiN(1,1)=-2.5;
%  attiN(3,1)=275; attiN(2,1)=-19; attiN(1,1)=0;
%  
[Xc,PK,Xerr]=kalm_gps_init_15D(posiN,Xc,PK,Xerr);%�������˲����ٴγ�ʼ��

while t<=t_stop  %������������ϵ������̿�ʼ
         
%     Fb=[-IMU_Data(count,6);-IMU_Data(count,5);IMU_Data(count,7)]; %m/s/s  ���ö��ļ�������ʽ���н�������
%     Wibb=[IMU_Data(count,3)-Wibb_mean(1);IMU_Data(count,2)-Wibb_mean(2);-IMU_Data(count,4)-Wibb_mean(3)];  % deg/s
    Fb=[IMU_Data(count,3);-IMU_Data(count,2);IMU_Data(count,4)]-Acc_modi; %m/s/s  2011-11-12  ���ö��ļ�������ʽ���н�������
%     Wibb=[IMU_Data(count,6);-IMU_Data(count,5);IMU_Data(count,7)]/deg_rad-Wibb_mean-Gyro_modi;  % deg/s
    %Wibb=Wibb-Wibb_const;
%     Wibb_mean(1)=-3.04241; Wibb_mean(2)=0.83652; Wibb_mean(3)=1.26051; 
     Wibb_mean(1)=0; Wibb_mean(2)=0; Wibb_mean(3)=0; 
     Wibb=[IMU_Data(count,6);-IMU_Data(count,5);IMU_Data(count,7)]/deg_rad-Gyro_modi-Wibb_mean;  % deg/s
   
  %[attiN]=atti_cal(T,Wibb,attiN,veloN,posiN);    % ���������㷨
  %[attiN]=atti_cal_cq(T,Wibb,attiN,veloN,posiN); % ��Ԫ���㷨
  [attiN,WnbbA_old]=atti_cal_cq_modi(T,Wibb-Gyro_modi_2,attiN,veloN,posiN,WnbbA_old);% ��Ԫ���㷨(����Чת��ʸ��������
      %��̬�Ƕ���⣬��������в��ö�׼�й��Ƶõ���������������У��
%      
%       attiN(3,1)=-Atti_MTiG(count,4);
%       if (attiN(3,1)<0)
%          attiN(3,1)=attiN(3,1)+360;
%       end   
       
  %[attiN,WnbbA_old]=atti_cal_cq_modi(T,Wibb,attiN,veloN,posiN,WnbbA_old);% ��Ԫ���㷨(����Чת��ʸ��������%2010-11-25
      %��̬�Ƕ����   
%  
%   if(rem(t,75)<0.1)    %ÿ��50��ʹ��1�εش�����
%    mag_sig_b=[IMU_Data(count,9);-IMU_Data(count,8);IMU_Data(count,10)];
%    [attiN]=mag_head(attiN,mag_sig_b); 
%  end      

[veloN]=velo_cal(T,Fb-Acc_modi-Acc_modi_2,attiN,veloN,posiN);%λ�ý��㣬������̲��ó�ʼ��׼���Ƶõ��ļ��ٶȼ�������У��
  %[veloN]=velo_cal(T,Fb,attiN,veloN,posiN);%2010-11-25
      %�����任

  [posiN]=posi_cal(T,veloN,posiN);
      %��λ����
  
  t=t+T;
 
  T_M = T_M + T; kflag = 0;  
  Mag=sqrt(Fb(1)^2+Fb(2)^2+Fb(3)^2)-g; 
  
  %if( T_M >= T_D2 )  %2010-11-14�޸� %ֻ���н������㣬�ݲ�������ϵ���
  if( max(Wibb)<= 4.5&&Mag<=0.15)  %2011-11-15�޸� %��̬ʱ������������
      [posiG,veloG]=simu_gps(t,posi,atti,veloB);
%       posiG=[GPS_Data(count_GPS,4);GPS_Data(count_GPS,3);GPS_Data(count_GPS,5)];
%       veloG=[GPS_Data(count_GPS,7);GPS_Data(count_GPS,6);-GPS_Data(count_GPS,8)];
      count_GPS=count_GPS+1;
        %GPS���
        
      T_M = 0.0; kflag = 1; 
      
%       if(t_stop<=62||(t_stop>=74&& t_stop<=90)||(t_stop>=106&& t_stop<=125)||(t_stop>=140&& t_stop<=150)||(t_stop>=170&& t_stop<=184)||(t_stop>=196&& t_stop<=208)||(t_stop>=224&& t_stop<=240))     
%          mag_sig_b=[-IMU_Data(count,9);IMU_Data(count,8);IMU_Data(count,10)];
%          [attiN]=mag_head(attiN,mag_sig_b); 
%       end  
      
     
      Xc = zeros(15,1); %�ջ�����У����������������ֵΪ0
      
      [Xc,PK,Xerr]=kalm_gps_15D(t,T_D2,Fb,attiN,veloN,posiN,posiG,veloG,Xc,PK,Xerr,kflag);
         %�������˲�   
      
%       if(t<=66)  
%         [veloN,posiN]=kalm_modi2(veloN,posiN,Xc);%��У����̬��ֻУ���ٶ���λ��
        %�����˲�����
    
%       else 
      [attiN,veloN,posiN]=kalm_modi(attiN,veloN,posiN,Xc);%ͬʱУ���ٶ���λ��
%       end
       
      
%      attiN(3,1)=-Atti_MTiG(count,4);
%       if (attiN(3,1)<0)
%          attiN(3,1)=attiN(3,1)+360;
%       end     
      
     %[attiN]=align_cal(Wibb,Fb,posiN);  %2011-11-15 ���ü��ٶȼƼ���ˮƽ��̬
     % attiN(3,1)=atti(3,1)+0.5;
      
      Gyro_modi_2(1,1) = Xc(10,1)/deg_rad;
      Gyro_modi_2(2,1) = Xc(11,1)/deg_rad;
      %Gyro_modi_2(3,1) = Xc(12,1)/deg_rad;
         %����������
         
      %Acc_modi_2(1,1) = Xc(13,1);
     % Acc_modi_2(2,1) = Xc(14,1);
      Acc_modi_2(3,1) = Xc(15,1);      
         %���ٶȼ�������
         
      GPSData=[GPSData;t,posiG',veloG'];
      KALData = [KALData;t,Xerr];
      ErrData = [ErrData;t,Xc'];
         %�����������       
  end
       
  TraceData=[TraceData;t,posi',veloB(2,1),atti'];
  IMUData=[IMUData;t,3600.0*Wibb',1/g*Fb'];
  IMUData_MOD=[IMUData_MOD;t,3600.0*(Wibb-Gyro_modi-Gyro_modi_2)',1/g*(Fb-Acc_modi)'];%У��
  %���IMU���
  SinsData=[SinsData;t,attiN',veloN',posiN'];
      %�����������
 count=count+1;
end

%%%%%%%%%%%%%%��������%%%%%%%%%
fig_num=0;

% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(TraceData(:,1),TraceData(:,2));
% subplot(3,1,2);plot(TraceData(:,1),TraceData(:,3));
% subplot(3,1,3);plot(TraceData(:,1),TraceData(:,4));
% xlabel('���к������棨���ȣ��ȣ���γ�ȣ��ȣ����߶ȣ��ף���');

% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(TraceData(:,1),TraceData(:,6));ylabel('����ǣ��㣩');
% subplot(3,1,2);plot(TraceData(:,1),TraceData(:,7));ylabel('�����ǣ��㣩');
% subplot(3,1,3);plot(TraceData(:,1),TraceData(:,8));ylabel('����Ƕȣ��㣩');
% %xlabel('ʱ�䣨t��')
% xlabel('���к������棨����Ƕȣ��ȣ��������Ƕȣ��ȣ�������Ƕȣ��ȣ���');
%    % ��������

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(IMUData(:,1),IMUData(:,2),IMUData_MOD(:,1),IMUData_MOD(:,2),'g');
subplot(3,1,2);plot(IMUData(:,1),IMUData(:,3),IMUData_MOD(:,1),IMUData_MOD(:,3),'g');
subplot(3,1,3);plot(IMUData(:,1),IMUData(:,4),IMUData_MOD(:,1),IMUData_MOD(:,4),'g');
xlabel('IMU���������X�ᣨ��/Сʱ��������Y�ᣨ��/Сʱ��������Z�ᣨ��/Сʱ����');

% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(IMUData_MOD(:,1),IMUData_MOD(:,2),'r');
% subplot(3,1,2);plot(IMUData_MOD(:,1),IMUData_MOD(:,3),'r');
% subplot(3,1,3);plot(IMUData_MOD(:,1),IMUData_MOD(:,4),'r');
% xlabel('IMU���������X�ᣨ��/Сʱ��������Y�ᣨ��/Сʱ��������Z�ᣨ��/Сʱ����');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(IMUData(:,1),IMUData(:,5),IMUData_MOD(:,1),IMUData_MOD(:,5),'g');
subplot(3,1,2);plot(IMUData(:,1),IMUData(:,6),IMUData_MOD(:,1),IMUData_MOD(:,6),'g');
subplot(3,1,3);plot(IMUData(:,1),IMUData(:,7),IMUData_MOD(:,1),IMUData_MOD(:,7),'g');
xlabel('IMU������ӱ�X��g�����ӱ�Y�ᣨg�����ӱ�Z�ᣨg����');
   %IMU����

% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(GPSData(:,1),GPSData(:,2));
% subplot(3,1,2);plot(GPSData(:,1),GPSData(:,3));
% subplot(3,1,3);plot(GPSData(:,1),GPSData(:,4));
% xlabel('GPS���棨���ȣ��ȣ���γ�ȣ��ȣ����߶ȣ��ף���');

% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(GPSData(:,1),GPSData(:,5));
% subplot(3,1,2);plot(GPSData(:,1),GPSData(:,6));
% subplot(3,1,3);plot(GPSData(:,1),GPSData(:,7));
% xlabel('GPS���棨�����ٶȣ���/�룩�������ٶȣ���/�룩�������ٶȣ���/�룩��');
%    %GPS����

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(SinsData(:,1),SinsData(:,2));ylabel('����ǣ��㣩');
subplot(3,1,2);plot(SinsData(:,1),SinsData(:,3));ylabel('�����ǣ��㣩');
subplot(3,1,3);plot(SinsData(:,1),SinsData(:,4));ylabel('����Ƕȣ��㣩');
xlabel('ʱ�䣨t��')
%xlabel('������̬�����������Ƕȣ��ȣ��������Ƕȣ��ȣ�������Ƕȣ��ȣ���');
   %��̬�Ƕ�����
   
fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(SinsData(:,1),SinsData(:,5));
subplot(3,1,2);plot(SinsData(:,1),SinsData(:,6));
subplot(3,1,3);plot(SinsData(:,1),SinsData(:,7));
xlabel('�����ٶ������������/�룩��������/�룩��������/�룩��');
   %�ٶ�����

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(SinsData(:,1),SinsData(:,8));
subplot(3,1,2);plot(SinsData(:,1),SinsData(:,9));
subplot(3,1,3);plot(SinsData(:,1),SinsData(:,10));
xlabel('�������������ȣ��ȣ���γ�ȣ��ȣ����߶ȣ��ף���');
   %λ������

   
figure(14); plot(SinsData(:,8),SinsData(:,9));
figure(13);plot3(SinsData(:,8),SinsData(:,9),SinsData(:,10)) ;
grid on;
% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(TraceData(:,1),SinsData(:,2)-TraceData(:,6));
% subplot(3,1,2);plot(TraceData(:,1),SinsData(:,3)-TraceData(:,7));
% subplot(3,1,3);plot(TraceData(:,1),SinsData(:,4)-TraceData(:,8));
% xlabel('��̬�Ƕ�������Ƕ����ȣ��������Ƕ����ȣ�������Ƕ����ȣ���');
%    %������̬�Ƕ��������    
   
% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(TraceData(:,2),TraceData(:,3),'r',SinsData(:,8),SinsData(:,9));
% subplot(3,1,2);plot(TraceData(:,1),SinsData(:,8)-TraceData(:,2));
% subplot(3,1,3);plot(TraceData(:,1),SinsData(:,9)-TraceData(:,3));
% xlabel('�ߵ���γ��λ�öԱȼ���������ߣ���γ��λ�ã��ȣ����������ȣ���γ�����ȣ���');
%    %����λ�ù켣���������
   
fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(TraceData(:,1),SinsData(:,8)-TraceData(:,2));
subplot(3,1,2);plot(TraceData(:,1),SinsData(:,9)-TraceData(:,3));
subplot(3,1,3);plot(TraceData(:,1),SinsData(:,10)-TraceData(:,4));
 xlabel('�ߵ���γ��λ�öԱȼ���������ߣ��������ȣ���γ�����ȣ���,�߶����ף�');
   %����λ�ù켣���������

% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(TraceData(:,2),TraceData(:,3),'r');
% subplot(3,1,2);plot(GPSData(:,2),GPSData(:,3),'g');
% subplot(3,1,3);plot(SinsData(:,8),SinsData(:,9));
% %xlabel('GPS��γ��λ�öԱȼ���������ߣ���γ��λ�ã��ȣ����������ȣ���γ�����ȣ���');
%    %����λ�ù켣���������  
%    
% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(GPSData(:,1),GPSData(:,2),'r',SinsData(:,1),SinsData(:,8));
% subplot(3,1,2);plot(GPSData(:,1),GPSData(:,3),'r',SinsData(:,1),SinsData(:,9));
% subplot(3,1,3);plot(GPSData(:,1),GPSData(:,4),'r',SinsData(:,1),SinsData(:,10));
% xlabel('��ϵ�����GPS�Աȣ����ȣ��ȣ���γ�ȣ��ȣ����߶ȣ��ף���');
% 
% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(GPSData(:,1),GPSData(:,5),'r',SinsData(:,1),SinsData(:,5));
% subplot(3,1,2);plot(GPSData(:,1),GPSData(:,6),'r',SinsData(:,1),SinsData(:,6));
% subplot(3,1,3);plot(GPSData(:,1),GPSData(:,7),'r',SinsData(:,1),SinsData(:,7));
% xlabel('��ϵ�����GPS�Աȣ������ٶȣ���/�룩�������ٶȣ���/�룩�������ٶȣ���/�룩��');
%     %��ϵ�����GPS�Ա����
% 
fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(KALData(:,1),KALData(:,2));
subplot(3,1,2);plot(KALData(:,1),KALData(:,3));
subplot(3,1,3);plot(KALData(:,1),KALData(:,4));
xlabel('�������˲������ƽ̨���ǣ��룩��');
% 
fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(KALData(:,1),KALData(:,5));
subplot(3,1,2);plot(KALData(:,1),KALData(:,6));
subplot(3,1,3);plot(KALData(:,1),KALData(:,7));
xlabel('�������˲�������ٶ�����/�룩��');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(KALData(:,1),KALData(:,9));
subplot(3,1,2);plot(KALData(:,1),KALData(:,8));
subplot(3,1,3);plot(KALData(:,1),KALData(:,10));
xlabel('�������˲������λ�����ף���');
% 
fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(KALData(:,1),KALData(:,11));
subplot(3,1,2);plot(KALData(:,1),KALData(:,12));
subplot(3,1,3);plot(KALData(:,1),KALData(:,13));
xlabel('���������������/Сʱ��');
% 
% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(KALData(:,1),KALData(:,14));
% subplot(3,1,2);plot(KALData(:,1),KALData(:,15));
% subplot(3,1,3);plot(KALData(:,1),KALData(:,16));
% xlabel('����һ������ɷ�����/Сʱ��');
% 
fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(KALData(:,1),KALData(:,14));
subplot(3,1,2);plot(KALData(:,1),KALData(:,15));
subplot(3,1,3);plot(KALData(:,1),KALData(:,16));
xlabel('���ٶȼ�һ������ɷ���g��');
%    %%%%%%%%%%�������������%%%%%%%%%%%
%    
% %return;
% 
% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(ErrData(:,1),ErrData(:,2)*180.0/pi*3600); %sec
% subplot(3,1,2);plot(ErrData(:,1),ErrData(:,3)*180.0/pi*3600); %sec
% subplot(3,1,3);plot(ErrData(:,1),ErrData(:,4)*180.0/pi*3600); %sec
% xlabel('ƽ̨�����������룩');
% 
% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(ErrData(:,1),ErrData(:,5)); %��/��
% subplot(3,1,2);plot(ErrData(:,1),ErrData(:,6)); %��/��
% subplot(3,1,3);plot(ErrData(:,1),ErrData(:,7)); %��/��
% xlabel('�ٶ�����������/�룩');
% 
% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(ErrData(:,1),ErrData(:,9)*180.0/pi); % deg
% subplot(3,1,2);plot(ErrData(:,1),ErrData(:,8)*180.0/pi); % deg
% subplot(3,1,3);plot(ErrData(:,1),ErrData(:,10));         % m
% xlabel('λ��������(���ȣ��ȣ���γ�ȣ��ȣ����߶ȣ��ף�)');
% 
% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(ErrData(:,1),ErrData(:,11)*180.0/pi*3600); %deg/h
% subplot(3,1,2);plot(ErrData(:,1),ErrData(:,12)*180.0/pi*3600); %deg/h
% subplot(3,1,3);plot(ErrData(:,1),ErrData(:,13)*180.0/pi*3600); %deg/h
% xlabel('���������������������/Сʱ��');
% 
% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(ErrData(:,1),ErrData(:,14)*180.0/pi*3600); %deg/h
% subplot(3,1,2);plot(ErrData(:,1),ErrData(:,15)*180.0/pi*3600); %deg/h
% subplot(3,1,3);plot(ErrData(:,1),ErrData(:,16)*180.0/pi*3600); %deg/h
% xlabel('����һ������ɷ�����������/Сʱ��');
% 
% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(ErrData(:,1),ErrData(:,17)/g); %g
% subplot(3,1,2);plot(ErrData(:,1),ErrData(:,18)/g); %g
% subplot(3,1,3);plot(ErrData(:,1),ErrData(:,19)/g); %g
% xlabel('���ٶȼ�һ������ɷ���������g��');
%   %�������˲���������

  Re=6378137.0;  %����뾶���ף� 
  f=1/298.257;   %�������Բ��
  long=posiN(1,1)*pi/180.0;lati=posiN(2,1)*pi/180.0;heig=posiN(3,1);%������λ��
  Rm=Re*(1-2*f+3*f*sin(lati)*sin(lati));
  Rn=Re*(1+f*sin(lati)*sin(lati)); %�������ʰ뾶���
    
fig_num = fig_num+1;figure(fig_num);
plot((SinsData(:,8)-118.855015)*pi/180*(Rn+posi(3,1))*cos(lati),(SinsData(:,9)-32.028929)*pi/180*(Rm+posi(3,1)));
% plot(SinsData(:,8),SinsData(:,9));
% %%%%%%%%%%%%%�洢��������%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% save trace.dat TraceData -ASCII;    %�洢��������
% save imu.dat   IMUData   -ASCII;    %�洢���������IMU����
% save sins.dat  SinsData  -ASCII;    %�洢�����������
% save kal.dat   KALData   -ASCII;    %�洢�������˲�Щ������
% save err.dat   ErrData   -ASCII;    %�洢�������˲����Ƶ��������ֵ



Re=6378137.0;                                      %����뾶���ף� 
f=1/298.257;                                        %�������Բ��
long=posi(1,1)*pi/180.0;lati=posi(2,1)*pi/180.0;

Rm=Re*(1-2*f+3*f*sin(lati)*sin(lati));
Rn=Re*(1+f*sin(lati)*sin(lati));

% clc;
% disp('������ ')
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
% disp('γ���� ')
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