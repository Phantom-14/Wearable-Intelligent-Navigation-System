clear;
close all
thighIMU_Data=load('imu_datathigh.txt');
IMU_Data=load('footdata.txt');
% Atti_MTiG=load('MT_01500291_007-001.txt');
%GPS_Data=load('GPS data 12-28.txt');

%t_stop=length(IMU_Data(:,1))*T;
%%%%%%%%仿真时间设置%%%%%%
t=0;
T=0.01;
t_stop=length(IMU_Data(:,1))/100;

%%%%%%%%常数设置%%%%%%%%%%%
g=9.7803698;         %重力加速度    （单位：米/秒/秒）
deg_rad=0.01745329252e0;% Transfer from angle degree to rad
Acc_modi=[0;0;-0.0588]; 
Wibb_mean=[0.2977;0.1421;-0.0060]; 
Gyro_modi=[0.000467590232943351;0.00485341878015159;0]; 
Number=[];
dyfoot=[];
dythigh=[];
stfoot=[];
stthigh=[];

t=0.01; 
count=1;   %计数
                                                              %%%%%%%%动静态分离设置%%%%%%%%%%%
while  t<=t_stop  %解算开始，过程计数
    
    
    Fb=[IMU_Data(count,3);-IMU_Data(count,2);IMU_Data(count,4)]; %m/s/s  2011-11-12  采用读文件数据形式进行捷联解算-Acc_modi
    Wibb=[IMU_Data(count,6);-IMU_Data(count,5);IMU_Data(count,7)]/deg_rad;  % deg/s-Wibb_mean-Gyro_modi  ||a<=-8||b>=8||c>=20
    Mag=sqrt(Fb(1)^2+Fb(2)^2+Fb(3)^2)-g;  
    a=Fb(1);b=Fb(2);c=Fb(3);
%    Mag=sqrt(Fb(1)^2+Fb(2)^2+Fb(3)^2)-g; 
    if( max(Wibb)<= 4.5&&Mag<=0.15||c<=-9||c>=19)  Number=[Number;0];    %------------------------
    else Number=[Number;count];
    end   
    t=t+T;
    count=count+1;  
    
%     Fb=[-IMU_Data(count,6);-IMU_Data(count,5);IMU_Data(count,7)]; %m/s/s  采用读文件数据形式进行捷联解算
%     Wibb=[IMU_Data(count,3);IMU_Data(count,2);-IMU_Data(count,4)];  %°/s
   
%     attiN(3,1)=Atti_MTiG(count,4);
%     if (attiN(3,1)<0)
%       attiN(3,1)=attiN(3,1)+360;
%     end  
 
%       if(rem(t,50)<0.1)    %每隔50秒使用1次地磁数据
%    mag_sig_b=[IMU_Data(count,8);-IMU_Data(count,7);IMU_Data(count,9)];
%    [attiN]=mag_head(attiN,mag_sig_b); 
%       end      
 
%     tmp_Fb = tmp_Fb+Fb;
%     tmp_Wibb = tmp_Wibb+Wibb;
%     kc=kc+1;

end


for i=1:length(IMU_Data(:,1))
   if  Number(i,1)==0 stfoot=[stfoot;IMU_Data(i,1:7)];
   else dyfoot=[dyfoot;IMU_Data(i,1:7)];
   end
end

for i=1:length(IMU_Data(:,1))
   if  Number(i,1)==0 stthigh=[stthigh;thighIMU_Data(i,1:7)];
   else  dythigh=[dythigh;thighIMU_Data(i,1:7)];
   end
end




load('-mat','aaa');
input_train=dythigh(1:7000,2:7)';               %-----------------
output_train=dyfoot(1:7000,2:7)';               %-----------------

input_test=dythigh(:,2:7)';
output_test=dyfoot(:,2:7)';
[inputn,inputps]=mapminmax(input_train);
[outputn,outputps]=mapminmax(output_train);


% 测试数据归一化
 [inputn_test,inputps]=mapminmax(input_test);
%网络预测输出
an=sim(net,inputn_test);
%网络输出反归一化
BPoutput=mapminmax('reverse',an,outputps);
% t1=[1:1:length(BPoutput)]/100;
% 结果分析
% figure(1);plot(t1,output_test(1,:),t1,BPoutput(1,:),'r');figure(2);plot(t1,output_test(2,:),t1,BPoutput(2,:),'r');figure(3);plot(t1,output_test(3,:),t1,BPoutput(3,:),'r');
% figure(4);plot(t1,output_test(4,:),t1,BPoutput(4,:),'r');figure(5);plot(t1,output_test(5,:),t1,BPoutput(5,:),'r');figure(6);plot(t1,output_test(6,:),t1,BPoutput(6,:),'r');

                                                    %%%%%%%虚拟足部传感数据构建及绘图对比%%%%%%%%%%%
first=dythigh(:,1);
out=[first BPoutput'];
% t1=[1:1:length(out)]/100;
% figure(1);plot(t1,output_test(:,2),t1,out(:,2),'r');figure(2);plot(t1,output_test(:,3),t1,out(:,3),'r');figure(3);plot(t1,output_test(:,4),t1,out(:,4),'r');
% figure(4);plot(t1,output_test(:,5),t1,out(:,5),'r');figure(5);plot(t1,output_test(:,6),t1,out(:,6),'r');figure(6);plot(t1,output_test(:,7),t1,out(:,7),'r');
% dlmwrite('dy-vir.txt',output,'delimiter','\t','precision','%.6f'); 

%载入输出和输入数据
dyvir=out;
admix=[dyvir;stfoot];
sorting= sortrows(admix,1);
virtual=sorting;
% virtual(:,1) = [];
dlmwrite('virtual.txt',virtual,'delimiter','\t','precision','%.6f');

t=[1:1:length(IMU_Data)]/100;
figure(1);plot(t,IMU_Data(:,2),t,virtual(:,2),'r');figure(2);plot(t,IMU_Data(:,3),t,virtual(:,3),'r');figure(3);plot(t,IMU_Data(:,4),t,virtual(:,4),'r');
figure(4);plot(t,IMU_Data(:,5),t,virtual(:,5),'r');figure(5);plot(t,IMU_Data(:,6),t,virtual(:,6),'r');figure(6);plot(t,IMU_Data(:,7),t,virtual(:,7),'r');

