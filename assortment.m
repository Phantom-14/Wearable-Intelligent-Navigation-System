                                                              
                                                               %%%%%%%%��ʼ����%%%%%%%%%%%
clear;
close all
thighIMU_Data=load('imu_datathigh.txt');
IMU_Data=load('footdata.txt');
%%%%%%%%����ʱ������%%%%%%
t=0;
T=0.01;
t_stop=length(IMU_Data(:,1))/100;

%%%%%%%%��������%%%%%%%%%%%
g=9.7803698;         %�������ٶ�    ����λ����/��/�룩
deg_rad=0.01745329252e0;% Transfer from angle degree to rad
Number=[];
dyfoot=[];
dythigh=[];
stfoot=[];
stthigh=[];

t=0.01; 
count=1;   %����
                                                              %%%%%%%%����̬��������%%%%%%%%%%%
while  t<=t_stop  %���㿪ʼ�����̼���
    
    
    Fb=[IMU_Data(count,3);-IMU_Data(count,2);IMU_Data(count,4)]; %m/s/s  2011-11-12  ���ö��ļ�������ʽ���н�������-Acc_modi
    Wibb=[IMU_Data(count,6);-IMU_Data(count,5);IMU_Data(count,7)]/deg_rad;  % deg/s-Wibb_mean-Gyro_modi  ||a<=-8||b>=8||c>=20
    Mag=sqrt(Fb(1)^2+Fb(2)^2+Fb(3)^2)-g;  
    a=Fb(1);b=Fb(2);c=Fb(3); 
    if( max(Wibb)<= 4.5&&Mag<=0.15||c<=-9||c>=19)  Number=[Number;0];    %------------------------
    else Number=[Number;count];
    end   
    t=t+T;
    count=count+1;  
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
%�ҳ�ѵ�����ݺ�Ԥ������
input_train=dythigh(1:7000,2:7)';               %-----------------
output_train=dyfoot(1:7000,2:7)';               %-----------------

input_test=dythigh(:,2:7)';
output_test=dyfoot(:,2:7)';

%ѵ����������������ݹ�һ��
[inputn,inputps]=mapminmax(input_train);
[outputn,outputps]=mapminmax(output_train);

% BP����ѵ��
net=newff(inputn,outputn,35,{'tansig','purelin'},'trainlm');  
net.trainParam.show=25;% ����25����ʾһ��
net.trainParam.epochs=300;% ��������300��
net.trainParam.lr=0.01;% learn rate ѧϰ����
net.trainParam.goal=0.00001;% ����ָ��mse
net.trainParam.mc=0.9;% ����ָ��

%����ѵ��
[net,tr]=train(net,inputn,outputn);

save('aaa','net');
%BP����Ԥ�⣬Ԥ�����ݹ�һ��
inputn_test=mapminmax('apply',input_test,inputps);
%����Ԥ�����
an=sim(net,inputn_test);
%�����������һ��
BPoutput=mapminmax('reverse',an,outputps);
% t1=[1:1:length(BPoutput)]/100;
% �������
                                                    %%%%%%%�����㲿�������ݹ�������ͼ�Ա�%%%%%%%%%%%
first=dythigh(:,1);
out=[first BPoutput'];
%�����������������
dyvir=out;
admix=[dyvir;stfoot];
sorting= sortrows(admix,1);
virtual=sorting;
dlmwrite('virtual.txt',virtual,'delimiter','\t','precision','%.6f');

t=[1:1:length(IMU_Data)]/100;
figure(1);plot(t,IMU_Data(:,2),t,virtual(:,2),'r');figure(2);plot(t,IMU_Data(:,3),t,virtual(:,3),'r');figure(3);plot(t,IMU_Data(:,4),t,virtual(:,4),'r');
figure(4);plot(t,IMU_Data(:,5),t,virtual(:,5),'r');figure(5);plot(t,IMU_Data(:,6),t,virtual(:,6),'r');figure(6);plot(t,IMU_Data(:,7),t,virtual(:,7),'r');

