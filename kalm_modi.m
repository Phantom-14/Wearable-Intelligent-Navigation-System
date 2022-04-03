%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%                   �������˲���������(����GPS����λ���ٶ����)
%
%  �������:
%           attiN-�����������̬�ǶȺ�������������򣨶ȣ��ȣ��ȣ���
%           veloN-��������Ի���ϵ���˶��ٶȶ��򡢱���������/�룩��
%           posiN-������������ķ�����λ�ã����ȣ��ȣ���γ�ȣ��ȣ����߶ȣ��ף����� 
%           Xc-�ۺ�ģ��״̬�������������
%  ���������         
%           attiN-������ĵ����������̬�ǶȺ�������������򣨶ȣ��ȣ��ȣ���
%           veloN-������ķ�������Ի���ϵ���˶��ٶȶ��򡢱���������/�룩��
%           posiN-������ĵ�����������ķ�����λ�ã����ȣ��ȣ���γ�ȣ��ȣ����߶ȣ��ף����� 
%
%                           ������ƣ�����  ���ڣ�2003/10/05
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [attiN,veloN,posiN]=kalm_modi(attiN,veloN,posiN,Xc)



  veloN(1,1) = veloN(1,1)-Xc(4,1);  % m/s
  veloN(2,1) = veloN(2,1)-Xc(5,1);
  veloN(3,1) = veloN(3,1)-Xc(6,1);  
    %�������ٶ�����

  posiN(1,1) = posiN(1,1) - Xc(8,1)*180.0/pi; % deg
  posiN(2,1) = posiN(2,1) - Xc(7,1)*180.0/pi; % deg
  posiN(3,1) = posiN(3,1) - Xc(9,1);          % m
    %������λ������
  
  %return;
  
  roll=attiN(1,1)*pi/180.0;pitch=attiN(2,1)*pi/180.0;head=attiN(3,1)*pi/180.0;
    
  Cbc=[cos(roll)*cos(head)+sin(roll)*sin(pitch)*sin(head), -cos(roll)*sin(head)+sin(roll)*sin(pitch)*cos(head), -sin(roll)*cos(pitch);
       cos(pitch)*sin(head),                               cos(pitch)*cos(head),                                sin(pitch);
       sin(roll)*cos(head)-cos(roll)*sin(pitch)*sin(head), -sin(roll)*sin(head)-cos(roll)*sin(pitch)*cos(head), cos(roll)*cos(pitch)];
  %����ϵC-->B(����ϵ����>����ϵ)
  
  Ccn=[ 1,       Xc(3,1),  -Xc(2,1);
       -Xc(3,1), 1          Xc(1,1);
        Xc(2,1), -Xc(1,1), 1        ];
    %��������N-->C(��������ϵ����>��������ϵ)
  
  Cbn =( Ccn'*Cbc' )';
    %��̬��������N--->B

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %����̬(���������������
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  attiN(1,1)=atan(-Cbn(1,3)/Cbn(3,3));
  attiN(2,1)=atan(Cbn(2,3)/sqrt(Cbn(2,1)*Cbn(2,1)+Cbn(2,2)*Cbn(2,2)));
  attiN(3,1)=atan(Cbn(2,1)/Cbn(2,2));
    %��λ������

  %�����ж�
  attiN(1,1)=attiN(1,1)*180.0/pi;
  attiN(2,1)=attiN(2,1)*180.0/pi;
  attiN(3,1)=attiN(3,1)*180.0/pi;
    % ��λ����

  if(Cbn(2,2)<0 ) 
   attiN(3,1)=180.0+attiN(3,1);
  else 
   if(Cbn(2,1)<0) attiN(3,1)=360.0+attiN(3,1); end
  end
    %����Ƕȣ���λ���ȣ�

  if(Cbn(3,3)<0)
   if(Cbn(1,3)>0) attiN(1,1)=180.0-attiN(1,1); end
   if(Cbn(1,3)<0) attiN(1,1)=-(180.0+attiN(1,1)); end
  end
    %����Ƕȣ���λ���ȣ�


