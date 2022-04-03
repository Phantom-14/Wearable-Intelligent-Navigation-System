%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%                    ��ʼ�ٶȼ������
%
%  �������:
%  atti     %��������������򣨵�λ���ȣ�
%  veloB    %�ɻ��˶��ٶȡ���X����Y��ͷ��Z���򣨵�λ����/�룩
%  ���������
%  veloN    %��������ʼ�ٶ�
%      
%                           ������ƣ�����  ���ڣ�2002/4/15
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [veloN]=veloN0(atti,veloB)

  roll=atti(1,1)*pi/180.0;pitch=atti(2,1)*pi/180.0;head=atti(3,1)*pi/180.0;

  %����ϵN-->B
  Cbn=[cos(roll)*cos(head)+sin(roll)*sin(pitch)*sin(head), -cos(roll)*sin(head)+sin(roll)*sin(pitch)*cos(head), -sin(roll)*cos(pitch);
       cos(pitch)*sin(head),                               cos(pitch)*cos(head),                                sin(pitch);
       sin(roll)*cos(head)-cos(roll)*sin(pitch)*sin(head), -sin(roll)*sin(head)-cos(roll)*sin(pitch)*cos(head), cos(roll)*cos(pitch)];


  veloN=Cbn'*veloB;



