function [attiN]=mag_head(attiN,mag_sig_b)

roll=attiN(1,1)*pi/180;
pitch=attiN(2,1)*pi/180;


C1=[cos(roll),0,-sin(roll);
 0,       1,     0;
 sin(roll),0,  cos(roll)];

C2=[1,   0,   0;
    0, cos(pitch),sin(pitch);
    0,-sin(pitch),cos(pitch)];

mag_sig_n=inv(C1*C2)*mag_sig_b;

attiN_abs=abs(atan(mag_sig_n(1,1)/mag_sig_n(2,1))*180/pi);%求取航向的反三角函数绝对值

% attiN(3,1)=attiN_abs;
if(mag_sig_n(1,1)<-0.0&&mag_sig_n(2,1)>0.0)%象限判断
    attiN(3,1)=attiN_abs;%第一象限
elseif(mag_sig_n(1,1)>0.0&&mag_sig_n(2,1)>0.0)   
    attiN(3,1)=360-attiN_abs;%第二象限
elseif(mag_sig_n(1,1)>0.0&&mag_sig_n(2,1)<-0.0)
    attiN(3,1)=180+attiN_abs;%第三象限
elseif(mag_sig_n(1,1)<-0.0&&mag_sig_n(2,1)<-0.0)
    attiN(3,1)=180-attiN_abs;%第四象限
end
    
    

