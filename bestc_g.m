clc
clear all
butai=load('butai.txt');
butai_label=load('butai_label.txt');
train_butai=[butai(1:2000,3:8);butai(4930:6930,3:8);butai(9931:11931,3:8)];
train_butai_label=[butai_label(1:2000);butai_label(4930:6930);butai_label(9931:11931)];
function[bestacc,bestc,bestg]=SVMcgForClass(train_label,train,cmin,cmax,gmin,gmax,v,cstep,gstep,accstep)
[X,Y]=meshgrid(cmin:cstep:cmax,gmin:gstep:gmax);
[m,n]=size(X);
cg=zeros(m,n);
bestc=0;
bestg=0;
bestacc=0;
basenum=2;
for i=1:m
for j=1:n
    cmd=['-v',num2str(v),'-c',num2str(basenum^X(i,j)),'-g',num2str(basenum^Y(i,j))];
    cg(i,j)=svmtrain(train_label,train,cmd);
          if cg(i,j)>bestacc
              bestacc=cg(i,j);
              bestc=basenum^X(i,j);
              bestg=basenum^Y(i,j);
          end
 eps=10^(-4)
         if abs(cg(i,j)-bestacc)<=eps&&bestc>basenum^X(i,j)
             bestacc=cg(i,j);
             bestc=basenum^X(i,j);
             bestg=basenum^Y(i,j);
         end
end
end
[bestacc,bestc,bestg]=SVMForClass(train_butai_label,train_butai,-10,10,-10,10);
end