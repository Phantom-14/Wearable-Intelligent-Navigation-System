clc
clear all
butai=load('butai.txt');
butai_label=load('butai_label.txt');
train_butai=[butai(1:2000,3:8);butai(4930:6930,3:8);butai(9931:11931,3:8)];
train_butai_label=[butai_label(1:2000);butai_label(4930:6930);butai_label(9931:11931)];
test_butai=[butai(2001:4929,3:8);butai(6031:9930,3:8);butai(11932:14931,3:8)];
test_butai_label=[butai_label(2001:4929);butai_label(6031:9930);butai_label(11932:14931)];
[mtrain,ntrain]=size(train_butai);
[mtest,ntest]=size(test_butai);
dataset=[train_butai;test_butai];
[dataset_scale,ps]=mapminmax(dataset',0,1);
dataset_scale=dataset_scale';
train_butai=dataset_scale(1:mtrain,:);
test_butai=dataset_scale((mtrain+1):(mtrain+mtest),:);
model=svmtrain(train_butai_label,train_butai,'-c 2048 -g 0.07 -v 10');
[predicted_label,accuracy]=svmpredict(test_butai_label,test_butai, model);