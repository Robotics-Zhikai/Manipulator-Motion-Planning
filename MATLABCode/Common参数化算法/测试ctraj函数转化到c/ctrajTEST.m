clc
clear
close all

%ctraj_c �������� ��ת��c���Ƿ���bug

ctrajDATA_c = load ('ctrajTEST.txt'); %����ת����c���������

ctrajDATA_Matlab = load('ctrajTESTMATLABData.mat');
ctrajDATA_Matlab = ctrajDATA_Matlab.traj; %����ͬ���ĳ�ĩλ��MATLAB���������

k=1;
for i =1:4:size(ctrajDATA_c,1)
    if norm(ctrajDATA_c(i:i+3,:)-ctrajDATA_Matlab(:,:,k)) > 0.001
        error('c����ת��������');
    end
    k=k+1;
end
disp('c����ת��û����');









