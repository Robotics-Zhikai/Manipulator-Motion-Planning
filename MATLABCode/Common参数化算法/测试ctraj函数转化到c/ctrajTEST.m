clc
clear
close all

%ctraj_c �������� ��ת��c���Ƿ���bug

% ctrajDATA_c = load ('ctrajTEST.txt'); %����ת����c���������
ctrajDATA_c = load ('testshortest.txt');

% ctrajDATA_Matlab = load('ctrajTESTMATLABData.mat');
% ctrajDATA_Matlab = ctrajDATA_Matlab.traj; %����ͬ���ĳ�ĩλ��MATLAB���������
ctrajDATA_Matlab = load('shortesttest.mat');
ctrajDATA_Matlab = ctrajDATA_Matlab.Tsequence;

k=1;
for i =1:4:size(ctrajDATA_c,1)
    if norm(ctrajDATA_c(i:i+3,:)-ctrajDATA_Matlab(:,:,k)) > 0.001
%         error('c����ת��������');
    end
    
    ctrajDATA_c(i:i+3,:)-ctrajDATA_Matlab(:,:,k)
    plot(k,norm(ctrajDATA_c(i:i+3,:)-ctrajDATA_Matlab(:,:,k)),'.');
    hold on
    pause(0.1);
    k=k+1;
end
% disp('c����ת��û����');









