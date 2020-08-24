clc
clear
close all

%ctraj_c 函数测试 看转换c后是否有bug

% ctrajDATA_c = load ('ctrajTEST.txt'); %这是转换的c计算的序列
ctrajDATA_c = load ('testshortest.txt');

% ctrajDATA_Matlab = load('ctrajTESTMATLABData.mat');
% ctrajDATA_Matlab = ctrajDATA_Matlab.traj; %这是同样的初末位姿MATLAB计算的序列
ctrajDATA_Matlab = load('shortesttest.mat');
ctrajDATA_Matlab = ctrajDATA_Matlab.Tsequence;

k=1;
for i =1:4:size(ctrajDATA_c,1)
    if norm(ctrajDATA_c(i:i+3,:)-ctrajDATA_Matlab(:,:,k)) > 0.001
%         error('c程序转化有问题');
    end
    
    ctrajDATA_c(i:i+3,:)-ctrajDATA_Matlab(:,:,k)
    plot(k,norm(ctrajDATA_c(i:i+3,:)-ctrajDATA_Matlab(:,:,k)),'.');
    hold on
    pause(0.1);
    k=k+1;
end
% disp('c程序转换没问题');









