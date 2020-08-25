clc
clear
close all

% BucketTipLinearPlanningMATData = load('BucketTipLinearPlanningMAT.mat');
BucketTipLinearPlanningMATData = load('shortesttest2.mat');
BucketTipLinearPlanningMATData = BucketTipLinearPlanningMATData.Sequence;
BucketTipLinearPlanningMATData = BucketTipLinearPlanningMATData';

% BucketTipLinearPlanningTXTData = load('BucketTipLinearPlanning.txt');
BucketTipLinearPlanningTXTData = load('testshortest2.txt');

BucketTipLinearPlanningMATData-BucketTipLinearPlanningTXTData