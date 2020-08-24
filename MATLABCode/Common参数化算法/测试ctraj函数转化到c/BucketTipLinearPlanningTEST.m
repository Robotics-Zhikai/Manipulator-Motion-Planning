clc
clear
close all

BucketTipLinearPlanningMATData = load('BucketTipLinearPlanningMAT.mat');
BucketTipLinearPlanningMATData = BucketTipLinearPlanningMATData.Sequence;
BucketTipLinearPlanningMATData = BucketTipLinearPlanningMATData';

BucketTipLinearPlanningTXTData = load('BucketTipLinearPlanning.txt');

BucketTipLinearPlanningMATData-BucketTipLinearPlanningTXTData