close all;
clear;
clc;

addpath("~/AHRS/src/");
addpath("~/AhrsWithMatlab/Datasets/");
load("moving_dataset.mat");
load("mahony_filtered_dataset.mat");

%%
ahrs = EKF;
rpy = ahrs.Run(accel, gyro, mag);
fig = ahrs.Plot(rpy, rpy_mahony);
path = fullfile('~','AHRS', 'docs', 'offline_ekf_test.png');
saveas(fig, path);