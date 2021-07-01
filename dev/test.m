%% preamble
close all; clear; clc;
addpath("~/AHRS/src/");

%% create arduino and MPU9250 objects
a = arduino('/dev/ttyACM0', 'Uno', 'Libraries', 'I2C');
ts = 0.01;  % sample time
fs = 1/ts;  % sample frequency (rate)
imu = mpu9250(a, 'SampleRate', fs, 'SamplesPerRead', 1, ...
    'OutputFormat', 'matrix', 'ReadMode', 'Latest');

%% create ekf object
p = EKF;
p.SetIMU(imu);

%% run test
ui = p.RunUI();
