%% preamble
[a, imu] = SetUpMPU9250(0.01);

%% create ekf object
p = EKF;
p.SetIMU(imu);

%% run test
ui = p.RunUI();
