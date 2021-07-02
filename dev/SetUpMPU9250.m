function [a, imu] = SetUpMPU9250(ts)
    % get path to functions
    addpath("~/AHRS/src/");
    
    % set up arduino and return obbject
    a = arduino('/dev/ttyACM0', 'Uno', 'Libraries', 'I2C');
    
    % sample rate
    fs = 1/ts;  % sample frequency (rate)
    
    % set up MPU-9250 and return object
    imu = mpu9250(a, 'SampleRate', fs, 'SamplesPerRead', 1, ...
        'OutputFormat', 'matrix', 'ReadMode', 'Latest');
end