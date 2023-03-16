function [a, imu] = SetUpMPU9250(ts)
    % set up arduino and return obbject
    a = arduino('COM5', 'Uno', 'Libraries', 'I2C');
    
    % sample rate
    fs = 1/ts;  % sample frequency (rate)
    
    % set up MPU-9250 and return object
    imu = mpu9250(a, 'SampleRate', fs, 'SamplesPerRead', 1, ...
        'OutputFormat', 'matrix', 'ReadMode', 'Latest');
end