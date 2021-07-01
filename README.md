# aerotinez/AHRS
### Attitude and Heading Reference System algorithms for the Arduino + MPU-9250 & MATLAB
#### Martin L Pryde *MSc*

martinpryde@ymail.com

Bienvenue!/Welcome!

This repository is for anyone wishing to try out the MPU-9250 9-axis IMU + magnetometer connected through Arduino in MATLAB. The main codes can be found in `/src/`.
#### THIS REPO REQUIRES R2019a OR NEWER!

### Getting started
Make sure you have the [MATLAB support package for Arduino hardware](https://uk.mathworks.com/help/supportpkg/arduinoio/index.html?s_tid=CRUX_lftnav) installed. Follow [these tutorials](https://uk.mathworks.com/help/supportpkg/arduinoio/get-started-with-matlab-support-package-for-arduino-hardware.html) to learn how to set up your Arduino in MATLAB and [this tutorial from MathWorks](https://uk.mathworks.com/help/fusion/ug/Estimating-Orientation-Using-Inertial-Sensor-Fusion-and-MPU-9250.html) to learn how to connect your MPU-9250 to your Arduino board. Clone this repository and your ready to go!

### Using the user interface
Each algorithm has a `.RunUI()` method which displays a real-time animation of your connected MPU-9250. For example, to use run UI with the Extended Kalman Filter ahrs you could do:

```matlab
% don't forget to add the source code to your path
addpath("<~/path-to-/AHRS/src/>");

% create arduino and MPU9250 objects
a = arduino('<your-serial-port>', '<your-board>', 'Libraries', 'I2C');
ts = 0.01;  % sample time
fs = 1/ts;  % sample frequency (rate)
imu = mpu9250(a, 'SampleRate', fs, 'SamplesPerRead', 1, 'OutputFormat', 'matrix', 'ReadMode', 'Latest');

% call the ekf
ahrs = EKF();

% input your imu
ahrs.SetIMU(imu);

% call the UI and enjoy!
ahrs.RunUI();
```
### Filtering a dataset offline
Want to filter a set of MPU-9250 readings offline? No problem! Make sure your sensor data is stored in n-by-3 matrices and use the `.Run()` method of the filter you wish to use! The filtered result is output as an n-by-3 matrix of Tait-Bryan/Euler angles in degrees ordered as `[roll, pitch, yaw]`. Make sure to set the sample time for your data in your chosen filter using `ahrs.SetSampleTime(<your sample time here>)`.

```matlab
ahrs = EKF();
filtered_data = ahrs.Run(accel_data, gyro_data, mag_data);
```
Additionally, once you have your filtered data, you can plot it using `.Plot()` and even compare it with data from another filtering method.
```matlab
% plot your filtered data
fig = ahrs.Plot(filtered_data);
clearvars fig;
% plot your most recently filtered data against other baseline data to compare filter performace
fig = ahrs.Plot(filtered_data, other_filtered_data);
```
### MPU-9250 magnetometer calibration
You can calibrate your MPU-9250's magnetometer for hard and soft iron biases by simply calling the `.MagCal()` method of any AHRS algorithm in `/src/`!
```matlab
% ^^ ...setup your arduino and mpu9250 objects as shown before

% choose any filter algorithm
ahrs = EKF();

% input your imu
ahrs.SetIMU(imu);

% call MagCal and follow the instructions in the console for 60 seconds
ahrs.MagCal();

% the resulting calibrations will be given in microTesla for the hard iron
% bias vector and as a 3x3 scaling matrix for the soft iron bias
```
### TODO:
-[x] quaternion-based extended kalman filter `EKF.m`
-[ ] add end & pause buttons to `UI.m` 
-[ ] add bias estimation & compensation to `EKF.m`
-[ ] complementary filter
-[ ] mahony filter
-[ ] madgwick filter
-[ ] UKF?
