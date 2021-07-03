classdef (Abstract) Ahrs < handle
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Attitude and Heading Reference System (AHRS)
    % Martin L Pryde MSc
    % martinpryde@ymail.com
    % https://github.com/aerotinez/AHRS
    %
    % This class seeks to achieve three goals:
    % 1. It should define the layout of all superclass ahrs classes which
    %    inherit from it.
    % 2. It should contain any and all methods common to all ahrs superclasses.
    %    This is to say that any future implementations of common methods should
    %    be implemented HERE.
    % 3. It should contain and implement any calibration tools for example, 
    %    magnetometer calibration.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % user may view this object's property values but must use a .Set() method 
    % to edit them during object lifetime.
    properties (GetAccess = public, SetAccess = protected)
        % MATLAB mpu9250() object used to read measurements from the hardware
        imu;

        % sample time for accelerometer, gyroscope and magnetometer
        sample_time (1,1) double = 0.01;

        % hard iron (additive) bias on magnetometer readings (microTesla)
        hard_iron_bias (1,3) double = [-3.8236, 140.6495, -170.3542];

        % soft iron (scaling) bias on magnetometer readings (dimensionless)
        soft_iron_bias (3,3) double = [1.0034, 0.0112, 0.0142; 0.0112, 1.0593, 0.0186; 0.0142, 0.0186, 0.9414];
    end

    % main methods for all ahrs superclasses
    methods (Abstract)
        % Each superclass must define these three methods
        
        % .Filter() can polymorphically accept a single set of measurements or
        % can accept an nx3 matrix of measurements and output the filtered
        % result as roll-pitch-yaw Tait-Bryan angles in degrees
        Filter(obj);

        % .StartUI() reads the imu in real-time and publishes the filtered
        % meaurements to an animated UIFigure object
        StartUI(obj);

        % .Plot() can take in either a single set of filtered measurements and 
        % plot them in on a figure or can take in two sets of measurements in 
        % order for the user to compare filtering techniques
        Plot(obj);
    end
    
    % set methods
    methods
        % .SetIMU accepts a MATLAB mpu9250 object which the class will use to
        % take real-time measurements from the MPU-9250. Once an mpu9250 has 
        % been assigned, the object sample time is set to match the sample time
        % of the mpu9250
        function SetIMU(obj, imu)
            % imu is a MATLAB mpu9250() objects
            if imu.SamplesPerRead ~= 1
                error("Invalid mpu9250(): <SamplesPerRead> property of input mpu9250() object must be set to 1.");
            elseif imu.OutputFormat ~= "matrix"
                error("Invalid mpu9250(): <OutputFormat> property of input mpu9250() object must be set to <'matrix'>");
            end
            obj.imu = imu;
            obj.sample_time = 1/obj.imu.SampleRate;
        end

        % . SetSampleTime() allows the user to change the object sample time
        % from the default of 0.01. However, the user may not change the sample
        % time if an imu has been set 
        function SetSampleTime(obj, sample_time)
            if isempty(obj.imu)
                obj.sample_time = sample_time;
            else
                error("Cannot set sample time when Ahrs.imu has been set.")
            end
        end

        % SetIronBias(b) sets the Ahrs hard_iron_bias property to <b>
        % SetIronBias(b,A) sets the Ahrs hard_iron_bias and
        % soft_iron_bias properties to <b> and <A> respectively
        function SetIronBias(obj, varargin)
            % number of input arguments minus the obj input
            n = nargin - 1;
            if n == 0
                error('Not enough input arguments');
            elseif n == 1
                obj.hard_iron_bias = varargin{1};
                % display the new hard iron bias in the terminal
                clc;
                fprintf('New Hard Iron Bias:\n\n');
                disp(varargin{1});
            elseif n == 2
                obj.hard_iron_bias = varargin{1};
                obj.soft_iron_bias = varargin{2};
                % display the new hard and soft iron biases in the terminal
                clc;
                fprintf('New Hard Iron Bias:\n\n');
                disp(varargin{1});
                fprintf('New Soft Iron Bias:\n\n');
                disp(varargin{2});
            else
                error('Too many input arguments');
            end
        end
    end
    
    % magnetometer calibration app
    methods (Access = public)
        % .MagCal() allows the user to accurately calibrate their MPU-9250's 
        % magnetometer in one minute! The user must first use the .SetIMU()
        % method to register their MPU-9250 with the class. After calling
        % .MagCal(), a figure appears and prompts the user to move the MPU-9250
        % about its body axes in order to gather readings from which to compute
        % the hard and soft iron biases. The figure is left available to the
        % user after the session has ended in order for them to check if they
        % are satisfied with the quality of the plotted sphereoid (which is to
        % say the distribution of the measurements)
        function MagCal(obj)
            % check that the user has set an mpu9250() object in the class
            if isempty(obj.imu)
                error('Set an imu object in order to begin magnetometer calibration');
            end
            
            % close all other open figures
            close all;

            % extract sample time from imu object
            ts = obj.sample_time;

            % duration of magnetometer calibration experiment
            tf = 60;

            % number of samples to be taken during experiment
            n = tf/ts;
            
            % congfigure plot screen
            fig = figure();
            ax = axes(fig);
            title(ax, "MPU-9250 Magnetometer Calibration Session");
            
            % most recent reading should be plotted as a single red marker
            p = animatedline(ax);
            p.MaximumNumPoints = 1;
            p.Color = 'red';
            p.LineStyle = 'none';
            p.MarkerSize = 8;
            p.Marker = 'O';
            p.MarkerFaceColor = 'red';
            
            % previous readings should be marked as blue markers
            m = nan(n,3); % container for magnetometer readings
            l = line(ax, m(:,1), m(:,2), m(:,3));
            l.Color = 'blue';
            l.LineStyle = 'none';
            l.MarkerSize = 2;
            l.Marker = 'X';
            
            % configure axes
            axis equal;
            grid on;
            hold on;
            view(ax,3);
            
            % begin experiment
            i = 1;
            clc;
            disp("Rotate the MPU-9250 about it's axes in such a manner as to plot a sphereoid on the figure.");
            while i < n + 1
                % read mpu9250
                [~,~,m(i,:)] = read(obj.imu);
                
                % update previous readings line
                l.XData = m(:,1);
                l.YData = m(:,2);
                l.ZData = m(:,3);
                
                % update current reading marker
                x = m(i,1);
                y = m(i,2);
                z = m(i,3);
                addpoints(p, x, y, z);
                
                % draw and increment
                drawnow limitrate;
                i = i + 1;
            end
            hold off;
            clc;
            
            % compute and set object soft & hard iron biases
            [A, b, ~] = magcal(m);
            obj.SetIronBias(b,A);

            % print new iron biases to the terminal
            fprintf('Calibration complete!\n\n');
            fprintf('New Hard Iron Bias:\n\n');
            disp(b);
            fprintf('New Soft Iron Bias:\n\n');
            disp(A);
        end
    end

    % useful quaternion and pre-processing methods for all ahrs superclasses
    methods (Access = protected)
        % .PreprocessMeasurments aligns the axes of readings taken by the
        % MPU-6050 imu SOC on the MPU-9250 board with the magnetometer.
        % WARNING: This method should only be called at the top of the .Filter
        % and .StartUI() methods in order to avoid runtime errors
        function [a, g, m] = PreprocessMeasurements(obj, accel, gyro, mag)
            a = accel;
            g = gyro;
            m = mag;

            % calibrate mag data
            m = (m - obj.hard_iron_bias)*obj.soft_iron_bias;
            
            % align mpu-9250 accel & gyro readings with magnetometer body
            % NED frame
            a = [-a(:,2), -a(:,1), a(:,3)];
            g = [g(:,2), g(:,1), -g(:,3)];
        end

        % .InitQuat uses the tilt algorithm in order to provide an initial
        % estimate of the MPU-9250 orientation before filtering. Read about
        % the tilt algorithm here: 
        % https://ahrs.readthedocs.io/en/latest/filters/tilt.html
        function q = InitQuat(obj, accel, mag)
            % normalize readings
            a = obj.NormVec(accel);
            m = obj.NormVec(mag);
            
            % sort accel readings
            ax = a(1,1);
            ay = a(1,2);
            az = a(1,3);

            % get roll & pitch
            roll = atan2(ay, az);
            pitch = atan2(-ax, ay*sin(roll) + az*cos(roll));

            % compute b-vector
            Cr = cos(roll);
            Sr = sin(roll);
            Cp = cos(pitch);
            Sp = sin(pitch);
            
            R = [Cp, Sp*Sr, Sp*Cr; 0, Cr, -Sr; -Sp, Cp*Sr, Cp*Cr];
            b = R*obj.CheckColVec(m);
            
            % get yaw
            bx = b(1);
            by = b(2);
            yaw = atan2(-by,bx);
            
            % output earth->body quaternion
            q = angle2quat(yaw, pitch, roll, 'ZYX');
            q = obj.CheckRowVec(q);
            q = obj.NormQuat(q);
        end

        % .QuatToDCM() converts earth->body quaternion into an earth->body
        % direction cosine matrix (dcm). Similar to MATLAB quat2dcm()
        function dcm = QuatToDCM(obj, q)
            q = obj.CheckRowVec(q);
            q = obj.QuatNorm(q);

            % q = [qs, qx, qy, qz] where qs is the scalar and qx, qy, and qz are
            % the vector components of the quaternion
            qs = q(1,1);
            qx = q(1,2);
            qy = q(1,3);
            qz = q(1,4);

            % compute dcm elements
            q11 = qx^2 - qy^2 - qz^2 + qs^2;
            q12 = 2*(qx*qy + qz*qs);
            q13 = 2*(qx*qz - qy*qs);
            q21 = 2*(qx*qy - qz*qs);
            q22 = -qx^2 + qy^2 - qz^2 + qs^2;
            q23 = 2*(qy*qz + qs*qx);
            q31 = 2*(qx*qz + qy*qs);
            q32 = 2*(qy*qz - qs*qx);
            q33 = -qx^2 - qy^2 + qz^2 + qs^2;

            % construct dcm
            dcm = [q11, q12, q13;
                q21, q22, q23;
                q31, q32, q33];
        end

        % .QuatToTaitBryan accepts an earth->body quaternion as an argument and 
        % converts it into a 1x3 vector of tait-bryan (ZYX) euler angles in 
        % degrees
        function rpy = QuatToTaitBryan(obj, q)
            % normalize input quaternion
            q = obj.NormQuat(q);
            q = obj.CheckRowVec(q);
            
            % convert to tait-bryan angles
            [yaw, pitch, roll] = quat2angle(q, 'ZYX');
            
            % convert to degrees
            roll = rad2deg(roll);
            pitch = rad2deg(pitch);
            yaw = rad2deg(yaw);

            rpy = [roll, pitch, yaw];
        end
    end
    
    % The following are mostly math-specific or sanity check methods 
    methods (Static, Access = protected)
        % turns a vector of length 3 into a 3x3 skew-symmetric matrix
        function s = Skew(v)
            p = v(1);
            q = v(2);
            r = v(3);
            s = [0, -r, q; r, 0, -p; -q, p, 0];
        end

        % takes in a vector. if the input is already a column vector, the input 
        % is returned unchanged. If the input is a row vector, the vector is 
        % returned as a column vector.
        function v_out = CheckColVec(v_in)
            [rows, columns] = size(v_in);
            if rows == 1
                v_out = v_in.';
            elseif columns == 1
                v_out = v_in;
            else
                % if neither row or column dimension is equal to one the input
                % is not a vector and an error is thrown
                error("Input is not a vector");
            end
        end

        % takes in a vector. if the input is already a row vector, the input is 
        % returned unchanged. If the input is a column vector, the vector is 
        % returned as a row vector.
        function v_out = CheckRowVec(v_in)
            [rows, columns] = size(v_in);
            if columns == 1
                v_out = v_in.';
            elseif rows == 1
                v_out = v_in;
            else
                % if neither row or column dimension is equal to one the input
                % is not a vector and an error is thrown
                error("Input is not a vector");
            end
        end

        % check that list of measurements entered is nx3 not 3xn. if it is 3xn, 
        % the function returns the measurents transposed
        function data_out = CheckDataDim(data_in)
            [rows, columns] = size(data_in);
            if columns > rows
                data_out = data_in';
            else
                data_out = data_in;
            end
        end

        % perform quaternion normalization in order to preserve orthogonality
        function q_out = NormQuat(q_in)
            q_out = q_in./norm(q_in);
        end

        % normalizes a vector, useful for accel and mag measurements
        function v_out = NormVec(v_in)
            v_out = v_in./norm(v_in);
        end
    end
end