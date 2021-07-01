classdef (Abstract) Ahrs < handle
    
    % Attitude and Heading Reference System (AHRS)
    % Martin L Pryde MSc
    %

    properties (GetAccess = public, SetAccess = protected)
        imu; % matlab <mpu9250()> object
        sample_time (1,1) double = 0.01; % system fundamental sample time
        hard_iron_bias (1,3) double = [-3.8236, 140.6495, -170.3542];
        soft_iron_bias (3,3) double = [1.0034, 0.0112, 0.0142; 0.0112, 1.0593, 0.0186; 0.0142, 0.0186, 0.9414];
    end
    
    methods
        % set methods for object properties 
        function SetIMU(obj, imu)
            if imu.SamplesPerRead ~= 1
                error("Invalid mpu9250(): <SamplesPerRead> property of input mpu9250() object must be set to 1.");
            elseif imu.OutputFormat ~= "matrix"
                error("Invalid mpu9250(): <OutputFormat> property of input mpu9250() object must be set to <'matrix'>");
            end
            obj.imu = imu;
            obj.sample_time = 1/obj.imu.SampleRate;
        end
        function SetSampleTime(obj, sample_time)
            if isempty(obj.imu)
                obj.sample_time = sample_time;
            else
                error("Cannot set sample time when Ahrs.imu has been set.")
            end
        end
        function SetIronBias(obj, varargin)
            % SetIronBias(b) sets the Ahrs hard_iron_bias property to <b>
            % SetIronBias(b,A) sets the Ahrs hard_iron_bias and
            % soft_iron_bias properties to <b> and <A> respectively
            n = nargin - 1; % number of input arguments minus the obj input
            if n == 0
                error('Not enough input arguments');
            elseif n == 1
                obj.hard_iron_bias = varargin{1};
                clc;
                fprintf('New Hard Iron Bias:\n\n');
                disp(varargin{1});
            elseif n == 2
                obj.hard_iron_bias = varargin{1};
                obj.soft_iron_bias = varargin{2};
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
    
    methods (Access = public)
        % method for calibrating magnetometer
        function MagCal(obj)
            % check that the user has set an <mpu9250()> object in the
            % class
            if isempty(obj.imu)
                error('Set an imu object in order to begin magnetometer calibration');
            end
            
            close all; % close all other open figures
            ts = obj.sample_time; % extract sample time from imu object
            tf = 60; % duration of magnetometer calibration experiment
            n = tf/ts; % number of samples to be taken during experiment
            
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
            fprintf('Calibration complete!\n\n');
            fprintf('New Hard Iron Bias:\n\n');
            disp(b);
            fprintf('New Soft Iron Bias:\n\n');
            disp(A);
        end
    end
    
    methods (Abstract)
        % methods to be uniquely defined for each ahrs algorithm
        Run(obj);
        RunUI(obj);
        Plot(obj);
    end
    
    methods (Abstract, Access = protected)
        % the filter method of each supeclass implements its core ahrs
        % algorithm
        Filter(obj);
    end
    
    methods (Access = protected)
        % methods common to all algorithms but not to be displayed to be
        % made available to the user
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
        function q = InitializeQuaternion(obj, accel, mag)
            % normalize readings
            a = accel./norm(accel);
            m = mag./norm(mag);

            a = obj.CheckRowVec(a);
            m = obj.CheckRowVec(m);

            % sort accel readings
            ax = a(1,1);
            ay = a(1,2);
            az = a(1,3);

            % get roll & pitch
            roll = atan2(ay, az);
            pitch = atan2(-ax, ay*sin(roll) + az*cos(roll));

            % rotate measured mag field into earth frame
            b = (yrot(-pitch)*xrot(-roll)*(m.')).';

            % sort mag readings
            bx = b(1,1);
            by = b(1,2);

            % get yaw
            yaw = atan2(-by, bx);

            % convert to earth->body quaternion
            q = angle2quat(yaw, pitch, roll);

            % normalize
            q = q./norm(q);

            % rotation matrices
            function R = xrot(a)
                C = cos(a);
                S = sin(a);
                R = [1, 0, 0;
                    0, C, S;
                    0, -S, C];
            end
            function R = yrot(a)
                C = cos(a);
                S = sin(a);
                R = [C, 0, -S;
                    0, 1, 0;
                    S, 0, C];
            end
        end
        function dcm = QuaternionToDCM(obj, q)
            % converts earth->body quaternion to earth->body dcm
            q = obj.CheckRowVec(q);

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
        function [roll, pitch, yaw] = QuaternionToTaitBryan(obj, q)
            % take earth->body quaternion and convert to tait-bryan euler angles in
            % degrees

            % normalize input quaternion
            q = q./norm(q);
            q = obj.CheckRowVec(q);
            
            % convert to tait-bryan angles
            [yaw, pitch, roll] = quat2angle(q, 'ZYX');
            
            % convert to degrees
            roll = rad2deg(roll);
            pitch = rad2deg(pitch);
            yaw = rad2deg(yaw);
        end
    end
    
    methods (Static, Access = protected)
        % methods useful to all algorithms but which do not call or modify
        % the class itself
        function s = Skew(v)
            % turns a vector of length 3 into a skew-symmetric matrix
            p = v(1);
            q = v(2);
            r = v(3);
            s = [0, -r, q; r, 0, -p; -q, p, 0];
        end
        function v_out = CheckColVec(v_in)
            % takes in a vector. if the input is already a column vector,
            % the input is returned unchanged. If the input is a row
            % vector, the vector is returned as a column vector.
            [rows, columns] = size(v_in);
            if columns > rows
                v_out = v_in.';
            else
                v_out = v_in;
            end
        end
        function v_out = CheckRowVec(v_in)
            % takes in a vector. if the input is already a row vector,
            % the input is returned unchanged. If the input is a column
            % vector, the vector is returned as a row vector.
            [rows, columns] = size(v_in);
            if rows > columns
                v_out = v_in.';
            else
                v_out = v_in;
            end
        end
        function data_out = CheckDataDim(data_in)
            % check that list of measurements entered is nx3 not 3xn. if it
            % is 3xn, the function returns the measurents transposed
            [rows, columns] = size(data_in);
            if columns > rows
                data_out = data_in';
            else
                data_out = data_in;
            end
        end
    end
end