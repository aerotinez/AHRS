classdef EKF < Ahrs
    
    % Extended Kalman Filter for MPU-9250 AHRS
    % Martin L Pryde MSc
    
    properties (GetAccess = public, SetAccess = protected)
        % sensor noise standard deviations
        sigma_a (1,1) double = 1e-03;
        sigma_g (1,1) double = 9e-04;
        sigma_m (1,1) double = 5e-03;
        
        % environmental constants
        % in NED frame (m/s^2)
        local_earth_gravity (1,3) double = [0, 0, 9.8665];
        % in NED frame (microTesla)
        local_earth_magnetic_field (1,3) double = [23.1715, 0.1976, 40.6293]; 
    end
    
    methods
        function rpy = Run(obj, accel, gyro, mag)
            a = accel;
            g = gyro;
            m = mag;
            
            % number of samples
            n = length(a);
            
            % array to hold filtered results
            rpy = zeros(n,3);
            
            % begin extended kalman filtering
            for i = 1:1:n
                rpy(i,:) = obj.Filter(a(i,:), g(i,:), m(i,:));
            end
        end
        
        function ui = RunUI(obj)
            if isempty(obj.imu)
                error('No mpu9250 object has been set up');
            end
            
            ui = UI();
            tf = 1000;
            t = 0;
            while t < tf
                % read imu
                [a, g, m] = read(obj.imu);
                % filter readings
                rpy = obj.Filter(a, g, m);
                % extract roll, pitch & yaw
                roll = deg2rad(rpy(1,1));
                pitch = deg2rad(rpy(1,2));
                yaw = deg2rad(rpy(1,3));
                % create dcm (body->earth)
                dcm = angle2dcm(yaw, pitch, roll, 'ZYX').';
                % update viewer
                ui.RotateBox(dcm);
                % increment program time
                t = t + obj.sample_time;
                pause(obj.sample_time);
            end
        end
        
        function fig = Plot(obj, varargin)
            rpy_ekf = varargin{1};
            n = length(rpy_ekf);
            tf = n*obj.sample_time;
            
            time = linspace(0,tf,n);

            % set figure position
            fig = figure();
            fig.Position = [500, 200, 800, 500];
            ax = axes(fig);

            % lables
            title('ekf estimated orientation');
            xlabel('time (seconds)');
            ylabel('orientation (degrees)');
            hold on;
            
            if nargin == 1
                error("Not enough input arguments");
            elseif nargin == 2
                PlotEKFData(ax, time, rpy_ekf);
            elseif nargin == 3
                rpy_ekf = varargin{1};
                rpy_true = varargin{2};
                PlotTrueData(ax, time, rpy_true);
                PlotEKFData(ax, time, rpy_ekf);
            else
                error("Too many input arguments");
            end

            leg = legend(ax);
            leg.String = {'roll_{true}', 'pitch_{true}', 'yaw_{true}', ...
                'roll_{ekf}', 'pitch_{ekf}', 'yaw_{ekf}'};
            leg.NumColumns = 6;
            leg.Location = 'southoutside';

            hold off;
            
            function PlotEKFData(ax, time, rpy_ekf)
                % line colors
                blue = [0, 0.4470, 0.7410];
                orange = [0.8500, 0.3250, 0.0980];
                yellow = [0.9290, 0.6940, 0.1250];
                
                l_ekf = plot(ax, time, rpy_ekf);
                l_ekf(1).LineWidth = 2;
                l_ekf(2).LineWidth = 2;
                l_ekf(3).LineWidth = 2;
                l_ekf(1).Color = blue;
                l_ekf(2).Color = orange;
                l_ekf(3).Color = yellow;
            end
            function PlotTrueData(ax, time, rpy_true)
                l_true = plot(ax, time, rpy_true);
                l_true(1).LineWidth = 2;
                l_true(2).LineWidth = 2;
                l_true(3).LineWidth = 2;
                l_true(1).LineStyle = ':';
                l_true(2).LineStyle = ':';
                l_true(3).LineStyle = ':';
                l_true(1).Color = 'k';
                l_true(2).Color = 'k';
                l_true(3).Color = 'k';
            end
        end
    end
    
    methods (Access = protected)
        function rpy = Filter(obj, accel, gyro, mag)
            % pre-process dataset to align IMU and magnetometer axes
            [a, g, m] = obj.PreprocessMeasurements(accel, gyro, mag);
            
            % state vector
            persistent X;
            if isempty(X)
                % initialize orientation
                q = obj.InitializeQuaternion(a, m);
                X = obj.CheckColVec(q);
            end
            
            % error covariance matrix
            persistent P;
            if isempty(P)
                P = eye(4,4);
            end
            
            % extended kalman filtering
            
            % a priori state
            Xa = obj.ProcessModel(g, X);

            % a priori error covariance matrix
            F = obj.ProcessJacobian(g);
            Q = obj.ProcessCovariance(Xa);
            Pa = F*P*(F.') + Q;

            % correction
            z = [a, m].';
            h = obj.MeasurementModel(Xa);
            H = obj.MeasurementJacobian(Xa);

            % innovation
            R = obj.MeasurementCovariance(X, a, m);
            S = H*Pa*(H.') + R;

            % kalman gain
            K = (Pa*(H.'))/S;

            % a posteriori state
            X = Xa + K*(z - h);

            % a posteriori error covariance matrix
            P = Pa - K*H*Pa;

            % normalize state (quaternion)
            X = X./norm(X);

            % extract euler angles
            [roll, pitch, yaw] = obj.QuaternionToTaitBryan(X);
            rpy = [roll, pitch, yaw];
        end
    end
    
    methods (Access = private)
        function W = Omega(obj, w)
            w = obj.CheckColVec(w);
            W = [obj.Skew(w), w; -w.', 0];
        end
        
        function f = ProcessModel(obj, w, q)
            q = q./norm(q);
            q = obj.CheckColVec(q);
            f = (eye(4,4) + (0.5*obj.sample_time).*obj.Omega(w))*q;
        end
        
        function F = ProcessJacobian(obj, w)
            F = (eye(4,4) + (0.5*obj.sample_time).*obj.Omega(w));
        end
        
        function Q = ProcessCovariance(obj, q)
            qw = q(1);
            qx = q(2);
            qy = q(3);
            qz = q(4);

            W = [
                qz, qy, -qx;
                -qy, qz, qw;
                qx, -qw, qz;
                -qw, -qx, -qy
                ];
            W = (obj.sample_time/2).*W;

            Sigma_g = obj.sigma_g^2.*eye(3,3);

            Q = W*Sigma_g*(W.');
        end
        
        function h = MeasurementModel(obj, q)
            g = obj.CheckColVec(obj.local_earth_gravity);
            r = obj.CheckColVec(obj.local_earth_magnetic_field);
            C = obj.QuaternionToDCM(q); % earth->body

            h = [C*g; C*r];
        end
        
        function H = MeasurementJacobian(obj, q)
            g = obj.local_earth_gravity;
            r = obj.local_earth_magnetic_field;
            
            qw = q(1);
            qx = q(2);
            qy = q(3);
            qz = q(4);
            
            gx = g(1);
            gy = g(2);
            gz = g(3);
            
            rx = r(1);
            ry = r(2);
            rz = r(3);
            
            % 1st row
            h11 = gx*qw + gy*qz - gz*qy;
            h12 = gx*qx + gy*qy + gz*qz;
            h13 = gy*qx - gx*qy - gz*qw;
            h14 = gy*qw - gx*qz + gz*qx;
            % 2nd row
            h21 = gy*qw - gx*qz + gz*qx; 
            h22 = gx*qy - gy*qx + gz*qw; 
            h23 = gx*qx + gy*qy + gz*qz; 
            h24 = gz*qy - gy*qz - gx*qw;
            % 3rd row
            h31 = gx*qy - gy*qx + gz*qw; 
            h32 = gx*qz - gy*qw - gz*qx; 
            h33 = gx*qw + gy*qz - gz*qy; 
            h34 = gx*qx + gy*qy + gz*qz;
            % 4th row
            h41 = qw*rx - qy*rz + qz*ry;
            h42 = qx*rx + qy*ry + qz*rz;
            h43 = qx*ry - qw*rz - qy*rx;
            h44 = qw*ry + qx*rz - qz*rx;
            % 5th row
            h51 = qw*ry + qx*rz - qz*rx;
            h52 = qw*rz - qx*ry + qy*rx;
            h53 = qx*rx + qy*ry + qz*rz;
            h54 = qy*rz - qw*rx - qz*ry;
            % 6th row
            h61 = qw*rz - qx*ry + qy*rx;
            h62 = qz*rx - qx*rz - qw*ry;
            h63 = qw*rx - qy*rz + qz*ry;
            h64 = qx*rx + qy*ry + qz*rz;
            
            H = 2*[h11, h12, h13, h14;
                h21, h22, h23, h24;
                h31, h32, h33, h34;
                h41, h42, h43, h44;
                h51, h52, h53, h54;
                h61, h62, h63, h64];
        end
        
        function R = MeasurementCovariance(obj, q, accel, mag)
            % normalize inputes
            a = NormVec(accel);
            m = NormVec(mag);
            g = NormVec(obj.local_earth_gravity);
            r = NormVec(obj.local_earth_magnetic_field);

            % acceleration gating test
            %
            % if the noramlized magnitude of the measurement from the accelerometer
            % deviates significantly from standard earth gravity then set sigma_a
            % to very high so the ekf relies on the gyro prediction and
            % magnetometer measurement

            accel_test = abs(norm(a) - norm(g));
            if accel_test > 0.05
                Sigma_a = 1e06;
            else
                Sigma_a = obj.sigma_a;
            end

            % magnetic field gating test
            %
            % if the normalized magntiude of the measurement from the magnetometer
            % AND the estimated magnetic inclination (dip) angle deviate
            % significantly from nominal then set sigma_m to very high so the ekf
            % relies on the gyro prediction and accelerometer measurement
            theta = rad2deg(acos(r(1))); % true local magnetic dip

            a = obj.CheckColVec(a);

            m = obj.CheckColVec(m);

            C = obj.QuaternionToDCM(q).'; % body->earth

            m_est = C*m;
            a_est = C*a;
            num = obj.CheckRowVec(m_est)*obj.CheckColVec(a_est);
            den = norm(m)*norm(a);
            theta_est = acos(num/den);
            theta_est = rad2deg(theta_est); % estimated local magnetic dip

            mag_test_1 = abs(norm(m) - norm(r));
            mag_test_2 = abs(theta_est - theta);

            if mag_test_1 > 0.1 && mag_test_2 > 2
                Sigma_m = 1e06;
            else
                Sigma_m = obj.sigma_m;
            end

            % construct measurement covariance matrix
            Ra = (Sigma_a^2).*eye(3,3);
            Rm = (Sigma_m^2).*eye(3,3);
            R = [Ra, zeros(3,3); zeros(3,3), Rm];

            function v_out = NormVec(v_in)
                v_out = v_in./norm(v_in);
            end
        end
    end
end