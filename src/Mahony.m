classdef Mahony < Ahrs
    properties (GetAccess = public, SetAccess = protected)
        kp (1,1) double = 10;
        ki (1,1) double = 1.5;
    end
    methods
        function varargout = Filter(obj, varargin)
            accel = varargin{1};
            gyro = varargin{2};
            mag = varargin{3};
            [a, g, m] = obj.PreprocessMeasurements(accel, gyro, mag);
            
            % number of input arguments minus obj
            nargs = nargin - 1;
            
            if nargs < 3 || nargs == 4
                msg = "Not enough input arguments";
                error(msg);
            elseif nargs == 3
                % number of samples
                n = length(a);
            
                rpy = zeros(n,3);
                
                q = obj.InitQuat(a(1,:), m(1,:));
                
                error_int = zeros(1,3);
                
                for i = 1:1:n
                    [rpy(i,:), q, error_int] = ...
                        obj.Main(a(i,:), g(i,:), m(i,:), q, error_int);
                end
                
                varargout{1} = rpy;
            elseif nargs == 5
                q = varargin{4};
                error_int = varargin{5};
                [rpy, q, error_int] = obj.Main(a, g, m, q, error_int);
                varargout{1} = rpy;
                varargout{2} = q;
                varargout{3} = error_int;
            else
                msg = "Too many input arguments";
                error(msg);
            end
        end
        
        function ui = StartUI(obj)
            if isempty(obj.imu)
                error('No mpu9250 object has been set up');
            end
            
            ui = UI();
            tf = 1000;
            t = 0;
            
            % initial quaternion
            [a, g, m] = read(obj.imu);
            [a, ~, m] = obj.PreprocessMeasurements(a, g, m);
            q = obj.InitQuat(a, m);
            
            % initial error covariance matrix
            error_int = zeros(1,3);
            
            while t < tf
                % read imu
                [a, g, m] = read(obj.imu);
                % filter readings
                [rpy, q, error_int]  = obj.Filter(a, g, m, q, error_int);
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
            rpy_mahony = varargin{1};
            n = length(rpy_mahony);
            tf = n*obj.sample_time;
            
            time = linspace(0,tf,n);

            % set figure position
            fig = figure();
            fig.Position = [500, 200, 800, 500];
            ax = axes(fig);

            % lables
            title('mahony estimated orientation');
            xlabel('time (seconds)');
            ylabel('orientation (degrees)');
            hold on;
            
            if nargin == 1
                error("Not enough input arguments");
            elseif nargin == 2
                PlotmahonyData(ax, time, rpy_mahony);
            elseif nargin == 3
                rpy_mahony = varargin{1};
                rpy_true = varargin{2};
                PlotTrueData(ax, time, rpy_true);
                PlotmahonyData(ax, time, rpy_mahony);
            else
                error("Too many input arguments");
            end

            leg = legend(ax);
            leg.String = {'roll_{true}', 'pitch_{true}', 'yaw_{true}', ...
                'roll_{mahony}', 'pitch_{mahony}', 'yaw_{mahony}'};
            leg.NumColumns = 6;
            leg.Location = 'southoutside';

            hold off;
            
            function PlotmahonyData(ax, time, rpy_mahony)
                % line colors
                blue = [0, 0.4470, 0.7410];
                orange = [0.8500, 0.3250, 0.0980];
                yellow = [0.9290, 0.6940, 0.1250];
                
                l = plot(ax, time, rpy_mahony);
                l(1).LineWidth = 2;
                l(2).LineWidth = 2;
                l(3).LineWidth = 2;
                l(1).Color = blue;
                l(2).Color = orange;
                l(3).Color = yellow;
            end
            function PlotTrueData(ax, time, rpy_true)
                l = plot(ax, time, rpy_true);
                l(1).LineWidth = 2;
                l(2).LineWidth = 2;
                l(3).LineWidth = 2;
                l(1).LineStyle = ':';
                l(2).LineStyle = ':';
                l(3).LineStyle = ':';
                l(1).Color = 'k';
                l(2).Color = 'k';
                l(3).Color = 'k';
            end
        end
    end
    
    methods (Access = private)
        function [rpy, q_new, error_int_new] = Main(obj, accel, gyro, mag, q, error_int)
            a = obj.NormVec(accel);
            g = gyro;
            m = obj.NormVec(mag);
            
            [bx, bz] = obj.MagRef(m, q);
            
            error_a = obj.GravError(a, q);
            
            error_m = obj.MagError(m, bx, bz, q);
            
            % compute orientation error
            error = error_a + error_m;

            % integrate orientation error
            error_int_new = error_int + error*obj.sample_time;
            
            % correct gyroscope readings
            g = g + obj.kp.*error + obj.ki*error_int_new;
            
            % compute quaternion derivative
            qg = [0, obj.CheckRowVec(g)];
            qdot = (1/2)*obj.QuatProd(q, qg);
            
            % integrate to compute new quaternion
            q = obj.CheckRowVec(q) + obj.sample_time*obj.CheckRowVec(qdot);
            q_new = obj.NormQuat(q);
            
            rpy = obj.QuatToTaitBryan(q);
        end
        
        function [bx, bz] = MagRef(obj, mag, q)
            % pre-processing
            q = obj.CheckRowVec(q);
            m = obj.CheckRowVec(mag);
            
            % augment magnetometer reading to quaternion form
            qm = [0, m];
            qconj = obj.QuatConj(q); 
            hprime = obj.QuatProd(qm, qconj);
            h = obj.QuatProd(q, hprime);
            bx = norm([h(1,2), h(1,3)]);
            bz = h(1,4);
        end
        
        % error in gravity estimate
        function error_a = GravError(obj, accel, q)
            a = obj.CheckRowVec(accel);
            
            qw = q(1);
            qx = q(2);
            qy = q(3);
            qz = q(4);
            
            v = [2*(qx*qz - qw*qy); 
                2*(qw*qx + qy*qz); 
                qw^2 - qx^2 - qy^2 + qz^2].';
            
            error_a = cross(a,v);
        end
        
        % error in magnetic field estimate
        function error_m = MagError(obj, mag, bx, bz, q)
            m = obj.CheckRowVec(mag);
            
            qw = q(1);
            qx = q(2);
            qy = q(3);
            qz = q(4);
            
            w = [2*bx*(0.5 - qy^2 - qz^2) + 2*bz*(qx*qz - qw*qy);
                2*bx*(qx*qy - qw*qz) + 2*bz*(qw*qx + qy*qz);
                2*bx*(qw*qy + qx*qz) + 2*bz*(0.5 - qx^2 - qy^2)];
            
            error_m = cross(m, w);
        end
    end
end