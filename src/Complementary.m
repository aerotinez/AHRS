classdef Complementary < Ahrs

    % Complementary Filter for MPU-9250 AHRS
    % Martin L Pryde MSc

    properties (GetAccess = public, SetAccess = protected)
        % complementary filter gain
        k (1,1) double = 0.02;
    end

    methods (Access = public)
        function varargout = Filter(obj, varargin)
            % number of input arguments minus object
            nargs = nargin - 1;
            
            accel = varargin{1};
            gyro = varargin{2};
            mag = varargin{3};
            
            [accel, gyro, mag] = obj.PreprocessMeasurements(accel, gyro, mag);

            if nargs < 3
                error("Not enough input arguments");
            elseif nargs == 3
                % number of samples
                n = length(accel);
                
                % array to hold filtered results
                rpy = zeros(n,3);
                
                % initial quaternion
                q = obj.InitQuat(accel(1,:), mag(1,:));

                for i = 1:1:n
                    [rpy(i,:), q] = obj.Main(accel(i,:), gyro(i,:), mag(i,:), q);
                end

                varargout{1} = rpy;
            elseif nargs == 4
                q_init = varargin{4};
                
                [rpy, q] = obj.Main(accel, gyro, mag, q_init);
                
                varargout{1} = rpy;
                varargout{2} = q;
            else
                error("Too many input arguments");
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

            while t < tf
                % read imu
                [a, g, m] = read(obj.imu);
                % filter readings
                [rpy, q]  = obj.Filter(a, g, m, q);
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
            rpy_comp = varargin{1};
            n = length(rpy_comp);
            tf = n*obj.sample_time;
            
            time = linspace(0,tf,n);

            % set figure position
            fig = figure();
            fig.Position = [500, 200, 800, 500];
            ax = axes(fig);

            % lables
            title('complementary estimated orientation');
            xlabel('time (seconds)');
            ylabel('orientation (degrees)');
            hold on;
            
            if nargin == 1
                error("Not enough input arguments");
            elseif nargin == 2
                PlotCompData(ax, time, rpy_comp);
            elseif nargin == 3
                rpy_comp = varargin{1};
                rpy_true = varargin{2};
                PlotTrueData(ax, time, rpy_true);
                PlotComplementaryData(ax, time, rpy_comp)
            else
                error("Too many input arguments");
            end

            leg = legend(ax);
            leg.String = {'roll_{true}', 'pitch_{true}', 'yaw_{true}', ...
                'roll_{ekf}', 'pitch_{ekf}', 'yaw_{ekf}'};
            leg.NumColumns = 6;
            leg.Location = 'southoutside';

            hold off;
            
            function PlotComplementaryData(ax, time, rpy_comp)
                % line colors
                blue = [0, 0.4470, 0.7410];
                orange = [0.8500, 0.3250, 0.0980];
                yellow = [0.9290, 0.6940, 0.1250];
                
                l = plot(ax, time, rpy_comp);
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
        function SetFilterGain(obj, gain)
            if gain > 1
                error("complementary filter gain must be a scalar in the range 0 - 1");
            end
            obj.k = gain;
        end
    end
    methods (Access = private)
        function [rpy, q] = Main(obj, accel, gyro, mag, q_init)
            a = accel;
            g = gyro;
            m = mag;
            
            q = obj.CheckColVec(q_init);
            
            % get quaternion from gyro (dead reckoning)
            W = [obj.Skew(g), g.'; - g, 0];
            qdot = 0.5*W*q;
            qg = q + qdot*obj.sample_time;
            qg = obj.NormQuat(qg);
            qg = obj.CheckRowVec(qg);

            % get quaternion from accel & mag
            qam = obj.InitQuat(a, m);
            qam = obj.CheckRowVec(qam);

            % perform complementary filtering
            q = (1 - obj.k).*qg + obj.k.*qam;
            q = obj.NormQuat(q);

            % convert to tait-bryan angles
            rpy = obj.QuatToTaitBryan(q);
        end
    end
end