classdef UI < matlab.apps.AppBase
% MPU-9250 Estimated Orientation Monitor
% Martin L Pryde MSc
% martinpryde@ymail.com
% 22nd of April 2021
%
% This simple UIFigure app animates the estimated orientation of an
% MPU-9250 connected through I2C to MATLAB. The user can monitor the
% estimation in a 3-dimensional animation alongside the real-time estimates
% of the MPU-9250 roll, pitch and yaw angles in degrees.
%
% The output of you AHRS algorithm should be a 3x3 double DCM matrix.
% Simply input your generated matrix as an argument into app.RotateBox() to
% animate your estimation.

    % App components
    properties (Access = private)
        UIFigure    matlab.ui.Figure;
        GridLayout  matlab.ui.container.GridLayout;
        AxesPanel   matlab.ui.container.Panel;
        UIAxes      matlab.ui.control.UIAxes;
        RollPanel   matlab.ui.container.Panel;
        RollLabel   matlab.ui.control.Label;
        PitchPanel  matlab.ui.container.Panel;
        PitchLabel  matlab.ui.control.Label;
        YawPanel    matlab.ui.container.Panel;
        YawLabel    matlab.ui.control.Label;
        Box;        % struct containing all the animation objects depicting the MPU-9250
        EarthBox;   % struct containing the starting position data of the MPU-9250 alligned with earth frame
    end
    
    % Constants
    properties (Constant, Access = private)
        % MPU-9250 animation graphic dimensions (mm)
        BREADTH = 25;
        DEPTH = 15;
        HEIGHT = 3;
        MINI_AXIS_LENGTH = 30;
        
        % animation colours (normalised RGB)
        BLUE = [0, 0.4470, 0.7410];
        ORANGE = [0.8500, 0.3250, 0.0980];
        YELLOW = [0.9290, 0.6940, 0.1250];
        PURPLE = [0.4940, 0.1840, 0.5560];
    end
    
    % App variables
    properties (Access = public)
        DCM = double(eye(3,3)); % direction cosine matrix initialised to identity
        % euler angles are calculated from app.DCM
        roll    (1,1) double;
        pitch   (1,1) double;
        yaw     (1,1) double;
    end
    
    % Component initialization
    methods (Access = private)
        % Create UIFigure and components Zon calling this function
        function CreateComponents(app)
            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [50, 200, 500, 500];
            app.UIFigure.Name = 'MPU-9250 Monitor';

            % Create GridLayout. 3 columns, 6 rows to get the correct ratio
            % of UIAxes to UILabel size for the roll, pitch and yaw panels.
            % Use 1x for all grid squares in order to accomodate UIFigure
            % resizing by the user.
            app.GridLayout = uigridlayout(app.UIFigure);
            app.GridLayout.ColumnWidth = {'1x', '1x', '1x'};
            app.GridLayout.RowHeight = {'1x', '1x', '1x', '1x', '1x', '1x'};

            % Create panel to contain UIAxes
            app.AxesPanel = uipanel(app.GridLayout);
            app.AxesPanel.Title = 'estimated orientation 3d';
            app.AxesPanel.Layout.Row = [1, 5];
            app.AxesPanel.Layout.Column = [1, 3];

            % Create UIAxes, square aspect ratio
            app.UIAxes = uiaxes(app.AxesPanel);
            title(app.UIAxes, 'MPU-9250 Estimated Orientation')
            xlabel(app.UIAxes, 'X_{e}')
            ylabel(app.UIAxes, 'Y_{e}')
            zlabel(app.UIAxes, 'Z_{e}')
            % app.UIAxes.View = [155 22.5];
            app.UIAxes.PlotBoxAspectRatio = [1, 1, 1];
            app.UIAxes.GridLineStyle = '-';
            app.UIAxes.XGrid = 'on';
            app.UIAxes.YGrid = 'on';
            app.UIAxes.ZGrid = 'on';
            app.UIAxes.Position = [50, 10, 360, 360];
            n = 1.5;
            app.UIAxes.XLim = [-n*app.BREADTH, n*app.BREADTH];
            app.UIAxes.YLim = [-n*app.BREADTH, n*app.BREADTH];
            app.UIAxes.ZLim = [-n*app.BREADTH, n*app.BREADTH];

            % Create panel to contain roll angle in degrees
            app.RollPanel = uipanel(app.GridLayout);
            app.RollPanel.Title = 'estimated roll';
            app.RollPanel.Layout.Row = 6;
            app.RollPanel.Layout.Column = 1;

            % Create label displaying the roll angle. Set background to
            % white, limit post-decimal to 2
            app.RollLabel = uilabel(app.RollPanel);
            app.RollLabel.Position = [0, 0, 152, 52];
            app.RollLabel.Text = '00.00';
            app.RollLabel.BackgroundColor = 'white';
            app.RollLabel.HorizontalAlignment = 'center';
            app.RollLabel.VerticalAlignment = 'center';
            app.RollLabel.FontSize = 26;
            app.RollLabel.FontWeight = 'bold';

            % Create panel for pitch
            app.PitchPanel = uipanel(app.GridLayout);
            app.PitchPanel.Title = 'estimated pitch';
            app.PitchPanel.Layout.Row = 6;
            app.PitchPanel.Layout.Column = 2;

            % Create label displaying pitch angle
            app.PitchLabel = uilabel(app.PitchPanel);
            app.PitchLabel.Position = [0, 0, 152, 52];
            app.PitchLabel.Text = '00.00';
            app.PitchLabel.BackgroundColor = 'white';
            app.PitchLabel.HorizontalAlignment = 'center';
            app.PitchLabel.VerticalAlignment = 'center';
            app.PitchLabel.FontSize = 26;
            app.PitchLabel.FontWeight = 'bold';

            % Create panel for yaw
            app.YawPanel = uipanel(app.GridLayout);
            app.YawPanel.Title = 'estimated yaw';
            app.YawPanel.Layout.Row = 6;
            app.YawPanel.Layout.Column = 3;

            % Create label displaying yaw angle
            app.YawLabel = uilabel(app.YawPanel);
            app.YawLabel.Position = [0, 0, 152, 52];
            app.YawLabel.Text = '00.00';
            app.YawLabel.BackgroundColor = 'white';
            app.YawLabel.HorizontalAlignment = 'center';
            app.YawLabel.VerticalAlignment = 'center';
            app.YawLabel.FontSize = 26;
            app.YawLabel.FontWeight = 'bold';

            % only show figure after all components are created
            app.UIFigure.Visible = 'on';
        end
        % Create 3d animation
        function Box = CreateBox(app)
            % The graphic representing the MPU-9250 is made from a surf()
            % object depicting 4 sides of a cuboid. The remaining two sides
            % are filled using a patch() object. Finally, a Y-X-Down
            % mini-axes is attached to the graphic.
            
            % Create 4 sides of the cube
            theta = -pi:pi/2:pi;
            phi = pi/4;
            C = cos(theta + phi);
            S = sin(theta + phi);
            x = 0.5*app.BREADTH*[C; C]/cos(phi);
            y = 0.5*app.DEPTH*[S; S]/sin(phi);
            z = 0.5*app.HEIGHT*[-ones(size(theta)); ones(size(theta))];
            Box.surf = surf(app.UIAxes, x, y, z);
            Box.surf.FaceColor = app.BLUE;
            hold(app.UIAxes, 'on');
            Box.surf.Annotation.LegendInformation.IconDisplayStyle = 'off';
            
            % Create other 2 sides
            Box.patch = patch(app.UIAxes, x.', y.', z.', app.BLUE);
            Box.patch.Annotation.LegendInformation.IconDisplayStyle = 'off';
            
            % Create mini NED axes
            or = [0, 0]; % origin
            Box.XAxis = line(app.UIAxes, [0, app.MINI_AXIS_LENGTH], or, or);
            Box.XAxis.LineWidth = 3;
            Box.XAxis.Color = app.ORANGE;
            Box.YAxis = line(app.UIAxes, or, [0, app.MINI_AXIS_LENGTH], or);
            Box.YAxis.LineWidth = 3;
            Box.YAxis.Color = app.YELLOW;
            Box.ZAxis = line(app.UIAxes, or, or, [0, app.MINI_AXIS_LENGTH]);
            Box.ZAxis.LineWidth = 3;
            Box.ZAxis.Color = app.PURPLE;
            
            % Zdata view
            app.UIAxes.View = [10, 35];
            
%             % Create legend
%             lgd = legend(app.UIAxes, 'X_{b}', 'Y_{b}', 'Z_{b}');
%             lgd.NumColumns = 3;
%             lgd.Location = 'south';
%             lgd.Position = [100, 10, 100, 10];
        end
        % Create box data in earth frame only
        function Box = CreateEarthBox(app)
            % The sole purpose of this object is to act as a container of
            % the data for when the MPU-9250 is in geometric alignment to
            % the earth frame. This position data is then multiplied with
            % the user's DCM to transform the graphic into the frame
            % computed by the user's AHRS algorithm.
            
            % Create mini NED axes
            or = [0, 0]; % origin
            Box.XAxis.XData = [0, app.MINI_AXIS_LENGTH];
            Box.XAxis.YData = or;
            Box.XAxis.ZData = or;
            Box.YAxis.XData = or;
            Box.YAxis.YData = [0, app.MINI_AXIS_LENGTH];
            Box.YAxis.ZData = or;
            Box.ZAxis.XData = or;
            Box.ZAxis.YData = or;
            Box.ZAxis.ZData = [0, app.MINI_AXIS_LENGTH];
            
            % Create 4 sides of the cube
            theta = -pi:pi/2:pi;
            phi = pi/4;
            C = cos(theta + phi);
            S = sin(theta + phi);
            x = 0.5*app.BREADTH*[C; C]/cos(phi);
            y = 0.5*app.DEPTH*[S; S]/sin(phi);
            z = 0.5*app.HEIGHT*[-ones(size(theta)); ones(size(theta))];
            Box.surf.XData = x;
            Box.surf.YData = y;
            Box.surf.ZData = z;
            
            % Create other 2 sides
            Box.patch.XData = x.';
            Box.patch.YData = y.';
            Box.patch.ZData = z.';
        end
    end
    
    % App creation and deletion
    methods (Access = public)
        % Construct app
        function app = UI
            % Create UIFigure and components
            CreateComponents(app);
            
            % Create moving box
            app.Box = CreateBox(app);
            
            % Assign coordinate data of the box in the earth frame
            app.EarthBox = CreateEarthBox(app);

            % Register the app with App Designer
            registerApp(app, app.UIFigure);

            if nargout == 0
                clear app
            end
        end
        % Code that executes before app deletion
        function delete(app)
            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
    
    % App methods
    methods (Access = public)
        % Redraw box in new position
        function RotateBox(app, DirectionCosineMatrix)
            % Components to be manipulated
            app.DCM = DirectionCosineMatrix;
            R = app.DCM;
            s = app.EarthBox.surf;
            p = app.EarthBox.patch;
            x = app.EarthBox.XAxis;
            y = app.EarthBox.YAxis;
            z = app.EarthBox.ZAxis;
            sb = app.Box.surf;
            pb = app.Box.patch;
            xb = app.Box.XAxis;
            yb = app.Box.YAxis;
            zb = app.Box.ZAxis;
            
            % Rotate 4 sides of cube
            b = R*[s.XData(1,:); s.YData(1,:); s.ZData(1,:)]; % surf bottom
            t = R*[s.XData(2,:); s.YData(2,:); s.ZData(2,:)]; % surf top
            sb.XData = [b(1,:); t(1,:)];
            sb.YData = [b(2,:); t(2,:)];
            sb.ZData = [b(3,:); t(3,:)];
            
            % Rotate other 2 sides
            b = R*[p.XData(:,1).'; p.YData(:,1).'; p.ZData(:,1).']; % patch bottom
            t = R*[p.XData(:,2).'; p.YData(:,2).'; p.ZData(:,2).']; % patch top
            pb.XData = [b(1,:).', t(1,:).'];
            pb.YData = [b(2,:).', t(2,:).'];
            pb.ZData = [b(3,:).', t(3,:).'];
            
            % Rotate Y axis
            xr = R*[x.XData; x.YData; x.ZData];
            xb.XData = xr(1,:);
            xb.YData = xr(2,:);
            xb.ZData = xr(3,:);
            
            % Rotate X axis
            yr = R*[y.XData; y.YData; y.ZData];
            yb.XData = yr(1,:);
            yb.YData = yr(2,:);
            yb.ZData = yr(3,:);
            
            % Rotate down axis
            zr = R*[z.XData; z.YData; z.ZData];
            zb.XData = zr(1,:);
            zb.YData = zr(2,:);
            zb.ZData = zr(3,:);
            
            GetEuler(app);
        end
        % Compute & display euler angles
        function GetEuler(app)
            R = app.DCM;
            r11 = R(1,1);
            r12 = R(1,2);
            r13 = R(1,3);
            r21 = R(2,1);
            r31 = R(3,1);
            r32 = R(3,2);
            r33 = R(3,3);
            
            % if pitch = -90 or pitch = 90 then gimbal lock occurs. This
            % method from <http://danceswithcode.net/engineeringnotes/
            % rotations_in_3d/rotations_in_3d_part2.html> prevents matrix
            % degeneration in these instances, though the yaw orientation
            % is momentarily lost.
            if r31 == 1.0
                app.yaw = 0.0;
                app.roll = atan2(-r12, -r13);
                app.pitch = -asin(r31);
            elseif r31 == -1.0
                app.yaw = 0.0;
                app.roll = atan2(r12, r13);
                app.pitch = -asin(r31);
            else
                app.roll = atan2(r32, r33);
                app.pitch = -asin(r31);
                app.yaw = atan2(r21, r11);
            end
            
            % write the current euler angles to the roll, pitch and yaw
            % labels
            RollText = rad2deg(app.roll);
            app.RollLabel.Text = sprintf('%0.2f°', RollText);
            Pitchtext = rad2deg(app.pitch);
            app.PitchLabel.Text = sprintf('%0.2f°', Pitchtext);
            YawText = rad2deg(app.yaw);
            app.YawLabel.Text = sprintf('%0.2f°', YawText);
        end
    end
end
