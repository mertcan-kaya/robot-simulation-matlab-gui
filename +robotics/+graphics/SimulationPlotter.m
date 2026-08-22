classdef SimulationPlotter
    % SIMULATIONPLOTTER Comprehensive diagnostic plotting and data export tool
    % for robot trajectory tracking, dynamic performance, torques, and Cartesian errors.
    
    methods (Static)
        function fig = show(logData)
            % SHOW Opens an interactive multi-tab diagnostic scope window.
            if nargin < 1 || isempty(logData) || ~isfield(logData, 'time') || isempty(logData.time)
                warndlg('No simulation log data available. Please run a simulation first.', 'Plot Scope');
                fig = [];
                return;
            end
            
            robotName = 'Robot';
            if isfield(logData, 'robot_name') && ~isempty(logData.robot_name)
                robotName = logData.robot_name;
            end
            
            modeStr = 'Simulation';
            if isfield(logData, 'mode') && ~isempty(logData.mode)
                modeStr = logData.mode;
            end
            
            fig = figure('Name', sprintf('Diagnostic Scope - %s (%s)', robotName, modeStr), ...
                         'NumberTitle', 'off', ...
                         'Color', [0.95 0.95 0.96], ...
                         'Position', [100 80 1100 700]);
            
            % Main layout with top toolbar and TabGroup
            mainLayout = uigridlayout(fig, [2, 1]);
            mainLayout.RowHeight = {40, '1x'};
            mainLayout.ColumnWidth = {'1x'};
            mainLayout.Padding = [10 10 10 10];
            mainLayout.RowSpacing = 8;
            
            % Top action toolbar panel
            toolPanel = uipanel(mainLayout, 'BorderType', 'none', 'BackgroundColor', [0.95 0.95 0.96]);
            btnLayout = uigridlayout(toolPanel, [1, 5]);
            btnLayout.ColumnWidth = {180, 140, 140, '1x', 150};
            btnLayout.Padding = [0 0 0 0];
            
            infoLabel = uilabel(btnLayout, 'Text', sprintf('Duration: %.2f s | Samples: %d', logData.time(end), length(logData.time)), ...
                                'FontWeight', 'bold');
            
            exportMatBtn = uibutton(btnLayout, 'Text', 'Export to .MAT', ...
                                    'ButtonPushedFcn', @(btn, event) robotics.graphics.SimulationPlotter.promptExportMAT(logData));
            exportCsvBtn = uibutton(btnLayout, 'Text', 'Export to .CSV', ...
                                    'ButtonPushedFcn', @(btn, event) robotics.graphics.SimulationPlotter.promptExportCSV(logData));
            
            % Tab Group for Diagnostic Views
            tabGroup = uitabgroup(mainLayout);
            
            % Tab 1: Joint Position & Tracking Error
            tab1 = uitab(tabGroup, 'Title', 'Joint Positions & Error');
            robotics.graphics.SimulationPlotter.plotJointPositions(tab1, logData);
            
            % Tab 2: Actuator Torques & Limits (if dynamic)
            tab2 = uitab(tabGroup, 'Title', 'Actuator Torques');
            robotics.graphics.SimulationPlotter.plotTorques(tab2, logData);
            
            % Tab 3: Joint Velocities & Accelerations
            tab3 = uitab(tabGroup, 'Title', 'Velocities & Accelerations');
            robotics.graphics.SimulationPlotter.plotVelocities(tab3, logData);
            
            % Tab 4: Cartesian Task Space & 3D Path
            tab4 = uitab(tabGroup, 'Title', 'Cartesian Path & Error');
            robotics.graphics.SimulationPlotter.plotCartesianPath(tab4, logData);
            
            % Tab 5: Phase Plane Portraits
            tab5 = uitab(tabGroup, 'Title', 'Phase Portraits');
            robotics.graphics.SimulationPlotter.plotPhasePortraits(tab5, logData);
        end
        
        function plotJointPositions(parentTab, logData)
            delete(parentTab.Children);
            t = logData.time;
            n = size(logData.q_act, 2);
            [nRows, nCols] = robotics.graphics.SimulationPlotter.getSubplotGrid(n);
            
            gl = uigridlayout(parentTab, [nRows, nCols]);
            gl.Padding = [10 10 10 10];
            gl.RowSpacing = 10;
            gl.ColumnSpacing = 10;
            
            for i = 1:n
                ax = uiaxes(gl);
                hold(ax, 'on');
                ax.XGrid = 'on';
                ax.YGrid = 'on';
                
                q_act_deg = rad2deg(logData.q_act(:, i));
                q_des_deg = rad2deg(logData.q_des(:, i));
                err_deg   = q_des_deg - q_act_deg;
                
                yyaxis(ax, 'left');
                plot(ax, t, q_des_deg, 'r--', 'LineWidth', 1.5, 'DisplayName', 'q_{des}');
                plot(ax, t, q_act_deg, 'b-', 'LineWidth', 1.2, 'DisplayName', 'q_{act}');
                ylabel(ax, 'Position [deg]');
                
                yyaxis(ax, 'right');
                plot(ax, t, err_deg, 'g-', 'LineWidth', 1.0, 'DisplayName', 'e_q');
                ylabel(ax, 'Error [deg]');
                
                title(ax, sprintf('Joint %d Tracking (Max |e| = %.3f°)', i, max(abs(err_deg))), 'FontWeight', 'bold');
                xlabel(ax, 'Time [s]');
                legend(ax, 'Location', 'best');
            end
        end
        
        function plotTorques(parentTab, logData)
            delete(parentTab.Children);
            t = logData.time;
            n = size(logData.q_act, 2);
            
            if ~isfield(logData, 'tau') || isempty(logData.tau) || all(logData.tau(:) == 0)
                uilabel(parentTab, 'Text', 'Torque data is available in Dynamic Simulation Mode.', ...
                        'HorizontalAlignment', 'center', 'FontSize', 14);
                return;
            end
            
            [nRows, nCols] = robotics.graphics.SimulationPlotter.getSubplotGrid(n);
            gl = uigridlayout(parentTab, [nRows, nCols]);
            gl.Padding = [10 10 10 10];
            
            for i = 1:n
                ax = uiaxes(gl);
                hold(ax, 'on');
                ax.XGrid = 'on';
                ax.YGrid = 'on';
                
                tau_i = logData.tau(:, i);
                plot(ax, t, tau_i, 'm-', 'LineWidth', 1.5, 'DisplayName', '\tau_{cmd}');
                
                % Overplot zero reference
                yline(ax, 0, 'k:');
                
                title(ax, sprintf('Joint %d Torque (Peak = %.2f Nm)', i, max(abs(tau_i))), 'FontWeight', 'bold');
                xlabel(ax, 'Time [s]');
                ylabel(ax, 'Torque [Nm]');
                legend(ax, 'Location', 'best');
            end
        end
        
        function plotVelocities(parentTab, logData)
            delete(parentTab.Children);
            t = logData.time;
            n = size(logData.q_act, 2);
            [nRows, nCols] = robotics.graphics.SimulationPlotter.getSubplotGrid(n);
            
            gl = uigridlayout(parentTab, [nRows, nCols]);
            gl.Padding = [10 10 10 10];
            
            for i = 1:n
                ax = uiaxes(gl);
                hold(ax, 'on');
                ax.XGrid = 'on';
                ax.YGrid = 'on';
                
                vel_deg = rad2deg(logData.q_vel_act(:, i));
                acc_deg = rad2deg(logData.q_acc_act(:, i));
                
                yyaxis(ax, 'left');
                plot(ax, t, vel_deg, 'b-', 'LineWidth', 1.2, 'DisplayName', '\omega_{act}');
                ylabel(ax, 'Velocity [deg/s]');
                
                yyaxis(ax, 'right');
                plot(ax, t, acc_deg, 'color', [0.85 0.33 0.1], 'LineWidth', 1.0, 'DisplayName', '\alpha_{act}');
                ylabel(ax, 'Acc [deg/s^2]');
                
                title(ax, sprintf('Joint %d Velocity & Acc', i), 'FontWeight', 'bold');
                xlabel(ax, 'Time [s]');
                legend(ax, 'Location', 'best');
            end
        end
        
        function plotCartesianPath(parentTab, logData)
            delete(parentTab.Children);
            t = logData.time;
            gl = uigridlayout(parentTab, [2, 2]);
            gl.Padding = [10 10 10 10];
            
            % 3D Cartesian Path
            ax3D = uiaxes(gl);
            ax3D.Layout.Row = [1 2];
            ax3D.Layout.Column = 1;
            hold(ax3D, 'on');
            ax3D.XGrid = 'on';
            ax3D.YGrid = 'on';
            ax3D.ZGrid = 'on';
            view(ax3D, 3);
            
            p_des = logData.p_des;
            p_act = logData.p_act;
            
            plot3(ax3D, p_des(:, 1), p_des(:, 2), p_des(:, 3), 'r--', 'LineWidth', 2, 'DisplayName', 'Desired Path');
            plot3(ax3D, p_act(:, 1), p_act(:, 2), p_act(:, 3), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Actual Path');
            plot3(ax3D, p_act(1, 1), p_act(1, 2), p_act(1, 3), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8, 'DisplayName', 'Start');
            plot3(ax3D, p_act(end, 1), p_act(end, 2), p_act(end, 3), 'rs', 'MarkerFaceColor', 'r', 'MarkerSize', 8, 'DisplayName', 'End');
            
            title(ax3D, '3D End-Effector Trajectory', 'FontWeight', 'bold');
            xlabel(ax3D, 'X [m]');
            ylabel(ax3D, 'Y [m]');
            zlabel(ax3D, 'Z [m]');
            legend(ax3D, 'Location', 'best');
            
            % Cartesian Euclidean Error vs Time
            axErr = uiaxes(gl);
            axErr.Layout.Row = 1;
            axErr.Layout.Column = 2;
            hold(axErr, 'on');
            axErr.XGrid = 'on';
            axErr.YGrid = 'on';
            
            p_err_mm = logData.p_err * 1000; % Convert to mm
            plot(axErr, t, p_err_mm, 'r-', 'LineWidth', 1.5, 'DisplayName', '||e_p||');
            title(axErr, sprintf('Position Tracking Error (Max = %.2f mm, Mean = %.2f mm)', ...
                                 max(p_err_mm), mean(p_err_mm)), 'FontWeight', 'bold');
            xlabel(axErr, 'Time [s]');
            ylabel(axErr, 'Error [mm]');
            legend(axErr, 'Location', 'best');
            
            % Cartesian X, Y, Z Coordinates vs Time
            axXYZ = uiaxes(gl);
            axXYZ.Layout.Row = 2;
            axXYZ.Layout.Column = 2;
            hold(axXYZ, 'on');
            axXYZ.XGrid = 'on';
            axXYZ.YGrid = 'on';
            
            plot(axXYZ, t, p_act(:, 1), 'r-', 'LineWidth', 1.2, 'DisplayName', 'X_{act}');
            plot(axXYZ, t, p_act(:, 2), 'g-', 'LineWidth', 1.2, 'DisplayName', 'Y_{act}');
            plot(axXYZ, t, p_act(:, 3), 'b-', 'LineWidth', 1.2, 'DisplayName', 'Z_{act}');
            plot(axXYZ, t, p_des(:, 1), 'r:', 'LineWidth', 1.0, 'DisplayName', 'X_{des}');
            plot(axXYZ, t, p_des(:, 2), 'g:', 'LineWidth', 1.0, 'DisplayName', 'Y_{des}');
            plot(axXYZ, t, p_des(:, 3), 'b:', 'LineWidth', 1.0, 'DisplayName', 'Z_{des}');
            
            title(axXYZ, 'Cartesian Coordinates vs Time', 'FontWeight', 'bold');
            xlabel(axXYZ, 'Time [s]');
            ylabel(axXYZ, 'Position [m]');
            legend(axXYZ, 'Location', 'best', 'NumColumns', 2);
        end
        
        function plotPhasePortraits(parentTab, logData)
            delete(parentTab.Children);
            t = logData.time;
            n = size(logData.q_act, 2);
            [nRows, nCols] = robotics.graphics.SimulationPlotter.getSubplotGrid(n);
            
            gl = uigridlayout(parentTab, [nRows, nCols]);
            gl.Padding = [10 10 10 10];
            
            for i = 1:n
                ax = uiaxes(gl);
                hold(ax, 'on');
                ax.XGrid = 'on';
                ax.YGrid = 'on';
                
                pos_err = rad2deg(logData.q_des(:, i) - logData.q_act(:, i));
                vel_err = rad2deg(logData.q_vel_des(:, i) - logData.q_vel_act(:, i));
                
                plot(ax, pos_err, vel_err, 'b-', 'LineWidth', 1.2);
                plot(ax, pos_err(1), vel_err(1), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 6, 'DisplayName', 'Start');
                plot(ax, pos_err(end), vel_err(end), 'rs', 'MarkerFaceColor', 'r', 'MarkerSize', 6, 'DisplayName', 'End');
                
                % Origin marker
                plot(ax, 0, 0, 'k+', 'MarkerSize', 8, 'LineWidth', 1.5, 'DisplayName', 'Target (0,0)');
                
                title(ax, sprintf('Joint %d Phase Portrait', i), 'FontWeight', 'bold');
                xlabel(ax, 'Position Error e [deg]');
                ylabel(ax, 'Velocity Error \ite^. [deg/s]');
                legend(ax, 'Location', 'best');
            end
        end
        
        function [nRows, nCols] = getSubplotGrid(n)
            if n <= 2
                nRows = n; nCols = 1;
            elseif n <= 3
                nRows = 3; nCols = 1;
            elseif n <= 4
                nRows = 2; nCols = 2;
            elseif n <= 6
                nRows = 3; nCols = 2;
            else
                nRows = ceil(n / 2); nCols = 2;
            end
        end
        
        function promptExportMAT(logData)
            [fileName, pathName] = uiputfile('*.mat', 'Save Simulation Log', 'robot_sim_log.mat');
            if isequal(fileName, 0) || isequal(pathName, 0)
                return;
            end
            fullPath = fullfile(pathName, fileName);
            robotics.graphics.SimulationPlotter.exportToMAT(logData, fullPath);
            msgbox(sprintf('Successfully exported log data to:\n%s', fullPath), 'Export Complete');
        end
        
        function promptExportCSV(logData)
            [fileName, pathName] = uiputfile('*.csv', 'Save Simulation CSV Data', 'robot_sim_data.csv');
            if isequal(fileName, 0) || isequal(pathName, 0)
                return;
            end
            fullPath = fullfile(pathName, fileName);
            robotics.graphics.SimulationPlotter.exportToCSV(logData, fullPath);
            msgbox(sprintf('Successfully exported CSV data to:\n%s', fullPath), 'Export Complete');
        end
        
        function exportToMAT(logData, filePath)
            save(filePath, 'logData', '-v7.3');
        end
        
        function exportToCSV(logData, filePath)
            t = logData.time;
            n = size(logData.q_act, 2);
            
            headers = {'Time'};
            dataMat = t;
            
            % Desired positions
            for i = 1:n, headers{end+1} = sprintf('q_des_%d', i); end
            dataMat = [dataMat, logData.q_des];
            
            % Actual positions
            for i = 1:n, headers{end+1} = sprintf('q_act_%d', i); end
            dataMat = [dataMat, logData.q_act];
            
            % Joint error
            for i = 1:n, headers{end+1} = sprintf('q_err_%d', i); end
            dataMat = [dataMat, logData.q_err];
            
            % Velocities
            for i = 1:n, headers{end+1} = sprintf('q_vel_act_%d', i); end
            dataMat = [dataMat, logData.q_vel_act];
            
            % Torques
            if isfield(logData, 'tau') && ~isempty(logData.tau)
                for i = 1:n, headers{end+1} = sprintf('tau_%d', i); end
                dataMat = [dataMat, logData.tau];
            end
            
            % Cartesian positions & error
            headers = [headers, {'p_des_x', 'p_des_y', 'p_des_z', 'p_act_x', 'p_act_y', 'p_act_z', 'p_err'}];
            dataMat = [dataMat, logData.p_des, logData.p_act, logData.p_err];
            
            T = array2table(dataMat, 'VariableNames', headers);
            writetable(T, filePath);
        end
    end
end
