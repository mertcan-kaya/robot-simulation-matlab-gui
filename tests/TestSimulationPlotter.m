classdef TestSimulationPlotter < matlab.unittest.TestCase
    % TESTSIMULATIONPLOTTER Tests simulation data logging, plotting window, and data export.
    
    properties
        Robot
        Model
        Engine
    end
    
    methods(TestMethodSetup)
        function setupSimulation(testCase)
            testCase.Robot = robotics.models.RobotFactory.create(1); % Franka Emika
            testCase.Model = robotics.models.SimulationModel();
            testCase.Model.kin = testCase.Robot.getKinematicParameters(0);
            testCase.Model.dyn = testCase.Robot.getInertialParameters();
            testCase.Model.ctr = testCase.Robot.getDefaultControlParams(0);
            
            n = testCase.Model.kin.n;
            testCase.Model.ini.q_pos = zeros(n, 1);
            testCase.Model.fin.q_pos = deg2rad([10; 20; -15; 30; 0; 20; 0]);
            testCase.Model.act.q_pos = testCase.Model.ini.q_pos;
            testCase.Model.act.q_vel = zeros(n, 1);
            testCase.Model.act.q_acc = zeros(n, 1);
            testCase.Model.tstp = 0.001;
            testCase.Model.tfin = 0.05;
            testCase.Model.tfin_trj = 0.05;
            testCase.Model.time_const = inf; % Run at maximum speed for testing
            
            testCase.Engine = robotics.engines.SimulationEngine(testCase.Model, testCase.Robot);
        end
    end
    
    methods(Test)
        function testDynamicSimulationLogging(testCase)
            testCase.Engine.runDynamics();
            
            logData = testCase.Model.logData;
            testCase.verifyNotEmpty(logData);
            testCase.verifyTrue(isfield(logData, 'time'));
            testCase.verifyTrue(isfield(logData, 'q_des'));
            testCase.verifyTrue(isfield(logData, 'q_act'));
            testCase.verifyTrue(isfield(logData, 'q_err'));
            testCase.verifyTrue(isfield(logData, 'tau'));
            testCase.verifyTrue(isfield(logData, 'p_des'));
            testCase.verifyTrue(isfield(logData, 'p_act'));
            testCase.verifyTrue(isfield(logData, 'p_err'));
            
            N = length(logData.time);
            testCase.verifyGreaterThan(N, 1);
            testCase.verifyEqual(size(logData.q_act, 1), N);
            testCase.verifyEqual(size(logData.q_act, 2), 7);
            testCase.verifyEqual(size(logData.tau, 1), N);
            testCase.verifyEqual(size(logData.p_err, 1), N);
        end
        
        function testKinematicSimulationLogging(testCase)
            testCase.Engine.runKinematics();
            
            logData = testCase.Model.logData;
            testCase.verifyNotEmpty(logData);
            testCase.verifyEqual(logData.mode, 'Kinematic');
            testCase.verifyGreaterThan(length(logData.time), 1);
        end
        
        function testPlotterFigureCreation(testCase)
            testCase.Engine.runDynamics();
            logData = testCase.Model.logData;
            
            fig = robotics.graphics.SimulationPlotter.show(logData);
            testCase.verifyNotEmpty(fig);
            testCase.verifyTrue(isvalid(fig));
            
            % Close figure after test
            delete(fig);
        end
        
        function testExportToMATAndCSV(testCase)
            testCase.Engine.runDynamics();
            logData = testCase.Model.logData;
            
            tempMat = fullfile(tempdir, 'test_export.mat');
            tempCsv = fullfile(tempdir, 'test_export.csv');
            
            % Clean up existing temp files if any
            if isfile(tempMat), delete(tempMat); end
            if isfile(tempCsv), delete(tempCsv); end
            
            % Export to MAT
            robotics.graphics.SimulationPlotter.exportToMAT(logData, tempMat);
            testCase.verifyTrue(isfile(tempMat));
            
            % Export to CSV
            robotics.graphics.SimulationPlotter.exportToCSV(logData, tempCsv);
            testCase.verifyTrue(isfile(tempCsv));
            
            % Clean up
            if isfile(tempMat), delete(tempMat); end
            if isfile(tempCsv), delete(tempCsv); end
        end
        
        function testPlottingIntoEmbeddedTabs(testCase)
            testCase.Engine.runDynamics();
            logData = testCase.Model.logData;
            
            fig = uifigure('Visible', 'off');
            tg = uitabgroup(fig);
            t1 = uitab(tg, 'Title', 'Joints');
            t2 = uitab(tg, 'Title', 'Torques');
            t3 = uitab(tg, 'Title', 'Cartesian');
            t4 = uitab(tg, 'Title', 'Phase');
            
            robotics.graphics.SimulationPlotter.plotJointPositions(t1, logData);
            testCase.verifyNotEmpty(t1.Children);
            
            robotics.graphics.SimulationPlotter.plotTorques(t2, logData);
            testCase.verifyNotEmpty(t2.Children);
            
            robotics.graphics.SimulationPlotter.plotCartesianPath(t3, logData);
            testCase.verifyNotEmpty(t3.Children);
            
            robotics.graphics.SimulationPlotter.plotPhasePortraits(t4, logData);
            testCase.verifyNotEmpty(t4.Children);
            
            delete(fig);
        end
    end
end
