classdef TestMainApp < matlab.uitest.TestCase
    % TESTMAINAPP Automated UI Test Suite for MainApp using matlab.uitest.TestCase
    %
    % Validates App Designer startup, robot model switching, slider manipulations,
    % interpolation profile configurations, and simulation execution.

    properties
        App
    end

    methods(TestMethodSetup)
        function launchApp(testCase)
            % Ensure an active display exists for UI gestures on Linux
            if isunix && ~ismac
                displayEnv = getenv('DISPLAY');
                testCase.assumeNotEmpty(displayEnv, ...
                    'Skipping UI gesture tests: active DISPLAY is required for matlab.uitest gestures.');
            end

            % Instantiate MainApp before each test
            testCase.App = MainApp;
            
            % Ensure the app and its figure are deleted after test execution
            testCase.addTeardown(@delete, testCase.App);
        end
    end

    methods(Test)

        function testAppStartup(testCase)
            % Verify that the UI figure is created and visible
            testCase.verifyTrue(isvalid(testCase.App.UIFigure));
            testCase.verifyEqual(char(testCase.App.UIFigure.Visible), 'on');

            % Verify controller and viewmodel are initialized
            testCase.verifyNotEmpty(testCase.App.controller);
            testCase.verifyNotEmpty(testCase.App.viewModel);

            % Verify default robot is Franka Emika with 7 DoF
            testCase.verifyEqual(char(testCase.App.RobotModelDropDown.Value), 'Franka Emika Robot');
            testCase.verifyEqual(testCase.App.DoFSpinner.Value, 7);

            % Verify time label is initialized
            testCase.verifyEqual(char(testCase.App.TimeLabel.Text), 'Not running');
        end

        function testRobotModelSwitching(testCase)
            % Test switching across all supported robot models
            robotModels = {
                'Franka Emika Robot', 7;
                'Universal Robots UR3', 6;
                'Unitree Z1', 6;
                'Stäubli RX160', 6;
                'Stäubli RX160L', 6;
                'Custom Robot', 3
            };

            for k = 1:size(robotModels, 1)
                modelName = robotModels{k, 1};
                expectedDoF = robotModels{k, 2};

                % Choose robot model from dropdown
                testCase.choose(testCase.App.RobotModelDropDown, modelName);

                % Verify DoF is updated correctly
                testCase.verifyEqual(testCase.App.controller.model.kin.n, expectedDoF);
                testCase.verifyEqual(testCase.App.DoFSpinner.Value, expectedDoF);

                % Verify UI components exist for the active joints
                testCase.verifyNotEmpty(testCase.App.DHTable.Data);
                testCase.verifyNotEmpty(testCase.App.JointPosMaxTable.Data);
            end
        end

        function testSimModeSwitching(testCase)
            % Test toggling between Kinematic and Dynamic simulation
            % Default is Kinematic with OnButton true (trj_on == 1)
            testCase.verifyEqual(testCase.App.controller.model.tstp, 0.001);
            testCase.verifyEqual(testCase.App.StepsEditField.Value, 0.001);
            testCase.verifyEqual(char(testCase.App.OnButton.Enable), 'on');

            % Switch to Dynamic mode
            testCase.choose(testCase.App.SimModeSwitch, 'Dynamic');
            testCase.verifyEqual(testCase.App.controller.model.tstp, 0.0001);
            testCase.verifyEqual(testCase.App.StepsEditField.Value, 0.0001);

            % Switch back to Kinematic mode
            testCase.choose(testCase.App.SimModeSwitch, 'Kinematic');
            testCase.verifyEqual(testCase.App.controller.model.tstp, 0.001);
            testCase.verifyEqual(testCase.App.StepsEditField.Value, 0.001);
            testCase.verifyEqual(char(testCase.App.FinalsEditField.Enable), 'off');
        end

        function testInteractiveKinematicDemoMode(testCase)
            % Test turning OnButton OFF in Kinematic mode
            testCase.choose(testCase.App.SimModeSwitch, 'Kinematic');
            testCase.press(testCase.App.OnButton);

            % Verify trajectory interpolation is OFF and single-robot demo mode is active
            testCase.verifyEqual(testCase.App.controller.model.trj_on, 0);
            testCase.verifyEqual(testCase.App.controller.model.ghost_on, 0);
            testCase.verifyEqual(char(testCase.App.RunButton.Enable), 'off');
            testCase.verifyEqual(char(testCase.App.SpeedxSpinner.Enable), 'off');
            testCase.verifyEqual(testCase.App.FinalsEditField.Value, 0);
            testCase.verifyEqual(char(testCase.App.InitialJointTab.Title), 'Joint');
            testCase.verifyEqual(char(testCase.App.InitialTaskTab.Title), 'Task');
            testCase.verifyEmpty(testCase.App.FinalJointTab.Parent);
            testCase.verifyEmpty(testCase.App.FinalTaskTab.Parent);

            % Verify moving sliders does not calculate trajectory final time
            testCase.type(testCase.App.Axis1InitialSpinner, 45);
            testCase.verifyEqual(testCase.App.FinalsEditField.Value, 0);

            % Turn OnButton back ON
            testCase.press(testCase.App.OnButton);

            testCase.verifyEqual(testCase.App.controller.model.trj_on, 1);
            testCase.verifyEqual(char(testCase.App.SpeedxSpinner.Enable), 'on');
            testCase.verifyEqual(char(testCase.App.InitialJointTab.Title), 'Initial Joint');
            testCase.verifyEqual(char(testCase.App.InitialTaskTab.Title), 'Initial Task');
            testCase.verifyNotEmpty(testCase.App.FinalJointTab.Parent);
            testCase.verifyNotEmpty(testCase.App.FinalTaskTab.Parent);
        end

        function testInteractiveDynamicTrackingMode(testCase)
            % Test turning OnButton OFF in Dynamic mode
            testCase.choose(testCase.App.SimModeSwitch, 'Dynamic');
            testCase.press(testCase.App.OnButton);

            % Verify trajectory interpolation is OFF and dynamic target tracking mode is active
            testCase.verifyEqual(testCase.App.controller.model.trj_on, 0);
            testCase.verifyEqual(testCase.App.controller.model.ghost_on, 1);
            testCase.verifyEqual(char(testCase.App.SpeedxSpinner.Enable), 'on');
            testCase.verifyEqual(char(testCase.App.FinalsEditField.Enable), 'on');
            testCase.verifyEqual(char(testCase.App.FinalJointTab.Title), 'Joint');
            testCase.verifyEqual(char(testCase.App.FinalTaskTab.Title), 'Task');
            testCase.verifyNotEmpty(testCase.App.FinalJointTab.Parent);
            testCase.verifyNotEmpty(testCase.App.FinalTaskTab.Parent);
            testCase.verifyEmpty(testCase.App.InitialJointTab.Parent);
            testCase.verifyEmpty(testCase.App.InitialTaskTab.Parent);

            % Verify modifying target sliders updates fin.q_pos and does not calculate trajectory time
            testCase.type(testCase.App.Axis1FinalSpinner, 30);
            testCase.verifyEqual(rad2deg(testCase.App.controller.model.fin.q_pos(1)), 30, 'AbsTol', 1e-2);

            % Verify setting simulation duration enables RunButton
            testCase.type(testCase.App.FinalsEditField, 5.0);
            testCase.verifyEqual(char(testCase.App.RunButton.Enable), 'on');
        end

        function testInterpolationProfileSelection(testCase)
            % Test trajectory profiles: Linear, Cubic, Quintic
            profiles = {'Linear', 1; 'Cubic', 2; 'Quintic', 3};

            for k = 1:size(profiles, 1)
                profName = profiles{k, 1};
                profCode = profiles{k, 2};

                testCase.choose(testCase.App.InterpolationDropDown, profName);
                testCase.verifyEqual(testCase.App.controller.model.trj_profile, profCode);

                % Verify velocity limits are visible for all profiles
                testCase.verifyEqual(char(testCase.App.VelEditField.Visible), 'on');
            end
        end

        function testJointSliderAndSpinnerInput(testCase)
            % Verify joint spinner interaction updates slider and controller
            targetVal = 20;

            % Update spinner value on active Initial Joint Tab
            testCase.type(testCase.App.Axis1InitialSpinner, targetVal);

            % Verify slider is synchronized
            testCase.verifyEqual(testCase.App.Axis1InitialSlider.Value, targetVal, 'AbsTol', 1e-2);

            % Verify controller joint position updated
            testCase.verifyEqual(rad2deg(testCase.App.controller.model.ini.q_pos(1)), targetVal, 'AbsTol', 1e-2);
        end

        function testKinematicSimulationExecution(testCase)
            % Ensure Kinematic mode is selected
            testCase.choose(testCase.App.SimModeSwitch, 'Kinematic');
            testCase.choose(testCase.App.InterpolationDropDown, 'Linear');

            % Set a non-zero initial joint angle on the currently active Initial Joint Tab
            testCase.type(testCase.App.Axis1InitialSpinner, -30);

            % Set velocity percentage to 100%
            testCase.type(testCase.App.VelEditField, 100);

            % Verify RunButton is now enabled
            testCase.verifyEqual(char(testCase.App.RunButton.Enable), 'on');

            % Press Run button
            testCase.press(testCase.App.RunButton);

            % Verify simulation ran and completed
            testCase.verifyEqual(char(testCase.App.TimeLabel.Text), 'Stopped');
        end

        function testTabSelection(testCase)
            % Test switching between Joint and Task tabs via RightTabGroup index
            % 1: Initial Joint, 2: Final Joint, 3: Initial Task, 4: Final Task
            testCase.choose(testCase.App.RightTabGroup, 3);
            testCase.verifyEqual(testCase.App.controller.model.task_mode, 1);

            testCase.choose(testCase.App.RightTabGroup, 4);
            testCase.verifyEqual(testCase.App.controller.model.task_mode, 2);

            testCase.choose(testCase.App.RightTabGroup, 1);
            testCase.verifyEqual(testCase.App.controller.model.task_mode, 0);
        end

        function testSwitch3DModelWhileRunning(testCase)
            % Set distinct initial and final joint configurations
            testCase.choose(testCase.App.RightTabGroup, 1); % Initial Joint
            testCase.type(testCase.App.Axis1InitialSpinner, -20);
            testCase.choose(testCase.App.RightTabGroup, 2); % Final Joint
            testCase.type(testCase.App.Axis1FinalSpinner, 45);

            % Simulate running state with intermediate actual and target positions
            testCase.App.controller.model.running_flag = 1;
            testCase.App.controller.model.act.q_pos(1) = deg2rad(10);
            testCase.App.controller.model.des.q_pos(1) = deg2rad(45);

            % Toggle 3D Models switch while running
            testCase.choose(testCase.App.DmodelsSwitch, 'Off');

            % Verify that the target robot and active states are NOT corrupted or reset to ini
            testCase.verifyEqual(rad2deg(testCase.App.controller.model.act.q_pos(1)), 10, 'AbsTol', 1e-2);
            testCase.verifyEqual(rad2deg(testCase.App.controller.model.fin.q_pos(1)), 45, 'AbsTol', 1e-2);
            testCase.verifyEqual(rad2deg(testCase.App.controller.model.des.q_pos(1)), 45, 'AbsTol', 1e-2);

            % Toggle back to 3D CAD mode
            testCase.choose(testCase.App.DmodelsSwitch, 'On');

            testCase.verifyEqual(rad2deg(testCase.App.controller.model.act.q_pos(1)), 10, 'AbsTol', 1e-2);
            testCase.verifyEqual(rad2deg(testCase.App.controller.model.fin.q_pos(1)), 45, 'AbsTol', 1e-2);
            testCase.verifyEqual(rad2deg(testCase.App.controller.model.des.q_pos(1)), 45, 'AbsTol', 1e-2);

            testCase.App.controller.model.running_flag = 0;
        end

    end
end
