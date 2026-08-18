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
            % Default is Kinematic
            testCase.verifyEqual(testCase.App.controller.model.tstp, 0.001);
            testCase.verifyEqual(testCase.App.StepsEditField.Value, 0.001);

            % Switch to Dynamic mode
            testCase.choose(testCase.App.SimModeSwitch, 'Dynamic');
            testCase.verifyEqual(testCase.App.controller.model.tstp, 0.0001);
            testCase.verifyEqual(testCase.App.StepsEditField.Value, 0.0001);
            testCase.verifyEqual(char(testCase.App.FinalsEditField.Enable), 'on');

            % Switch back to Kinematic mode
            testCase.choose(testCase.App.SimModeSwitch, 'Kinematic');
            testCase.verifyEqual(testCase.App.controller.model.tstp, 0.001);
            testCase.verifyEqual(testCase.App.StepsEditField.Value, 0.001);
            testCase.verifyEqual(char(testCase.App.FinalsEditField.Enable), 'off');
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

    end
end
