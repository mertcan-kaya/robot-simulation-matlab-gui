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

            % Instantiate MainApp2 before each test
            testCase.App = MainApp2;
            
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

            % Verify default OnButton state is false
            testCase.verifyFalse(testCase.App.OnButton.Value);

            % Verify time label is initialized
            testCase.verifyEqual(char(testCase.App.TimeLabel.Text), 'Not running');
        end

        function testRobotModelSwitching(testCase)
            % Test switching across all supported robot models
            items = testCase.App.RobotModelDropDown.Items;
            expectedDoFs = [7, 6, 6, 6, 6, 3];

            for k = 1:numel(items)
                modelName = items{k};
                expectedDoF = expectedDoFs(k);

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
            % Default is Kinematic with OnButton false (trj_on == 0)
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
            % Ensure Kinematic mode is active (starts with OnButton == false)
            testCase.choose(testCase.App.SimModeSwitch, 'Kinematic');
            if testCase.App.OnButton.Value
                testCase.press(testCase.App.OnButton);
            end

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

            % Turn OnButton ON
            testCase.press(testCase.App.OnButton);

            testCase.verifyEqual(testCase.App.controller.model.trj_on, 1);
            testCase.verifyEqual(char(testCase.App.SpeedxSpinner.Enable), 'on');
            testCase.verifyEqual(char(testCase.App.InitialJointTab.Title), 'Initial Joint');
            testCase.verifyEqual(char(testCase.App.InitialTaskTab.Title), 'Initial Task');
            testCase.verifyNotEmpty(testCase.App.FinalJointTab.Parent);
            testCase.verifyNotEmpty(testCase.App.FinalTaskTab.Parent);

            % Turn OnButton back OFF
            testCase.press(testCase.App.OnButton);
            testCase.verifyEqual(testCase.App.controller.model.trj_on, 0);
            testCase.verifyEqual(char(testCase.App.InitialJointTab.Title), 'Joint');
        end

        function testInteractiveDynamicTrackingMode(testCase)
            % Switch to Dynamic mode with OnButton OFF
            testCase.choose(testCase.App.SimModeSwitch, 'Dynamic');
            if testCase.App.OnButton.Value
                testCase.press(testCase.App.OnButton);
            end

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
            % Turn OnButton ON to enable trajectory interpolation and controls
            if ~testCase.App.OnButton.Value
                testCase.press(testCase.App.OnButton);
            end

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
            % Ensure Kinematic mode is selected and OnButton is ON
            testCase.choose(testCase.App.SimModeSwitch, 'Kinematic');
            if ~testCase.App.OnButton.Value
                testCase.press(testCase.App.OnButton);
            end
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
            % Turn OnButton ON so that all 4 tabs are present
            if ~testCase.App.OnButton.Value
                testCase.press(testCase.App.OnButton);
            end

            % Test switching between Joint and Task tabs via RightTabGroup titles
            testCase.choose(testCase.App.RightTabGroup, 'Initial Task');
            testCase.verifyEqual(testCase.App.controller.model.task_mode, 1);

            testCase.choose(testCase.App.RightTabGroup, 'Final Task');
            testCase.verifyEqual(testCase.App.controller.model.task_mode, 2);

            testCase.choose(testCase.App.RightTabGroup, 'Initial Joint');
            testCase.verifyEqual(testCase.App.controller.model.task_mode, 0);
        end

        function testSwitch3DModelWhileRunning(testCase)
            % Turn OnButton ON to configure initial and final tabs
            if ~testCase.App.OnButton.Value
                testCase.press(testCase.App.OnButton);
            end

            % Set distinct initial and final joint configurations
            testCase.choose(testCase.App.RightTabGroup, 'Initial Joint');
            testCase.type(testCase.App.Axis1InitialSpinner, -20);
            
            testCase.choose(testCase.App.RightTabGroup, 'Final Joint');
            testCase.type(testCase.App.Axis1FinalSpinner, 45);

            % Simulate running state with intermediate actual and target positions
            testCase.App.controller.model.running_flag = 1;
            testCase.App.controller.model.act.q_pos(1) = deg2rad(10);
            testCase.App.controller.model.des.q_pos(1) = deg2rad(45);

            % Toggle 3D Models checkbox while running
            testCase.press(testCase.App.DmodelsCheckBox);

            % Verify that the target robot and active states are NOT corrupted or reset to ini
            testCase.verifyEqual(rad2deg(testCase.App.controller.model.act.q_pos(1)), 10, 'AbsTol', 1e-2);
            testCase.verifyEqual(rad2deg(testCase.App.controller.model.fin.q_pos(1)), 45, 'AbsTol', 1e-2);
            testCase.verifyEqual(rad2deg(testCase.App.controller.model.des.q_pos(1)), 45, 'AbsTol', 1e-2);

            % Toggle back to 3D CAD mode
            testCase.press(testCase.App.DmodelsCheckBox);

            testCase.verifyEqual(rad2deg(testCase.App.controller.model.act.q_pos(1)), 10, 'AbsTol', 1e-2);
            testCase.verifyEqual(rad2deg(testCase.App.controller.model.fin.q_pos(1)), 45, 'AbsTol', 1e-2);
            testCase.App.controller.model.running_flag = 0;
        end

        function testControlSpaceSwitching(testCase)
            % Switch to Dynamic mode
            testCase.choose(testCase.App.SimModeSwitch, 'Dynamic');
            
            % Navigate to Control tab (LeftTabGroup tab 4)
            testCase.choose(testCase.App.LeftTabGroup, 4);
            
            % Default is Joint space with 7 rows for Franka
            testCase.verifyEqual(char(testCase.App.ControlSpaceDropDown.Value), 'Joint');
            testCase.verifyEqual(size(testCase.App.GainTable.Data, 1), 7);
            testCase.verifyEqual(testCase.App.controller.model.ctr.space, 0);
            
            % Switch to Task Space
            testCase.choose(testCase.App.ControlSpaceDropDown, 'Task');
            testCase.verifyEqual(testCase.App.controller.model.ctr.space, 1);
            testCase.verifyEqual(size(testCase.App.GainTable.Data, 1), 6);
            testCase.verifyEqual(testCase.App.GainTable.RowName, {'x'; 'y'; 'z'; 'rx'; 'ry'; 'rz'});
            
            % Switch to IDC under Task Space
            testCase.choose(testCase.App.ControlTypeDropDown, 'IDC');
            testCase.verifyEqual(testCase.App.controller.model.ctr.algo, 1);
            testCase.verifyEqual(size(testCase.App.GainTable.Data, 1), 6);
            
            % Switch back to Joint Space
            testCase.choose(testCase.App.ControlSpaceDropDown, 'Joint');
            testCase.verifyEqual(testCase.App.controller.model.ctr.space, 0);
            testCase.verifyEqual(size(testCase.App.GainTable.Data, 1), 7);
        end

        function testTabOrderingAlwaysJointFirstTaskLast(testCase)
            % Test 4-tab trajectory mode ordering (OnButton == true)
            if ~testCase.App.OnButton.Value
                testCase.press(testCase.App.OnButton);
            end

            tabs4 = testCase.App.RightTabGroup.Children;
            testCase.verifyEqual(numel(tabs4), 4);
            testCase.verifyEqual(char(tabs4(1).Title), 'Initial Joint');
            testCase.verifyEqual(char(tabs4(2).Title), 'Final Joint');
            testCase.verifyEqual(char(tabs4(3).Title), 'Initial Task');
            testCase.verifyEqual(char(tabs4(4).Title), 'Final Task');

            % Test 2-tab demo mode ordering (OnButton == false)
            testCase.press(testCase.App.OnButton);
            tabs2 = testCase.App.RightTabGroup.Children;
            testCase.verifyEqual(numel(tabs2), 2);
            testCase.verifyEqual(char(tabs2(1).Title), 'Joint');
            testCase.verifyEqual(char(tabs2(2).Title), 'Task');
        end

        function testEmbeddedScopeTelemetryInMainApp2(testCase)
            % Run a short kinematic simulation
            if ~testCase.App.OnButton.Value
                testCase.press(testCase.App.OnButton);
            end
            testCase.choose(testCase.App.InterpolationDropDown, 'Linear');
            testCase.type(testCase.App.Axis1InitialSpinner, -10);
            testCase.type(testCase.App.VelEditField, 100);
            testCase.press(testCase.App.RunButton);

            % Switch to Scope & Telemetry tab
            testCase.choose(testCase.App.TabGroup, 'Scope & Telemetry');
            testCase.verifyEqual(testCase.App.TabGroup.SelectedTab, testCase.App.ScopeTelemetryTab);

            % Verify sub-tabs render with plot children
            testCase.choose(testCase.App.TabGroup2, 'Joint Tracking & Errors');
            testCase.verifyNotEmpty(testCase.App.JointTrackingErrorsTab.Children);

            testCase.choose(testCase.App.TabGroup2, 'Cartesian Path & Error');
            testCase.verifyNotEmpty(testCase.App.CartesianPathErrorTab.Children);

            testCase.choose(testCase.App.TabGroup2, 'Phase Portraits');
            testCase.verifyNotEmpty(testCase.App.PhasePortraitsTab.Children);

            % Switch back to 3D View tab
            testCase.choose(testCase.App.TabGroup, '3D View');
            testCase.verifyEqual(testCase.App.TabGroup.SelectedTab, testCase.App.DViewTab);
        end

        function testHUDOverlayTelemetry(testCase)
            % Verify HUD panel and labels are initialized
            testCase.verifyTrue(isvalid(testCase.App.HUDPanel));
            testCase.verifyTrue(isvalid(testCase.App.HUDPosLabel));
            testCase.verifyTrue(isvalid(testCase.App.HUDManipLabel));
            testCase.verifyTrue(isvalid(testCase.App.HUDSingularityBadge));
            testCase.verifyTrue(isvalid(testCase.App.HUDLimitBadge));
            testCase.verifyTrue(isvalid(testCase.App.q_actLabel));

            % Verify HUD updates on slider changes
            testCase.type(testCase.App.Axis1InitialSpinner, 30);
            testCase.verifyNotEmpty(testCase.App.HUDPosLabel.Text);
            testCase.verifyNotEmpty(testCase.App.HUDManipLabel.Text);

            % Toggle HUD visibility via UI gesture
            testCase.press(testCase.App.HUDCheckBox);
            testCase.verifyEqual(char(testCase.App.HUDPanel.Visible), 'off');

            testCase.press(testCase.App.HUDCheckBox);
            testCase.verifyEqual(char(testCase.App.HUDPanel.Visible), 'on');
        end

        function testManipulabilityEllipsoidVisualizer(testCase)
            % Verify ellipsoid controls exist
            testCase.verifyTrue(isvalid(testCase.App.EllipsoidCheckBox));
            testCase.verifyTrue(isvalid(testCase.App.EnablemanipulabilityellipsoidMenu));
            testCase.verifyFalse(testCase.App.EllipsoidCheckBox.Value);

            % Toggle Ellipsoid ON via UI gesture
            testCase.press(testCase.App.EllipsoidCheckBox);
            testCase.verifyEqual(testCase.App.controller.model.ellipsoid_on, 1);
            testCase.verifyTrue(~isempty(testCase.App.controller.renderer.EllipsoidSurface) && isvalid(testCase.App.controller.renderer.EllipsoidSurface));
            testCase.verifyEqual(char(testCase.App.controller.renderer.EllipsoidSurface.Visible), 'on');

            % Verify update on slider movement
            testCase.type(testCase.App.Axis1InitialSpinner, 45);
            testCase.verifyEqual(char(testCase.App.controller.renderer.EllipsoidSurface.Visible), 'on');

            % Toggle Ellipsoid OFF via UI gesture
            testCase.press(testCase.App.EllipsoidCheckBox);
            testCase.verifyEqual(testCase.App.controller.model.ellipsoid_on, 0);
            testCase.verifyEqual(char(testCase.App.controller.renderer.EllipsoidSurface.Visible), 'off');
        end

    end
end
