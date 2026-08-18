classdef RobotFactory
    % ROBOTFACTORY Factory class for instantiating polymorphic robot models.
    %   Provides a centralized instantiation interface to decouple simulation
    %   engines from concrete robot model implementations.
    
    methods (Static)
        function robot = create(robot_model_id, customParams)
            % CREATE Instantiates a concrete RobotModel instance.
            %   robot = CREATE(ROBOT_MODEL_ID, CUSTOMPARAMS) returns an instance of
            %   the requested robot model.
            %
            %   Model IDs:
            %       0 - Custom Robot (user-defined DH and dynamic parameters)
            %       1 - Franka Emika Panda (7-DoF)
            %       2 - Universal Robots UR3 (6-DoF)
            %       3 - Unitree Z1 (6-DoF)
            %       4 - Staubli RX160 (6-DoF)
            %       5 - Staubli RX160L (6-DoF Long Reach)
            %
            %   Inputs:
            %       robot_model_id - integer scalar: Model identifier (0 to 5)
            %       customParams   - (optional) struct: Custom DH/inertial parameters
            %
            %   Outputs:
            %       robot          - Concrete subclass instance of RobotModel
            
            if nargin < 1 || isempty(robot_model_id) || ~isnumeric(robot_model_id)
                robot_model_id = 0;
            end
            
            switch robot_model_id
                case 1
                    robot = robotics.models.FrankaEmika();
                case 2
                    robot = robotics.models.UR3();
                case 3
                    robot = robotics.models.UnitreeZ1();
                case 4
                    robot = robotics.models.StaubliRX160();
                case 5
                    robot = robotics.models.StaubliRX160L();
                case 0
                    if nargin > 1 && ~isempty(customParams)
                        robot = robotics.models.CustomRobot(customParams);
                    else
                        robot = robotics.models.CustomRobot();
                    end
                otherwise
                    warning('RobotFactory:UnknownModelId', ...
                        'Unknown robot_model_id %s. Defaulting to CustomRobot.', num2str(robot_model_id));
                    robot = robotics.models.CustomRobot();
            end
        end
    end
end





