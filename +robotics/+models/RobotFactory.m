classdef RobotFactory
    methods (Static)
        function robot = create(robot_model_id, customParams)
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
                otherwise
                    if nargin > 1
                        robot = robotics.models.CustomRobot(customParams);
                    else
                        robot = robotics.models.CustomRobot();
                    end
            end
        end
    end
end





