classdef RobotFactory
    methods (Static)
        function robot = create(robot_model_id)
            switch robot_model_id
                case 1
                    robot = FrankaEmika();
                case 2
                    robot = UR3();
                case 3
                    robot = UnitreeZ1();
                case 4
                    robot = StaubliRX160();
                case 5
                    robot = StaubliRX160L();
                otherwise
                    robot = CustomRobot();
            end
        end
    end
end
