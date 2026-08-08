classdef TrajectoryPlannerFactory
    % TRAJECTORYPLANNERFACTORY Factory class to instantiate the correct 
    % TrajectoryPlanner strategy based on the profile index.
    
    methods (Static)
        function planner = create(trj_profile)
            switch trj_profile
                case 1
                    planner = robotics.trajectory.LinearPlanner();
                case 2
                    planner = robotics.trajectory.CubicPolynomialPlanner();
                case 3
                    planner = robotics.trajectory.QuinticPolynomialPlanner();
                case 4
                    planner = robotics.trajectory.TrapezoidalVelocityPlanner();
                otherwise
                    error('Unknown trajectory profile index: %d', trj_profile);
            end
        end
    end
end

