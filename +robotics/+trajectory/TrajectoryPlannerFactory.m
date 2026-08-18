classdef TrajectoryPlannerFactory
    % TRAJECTORYPLANNERFACTORY Factory class to instantiate and cache 
    % TrajectoryPlanner strategies based on the profile index.
    
    methods (Static)
        function planner = create(trj_profile)
            persistent linearPlanner cubicPlanner quinticPlanner trapPlanner
            switch trj_profile
                case 1
                    if isempty(linearPlanner), linearPlanner = robotics.trajectory.LinearPlanner(); end
                    planner = linearPlanner;
                case 2
                    if isempty(cubicPlanner), cubicPlanner = robotics.trajectory.CubicPolynomialPlanner(); end
                    planner = cubicPlanner;
                case 3
                    if isempty(quinticPlanner), quinticPlanner = robotics.trajectory.QuinticPolynomialPlanner(); end
                    planner = quinticPlanner;
                case 4
                    if isempty(trapPlanner), trapPlanner = robotics.trajectory.TrapezoidalVelocityPlanner(); end
                    planner = trapPlanner;
                otherwise
                    error('Unknown trajectory profile index: %d', trj_profile);
            end
        end
    end
end

