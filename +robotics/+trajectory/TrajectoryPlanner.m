classdef (Abstract) TrajectoryPlanner
    % TRAJECTORYPLANNER Abstract base class for all trajectory generation strategies.
    
    methods (Abstract)
        % computeTime calculates the minimum time required to complete the
        % trajectory given velocity and acceleration limits.
        tf = computeTime(obj, qi, qf, prcnt, velLim, accLim);
        
        % p2pTrj calculates point-to-point interpolation factors (position, 
        % velocity, and acceleration) at a given time t for a total time tf.
        [s_pos, s_vel, s_acc] = p2pTrj(obj, t, tf);
    end
end
