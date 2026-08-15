classdef TrajectoryEngine
    % TRAJECTORYENGINE Encapsulates trajectory generation algorithms
    % and timing computations for robotic simulations.
    
    methods (Static)
        function [q_pos, q_vel, q_acc] = generateTrajectory(trjConfig, kin, ini_q, fin_q, k)
            % generateTrajectory computes the desired joint position, velocity,
            % and acceleration at step k.
            
            t = k * trjConfig.tstp;
            tf = trjConfig.tfin_trj;
            
            if t > tf
                t = tf;
            end
        
            qi = ini_q(:);
            qf = fin_q(:);
            
            if trjConfig.trj_profile ~= 0
                % With interpolation
                
                % Joint space trajectory
                D = qf - qi;
                
                planner = robotics.trajectory.TrajectoryPlannerFactory.create(trjConfig.trj_profile);
                [s_pos, s_vel, s_acc] = planner.p2pTrj(t, tf);
                
                q_pos = qi + s_pos * D;
                q_vel = s_vel * D;
                q_acc = s_acc * D;
            else
                % Without interpolation
                q_pos = qf;
                q_vel = zeros(kin.n, 1);
                q_acc = zeros(kin.n, 1);
            end
        end
        
        function tf = computeTime(qi, qf, prcnt, trj_profile, velLim, accLim)
            % computeTime calculates the required time to complete a trajectory
            % given velocity and acceleration limits.
            
            if trj_profile == 0
                tf = 0;
            else
                planner = robotics.trajectory.TrajectoryPlannerFactory.create(trj_profile);
                tf = planner.computeTime(qi, qf, prcnt, velLim, accLim);
            end
        end
    end
end

