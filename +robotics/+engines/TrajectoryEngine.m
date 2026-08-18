classdef TrajectoryEngine
    % TRAJECTORYENGINE Static trajectory generation and duration planning algorithms.
    %   Encapsulates joint-space point-to-point path interpolation (Linear, Cubic,
    %   Quintic) and analytical trajectory duration estimation based on dynamic limits.
    
    methods (Static)
        function [q_pos, q_vel, q_acc] = generateTrajectory(trjConfig, kin, ini_q, fin_q, k)
            % GENERATETRAJECTORY Computes desired joint states at discrete time step k.
            %   [Q_POS, Q_VEL, Q_ACC] = GENERATETRAJECTORY(TRJCONFIG, KIN, INI_Q, FIN_Q, K)
            %   evaluates polynomial or linear interpolation at t = k * tstp to yield
            %   the current desired position, velocity, and acceleration.
            %
            %   Inputs:
            %       trjConfig - struct: Timing and profile configuration:
            %                   .tstp        - Time step size in seconds [s]
            %                   .tfin_trj    - Total trajectory duration [s]
            %                   .trj_profile - Profile (0: None, 1: Linear, 2: Cubic, 3: Quintic)
            %       kin       - struct: Kinematic parameters (.n: number of DoF)
            %       ini_q     - (Nx1) double: Initial joint position vector [rad or m]
            %       fin_q     - (Nx1) double: Final target joint position vector [rad or m]
            %       k         - integer scalar: Current discrete simulation step index
            %
            %   Outputs:
            %       q_pos     - (Nx1) double: Desired joint positions at step k [rad or m]
            %       q_vel     - (Nx1) double: Desired joint velocities at step k [rad/s or m/s]
            %       q_acc     - (Nx1) double: Desired joint accelerations at step k [rad/s^2 or m/s^2]
            %
            %   See also computeTime.
            
            t = k * trjConfig.tstp;
            tf = trjConfig.tfin_trj;
            
            if t > tf
                t = tf;
            end
        
            qi = ini_q(:);
            qf = fin_q(:);
            
            if trjConfig.trj_profile ~= 0
                % Joint space trajectory interpolation
                D = qf - qi;
                
                planner = robotics.trajectory.TrajectoryPlannerFactory.create(trjConfig.trj_profile);
                [s_pos, s_vel, s_acc] = planner.p2pTrj(t, tf);
                
                q_pos = qi + s_pos * D;
                q_vel = s_vel * D;
                q_acc = s_acc * D;
            else
                % Without interpolation (step input)
                q_pos = qf;
                q_vel = zeros(kin.n, 1);
                q_acc = zeros(kin.n, 1);
            end
        end
        
        function tf = computeTime(qi, qf, prcnt, trj_profile, velLim, accLim)
            % COMPUTETIME Calculates minimum trajectory duration satisfying kinematic limits.
            %   tf = COMPUTETIME(QI, QF, PRCNT, TRJ_PROFILE, VELLIM, ACCLIM) calculates
            %   the fastest valid execution time tf subject to velocity and acceleration
            %   bounds scaled by speed percentage (prcnt).
            %
            %   Inputs:
            %       qi          - (Nx1) double: Starting joint positions [rad or m]
            %       qf          - (Nx1) double: Target joint positions [rad or m]
            %       prcnt       - double scalar: Speed percentage factor (0 to 1)
            %       trj_profile - integer scalar: Trajectory profile identifier (0, 1, 2, 3)
            %       velLim      - (Nx2) double: Joint velocity limits [min, max] [rad/s]
            %       accLim      - (Nx2) double: Joint acceleration limits [min, max] [rad/s^2]
            %
            %   Outputs:
            %       tf          - double scalar: Required trajectory duration in seconds [s]
            
            if trj_profile == 0
                tf = 0;
            else
                planner = robotics.trajectory.TrajectoryPlannerFactory.create(trj_profile);
                tf = planner.computeTime(qi, qf, prcnt, velLim, accLim);
            end
        end
    end
end

