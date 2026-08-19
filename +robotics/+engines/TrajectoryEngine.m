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
        
        function [q_pos, q_vel, q_acc] = generateTrajectoryBatch(trjConfig, kin, ini_q, fin_q, t_vec)
            % GENERATETRAJECTORYBATCH Evaluates trajectory across an entire time vector in a single vectorized call.
            %   [Q_POS, Q_VEL, Q_ACC] = GENERATETRAJECTORYBATCH(TRJCONFIG, KIN, INI_Q, FIN_Q, T_VEC)
            %   evaluates polynomial/linear interpolation for all timesteps in T_VEC simultaneously.
            %
            %   Inputs:
            %       trjConfig - struct: Timing and profile configuration (.tfin_trj, .trj_profile)
            %       kin       - struct: Kinematic parameters (.n: number of DoF)
            %       ini_q     - (Nx1) double: Initial joint position vector [rad or m]
            %       fin_q     - (Nx1) double: Final target joint position vector [rad or m]
            %       t_vec     - (1xM) or (Mx1) double: Vector of evaluation time points [s]
            %
            %   Outputs:
            %       q_pos     - (NxM) double: Desired joint positions across M time steps [rad or m]
            %       q_vel     - (NxM) double: Desired joint velocities across M time steps [rad/s or m/s]
            %       q_acc     - (NxM) double: Desired joint accelerations across M time steps [rad/s^2 or m/s^2]
            
            tf = trjConfig.tfin_trj;
            qi = ini_q(:);
            qf = fin_q(:);
            D = qf - qi;
            M = length(t_vec);
            
            if trjConfig.trj_profile ~= 0
                planner = robotics.trajectory.TrajectoryPlannerFactory.create(trjConfig.trj_profile);
                [s_pos, s_vel, s_acc] = planner.p2pTrj(t_vec(:)', tf);
                
                q_pos = qi + D * s_pos;
                q_vel = D * s_vel;
                q_acc = D * s_acc;
            else
                q_pos = repmat(qf, 1, M);
                q_vel = zeros(kin.n, M);
                q_acc = zeros(kin.n, M);
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
            %       prcnt       - double scalar: Speed percentage factor (0 to 1 or 0 to 100)
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
        
        function tf = computeTaskSpaceTime(ini_x, fin_x, prcnt, trj_profile, taskVelLim, taskAccLim)
            % COMPUTETASKSPACETIME Computes duration satisfying linear & angular task space limits.
            %   tf = COMPUTETASKSPACETIME(INI_X, FIN_X, PRCNT, TRJ_PROFILE, TASKVELLIM, TASKACCLIM)
            %   calculates duration for Cartesian position and orientation trajectory.
            
            if trj_profile == 0
                tf = 0;
                return;
            end
            
            p_ei = ini_x(1:3);
            r_ei = ini_x(4:6);
            p_ef = fin_x(1:3);
            r_ef = fin_x(4:6);
            
            Dp = p_ef - p_ei;
            Dr = r_ef - r_ei;
            D = [Dp; Dr];
            
            % Default task limits if not specified
            if nargin < 5 || isempty(taskVelLim)
                max_tspcVel = [1.0; 1.0; 1.0; 0.8; 0.8; 0.8]; % [m/s; rad/s]
            else
                if size(taskVelLim, 2) == 2
                    max_tspcVel = abs(taskVelLim(:, 2));
                else
                    max_tspcVel = abs(taskVelLim(:));
                end
                if length(max_tspcVel) == 2
                    max_tspcVel = [repmat(max_tspcVel(1), 3, 1); repmat(max_tspcVel(2), 3, 1)];
                elseif length(max_tspcVel) == 1
                    max_tspcVel = repmat(max_tspcVel, 6, 1);
                end
            end
            
            if nargin < 6 || isempty(taskAccLim)
                max_tspcAcc = [8.0; 8.0; 8.0; 2.0; 2.0; 2.0]; % [m/s^2; rad/s^2]
            else
                if size(taskAccLim, 2) == 2
                    max_tspcAcc = abs(taskAccLim(:, 2));
                else
                    max_tspcAcc = abs(taskAccLim(:));
                end
                if length(max_tspcAcc) == 2
                    max_tspcAcc = [repmat(max_tspcAcc(1), 3, 1); repmat(max_tspcAcc(2), 3, 1)];
                elseif length(max_tspcAcc) == 1
                    max_tspcAcc = repmat(max_tspcAcc, 6, 1);
                end
            end
            
            if length(prcnt) == 1
                v_prcnt = prcnt;
                a_prcnt = prcnt;
            else
                v_prcnt = prcnt(1);
                a_prcnt = prcnt(2);
            end
            
            if v_prcnt > 1.0
                v_prcnt = v_prcnt / 100.0;
            end
            if a_prcnt > 1.0
                a_prcnt = a_prcnt / 100.0;
            end
            v_prcnt = max(v_prcnt, 1e-4);
            a_prcnt = max(a_prcnt, 1e-4);
            
            kv = v_prcnt * max_tspcVel;
            ka = a_prcnt * max_tspcAcc;
            
            switch trj_profile
                case 1 % Linear
                    tf = max(abs(D) ./ kv);
                case 2 % Cubic
                    tf = max(max(3 * abs(D) ./ (2 * kv)), max(sqrt(6 * abs(D) ./ ka)));
                case 3 % Quintic
                    tf = max(max(15 * abs(D) ./ (8 * kv)), max(sqrt(10 * abs(D) ./ (sqrt(3) * ka))));
                case 4 % Trapezoidal
                    tf = max(max(2 * abs(D) ./ kv), max(sqrt(4 * abs(D) ./ ka)));
                otherwise
                    tf = 0;
            end
            tf = max(tf, 1e-4);
        end
        
        function [q_pos, q_vel, q_acc, x_pos, x_vel, x_acc] = generateTaskSpaceTrajectory(...
                trjConfig, robot, invConfig, kin, ini_x, fin_x, q_prev, k, eulerSet)
            % GENERATETASKSPACETRAJECTORY Computes Cartesian and resolved joint states at step k.
            %   [Q_POS, Q_VEL, Q_ACC, X_POS, X_VEL, X_ACC] = GENERATETASKSPACETRAJECTORY(...)
            %   evaluates Task Space path interpolation in position and Euler orientation,
            %   resolving joint kinematics via inverse kinematics and Jacobian mapping.
            
            if nargin < 9 || isempty(eulerSet)
                eulerSet = 1;
            end
            
            t = k * trjConfig.tstp;
            tf = trjConfig.tfin_trj;
            if t > tf
                t = tf;
            end
            
            p_ei = ini_x(1:3);
            r_ei = ini_x(4:6);
            p_ef = fin_x(1:3);
            r_ef = fin_x(4:6);
            
            Dp = p_ef - p_ei;
            Dr = r_ef - r_ei;
            
            if trjConfig.trj_profile ~= 0
                planner = robotics.trajectory.TrajectoryPlannerFactory.create(trjConfig.trj_profile);
                [s_pos, s_vel, s_acc] = planner.p2pTrj(t, tf);
                
                t_pos   = p_ei + s_pos * Dp;
                r_posEA = r_ei + s_pos * Dr;
                R_mat   = robotics.math.getRotMatfromEA(r_posEA, eulerSet);
                
                t_vel   = s_vel * Dp;
                r_velEA = s_vel * Dr;
                E_R     = robotics.math.getE_Rmatrix(r_posEA, eulerSet);
                r_vel   = E_R * r_velEA;
                
                t_acc   = s_acc * Dp;
                r_accEA = s_acc * Dr;
                E_RDot  = robotics.math.getE_RDotMatrix(r_posEA, r_velEA, eulerSet);
                r_acc   = E_RDot * r_velEA + E_R * r_accEA;
                
                x_pos = [t_pos; r_posEA];
                x_vel = [t_vel; r_vel];
                x_acc = [t_acc; r_acc];
                
                % Solve inverse kinematics for joint positions
                q_pos = robotics.engines.KinematicsEngine.inverseKinematics(...
                    robot, invConfig, kin, R_mat, t_pos, q_prev);
                
                % Numerical / Resolved rate Jacobian mapping for velocities
                TI_0 = invConfig.TI_0;
                TI_i = robotics.engines.KinematicsEngine.getTransMatrix(...
                    TI_0, kin.a_j, kin.alpha_j, kin.d_j, kin.theta_O_j, kin.j_type, q_pos);
                p_ee = TI_i(1:3, 4, kin.n+2);
                J = zeros(6, kin.n);
                for j = 1:kin.n
                    z_j = TI_i(1:3, 3, j+1);
                    p_j = TI_i(1:3, 4, j+1);
                    if kin.j_type(j) == 1
                        J(:, j) = [cross(z_j, p_ee - p_j); z_j];
                    else
                        J(:, j) = [z_j; zeros(3, 1)];
                    end
                end
                
                % Damped pseudo-inverse for velocity mapping
                lambda2 = 1e-4;
                J_pinv = J' / (J * J' + lambda2 * eye(6));
                q_vel = J_pinv * x_vel;
                q_acc = J_pinv * x_acc;
            else
                x_pos = [p_ef; r_ef];
                x_vel = zeros(6, 1);
                x_acc = zeros(6, 1);
                R_mat = robotics.math.getRotMatfromEA(r_ef, eulerSet);
                q_pos = robotics.engines.KinematicsEngine.inverseKinematics(...
                    robot, invConfig, kin, R_mat, p_ef, q_prev);
                q_vel = zeros(kin.n, 1);
                q_acc = zeros(kin.n, 1);
            end
        end
    end
end

