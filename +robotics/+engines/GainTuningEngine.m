classdef GainTuningEngine
    % GAINTUNINGENGINE Computes model-based, critically-damped and highly stable controller gains
    % for robot manipulators across Joint Space and Task Space PID & IDC controllers.
    
    methods (Static)
        function ctr = computeOptimalGains(robot, algo_id, space, tcyc)
            % COMPUTEOPTIMALGAINS Computes complete set of stable optimal gains for a robot model.
            % Inputs:
            %   robot   - robotics.models.RobotModel instance
            %   algo_id - integer (0: PID, 1: IDC / CTC) (optional, default 0)
            %   space   - integer (0: Joint Space, 1: Task Space) (optional, default 0)
            %   tcyc    - double (sampling cycle time in seconds) (optional)
            
            if nargin < 2 || isempty(algo_id)
                algo_id = 0;
            end
            if nargin < 3 || isempty(space)
                space = 0;
            end
            if nargin < 4 || isempty(tcyc)
                if isa(robot, 'robotics.models.UR3')
                    tcyc = 0.008;
                elseif isa(robot, 'robotics.models.StaubliRX160') || isa(robot, 'robotics.models.StaubliRX160L')
                    tcyc = 0.004;
                else
                    tcyc = 0.001;
                end
            end
            
            kin = robot.getKinematicParameters(0);
            dyn = robot.getInertialParameters();
            
            ctr.tcyc = tcyc;
            ctr.algo = algo_id;
            ctr.space = space;
            ctr.comp_grv = 1;
            
            % 1. Joint Space IDC Gains (Decoupled feedback linearization)
            [Kp_jnt_idc, Ki_jnt_idc, Kd_jnt_idc] = robotics.engines.GainTuningEngine.computeJointIDCGains(kin, tcyc);
            ctr.Kp_jnt_idc = Kp_jnt_idc;
            ctr.Ki_jnt_idc = Ki_jnt_idc;
            ctr.Kd_jnt_idc = Kd_jnt_idc;
            
            % 2. Joint Space PID Gains (Inertia-scaled with damping margin)
            [Kp_jnt_pid, Ki_jnt_pid, Kd_jnt_pid] = robotics.engines.GainTuningEngine.computeJointPIDGains(kin, dyn, tcyc);
            ctr.Kp_jnt_pid = Kp_jnt_pid;
            ctr.Ki_jnt_pid = Ki_jnt_pid;
            ctr.Kd_jnt_pid = Kd_jnt_pid;
            
            % 3. Task Space IDC Gains (Decoupled Cartesian acceleration dynamics)
            [Kp_tsk_idc, Ki_tsk_idc, Kd_tsk_idc] = robotics.engines.GainTuningEngine.computeTaskIDCGains(tcyc);
            ctr.Kp_tsk_idc = Kp_tsk_idc;
            ctr.Ki_tsk_idc = Ki_tsk_idc;
            ctr.Kd_tsk_idc = Kd_tsk_idc;
            
            % 4. Task Space PID Gains (Virtual Cartesian impedance)
            [Kp_tsk_pid, Ki_tsk_pid, Kd_tsk_pid] = robotics.engines.GainTuningEngine.computeTaskPIDGains(kin, dyn, tcyc);
            ctr.Kp_tsk_pid = Kp_tsk_pid;
            ctr.Ki_tsk_pid = Ki_tsk_pid;
            ctr.Kd_tsk_pid = Kd_tsk_pid;
        end
        
        function [Kp, Ki, Kd] = computeJointIDCGains(kin, tcyc)
            % Compute smooth, well-damped IDC gains: wn ~ 10-14 rad/s, zeta ~ 1.1
            n = kin.n;
            % Bound continuous natural frequency safely within discrete Nyquist bandwidth
            wn_base = min(12, pi / (12 * tcyc));
            
            Kp = zeros(n, 1);
            Kd = zeros(n, 1);
            Ki = zeros(n, 1);
            
            for i = 1:n
                if i <= 3
                    wn = wn_base;
                else
                    wn = 1.15 * wn_base;
                end
                
                % Over-damped (zeta = 1.1) for zero overshoot in simulation
                zeta = 1.1;
                Kp(i) = round(wn^2);
                Kd(i) = round(2 * zeta * wn);
                Ki(i) = 0;
            end
        end
        
        function [Kp, Ki, Kd] = computeJointPIDGains(kin, dyn, tcyc)
            % Compute Joint PID gains scaled by effective joint inertia with safe saturation
            n = kin.n;
            
            % Evaluate effective diagonal inertia across key configurations
            if ~isfield(kin, 'q_posLim') || isempty(kin.q_posLim) || size(kin.q_posLim, 1) < n
                kin.q_posLim = [-pi*ones(n, 1), pi*ones(n, 1)];
            end
            
            q_samples = [zeros(n, 1), ...
                         mean(kin.q_posLim, 2), ...
                         kin.q_posLim(:, 1) + 0.25 * (kin.q_posLim(:, 2) - kin.q_posLim(:, 1)), ...
                         kin.q_posLim(:, 1) + 0.75 * (kin.q_posLim(:, 2) - kin.q_posLim(:, 1))];
            
            M_diag_max = zeros(n, 1);
            g_zero = zeros(3, 1);
            
            for s = 1:size(q_samples, 2)
                q = q_samples(:, s);
                for i = 1:n
                    ei = zeros(n, 1);
                    ei(i) = 1;
                    tau_i = robotics.engines.DynamicsEngine.inverseDynamicsMNEA(...
                        kin, q, zeros(n, 1), zeros(n, 1), ei, g_zero, dyn.pj_j);
                    M_diag_max(i) = max(M_diag_max(i), tau_i(i));
                end
            end
            
            M_diag_max = max(M_diag_max, 0.05);
            
            wn_base = min(10, pi / (16 * tcyc));
            
            Kp = zeros(n, 1);
            Kd = zeros(n, 1);
            Ki = zeros(n, 1);
            
            for i = 1:n
                if i <= 3
                    wn = wn_base;
                    % Heavy joints: cap Kp to prevent torque saturation
                    raw_kp = M_diag_max(i) * wn^2;
                    kp_val = min(1200, max(80, raw_kp));
                else
                    wn = 1.2 * wn_base;
                    raw_kp = M_diag_max(i) * wn^2;
                    kp_val = min(200, max(25, raw_kp));
                end
                
                % Compute well-damped Kd from actual Kp and M_diag (zeta ~ 1.2)
                zeta = 1.2;
                kd_val = 2 * zeta * sqrt(kp_val * M_diag_max(i));
                
                Kp(i) = round(kp_val, 1);
                Kd(i) = round(kd_val, 2);
                Ki(i) = 0;
            end
        end
        
        function [Kp, Ki, Kd] = computeTaskIDCGains(tcyc)
            % Compute decoupled Cartesian acceleration error gains (moderate bandwidth)
            % Translation: wn = 12 rad/s, zeta = 1.1 -> Kp = 144, Kd = 26
            % Rotation:    wn = 8 rad/s,  zeta = 1.1 -> Kp = 64,  Kd = 18
            
            Kp_trans = 150;
            Kd_trans = 26;
            
            Kp_rot   = 75;
            Kd_rot   = 18;
            
            Kp = [Kp_trans; Kp_trans; Kp_trans; Kp_rot; Kp_rot; Kp_rot];
            Kd = [Kd_trans; Kd_trans; Kd_trans; Kd_rot; Kd_rot; Kd_rot];
            Ki = zeros(6, 1);
        end
        
        function [Kp, Ki, Kd] = computeTaskPIDGains(kin, dyn, tcyc)
            % Compute compliant, well-damped Cartesian impedance for Jacobian Transpose Control
            total_mass = sum(dyn.m_j);
            
            % Smooth translational stiffness (scaled smoothly with robot mass, capped at 800 N/m)
            Kp_trans = round(min(800, max(400, total_mass * 25)));
            Kd_trans = round(2 * 1.1 * sqrt(Kp_trans * total_mass * 0.15));
            
            Kp_rot   = 60;
            Kd_rot   = 10;
            
            Kp = [Kp_trans; Kp_trans; Kp_trans; Kp_rot; Kp_rot; Kp_rot];
            Kd = [Kd_trans; Kd_trans; Kd_trans; Kd_rot; Kd_rot; Kd_rot];
            Ki = zeros(6, 1);
        end
    end
end
