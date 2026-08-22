classdef ControlEngine
    % CONTROLENGINE Encapsulates robot control algorithms 
    % including Joint and Task Space PID and Computed Torque Method (CTM/IDC).
    
    methods (Static)
        function tau = computeTorque(ctr, kin, dyn, des, fbk, TI_0)
            % Compute the required joint torques based on the selected control algorithm
            % Inputs:
            %   ctr   - struct: Control configuration & gains (algo, space, gains, errsum, etc.)
            %   kin   - struct: Kinematic parameters
            %   dyn   - struct: Dynamic parameters
            %   des   - struct: Desired state (q_pos, q_vel, q_acc, and optional task pose/velocity)
            %   fbk   - struct: Feedback state (q_pos, q_vel)
            %   TI_0  - (4x4) double (optional): Base transform (default eye(4))
            
            if nargin < 6 || isempty(TI_0)
                TI_0 = eye(4);
            end
            
            if isfield(kin, 'g0') && ~isempty(kin.g0)
                g0 = kin.g0;
            else
                g0 = -[0; 0; 9.81];
            end
            
            if ~isfield(ctr, 'space')
                ctr.space = 0; % Default to Joint Space (0: Joint, 1: Task)
            end
            
            if ctr.space == 1
                % ==========================================
                % TASK SPACE CONTROL
                % ==========================================
                % Compute forward kinematics and Jacobian for feedback state
                TI_i = robotics.engines.KinematicsEngine.getTransMatrix(...
                    TI_0, kin.a_j, kin.alpha_j, kin.d_j, kin.theta_O_j, kin.j_type, fbk.q_pos);
                p_fbk = TI_i(1:3, 4, kin.n+2);
                R_fbk = TI_i(1:3, 1:3, kin.n+2);
                
                J = robotics.engines.KinematicsEngine.getGeometricJacobian(TI_0, kin, fbk.q_pos);
                xdot_fbk = J * fbk.q_vel; % 6x1 feedback Cartesian velocity [v; w]
                
                % Desired Task Pose & Velocity
                if isfield(des, 't_pos') && ~isempty(des.t_pos) && isfield(des, 'Re') && ~isempty(des.Re)
                    p_des = des.t_pos;
                    R_des = des.Re;
                else
                    % Derive task pose from des.q_pos
                    T_des_i = robotics.engines.KinematicsEngine.getTransMatrix(...
                        TI_0, kin.a_j, kin.alpha_j, kin.d_j, kin.theta_O_j, kin.j_type, des.q_pos);
                    p_des = T_des_i(1:3, 4, kin.n+2);
                    R_des = T_des_i(1:3, 1:3, kin.n+2);
                end
                
                if isfield(des, 'x_vel') && ~isempty(des.x_vel)
                    xdot_des = des.x_vel;
                else
                    J_des = robotics.engines.KinematicsEngine.getGeometricJacobian(TI_0, kin, des.q_pos);
                    xdot_des = J_des * des.q_vel;
                end
                
                if isfield(des, 'x_acc') && ~isempty(des.x_acc)
                    xddot_des = des.x_acc;
                else
                    xddot_des = zeros(6, 1);
                end
                
                % Task Space Error: Position + Orientation
                err_p = p_des - p_fbk;
                err_o = robotics.engines.KinematicsEngine.computeOrientationError(R_des, R_fbk);
                err_m = [err_p; err_o]; % 6x1
                errdot_m = xdot_des - xdot_fbk; % 6x1
                
                if ~isfield(ctr, 'errsum_tsk') || isempty(ctr.errsum_tsk) || length(ctr.errsum_tsk) ~= 6
                    ctr.errsum_tsk = zeros(6, 1);
                end
                ctr.errsum_tsk = ctr.errsum_tsk + err_m * ctr.tcyc;
                
                if ctr.algo == 1
                    % Task Space Inverse Dynamics Control (IDC / CTC)
                    if ~isfield(ctr, 'Kp_tsk_idc') || isempty(ctr.Kp_tsk_idc)
                        ctr.Kp_tsk_idc = [1600; 1600; 1600; 110; 110; 110];
                        ctr.Ki_tsk_idc = zeros(6, 1);
                        ctr.Kd_tsk_idc = [160; 160; 160; 11; 11; 11];
                    end
                    a_ctrl = ctr.Kp_tsk_idc.*err_m + ctr.Ki_tsk_idc.*ctr.errsum_tsk + ctr.Kd_tsk_idc.*errdot_m;
                    a_sum = a_ctrl + xddot_des;
                    
                    % Damped Least Squares Inverse for Jacobian acceleration mapping
                    JJt = J * J';
                    lambda_sq = 1e-4;
                    if min(svd(J)) < 0.05
                        lambda_sq = 1e-2;
                    end
                    J_dls = J' / (JJt + lambda_sq * eye(6));
                    qdd_ctrl = J_dls * a_sum;
                    
                    tau_ctrl = robotics.engines.DynamicsEngine.inverseDynamicsMNEA(...
                        kin, fbk.q_pos, fbk.q_vel, fbk.q_vel, qdd_ctrl, g0, dyn.pj_j);
                else
                    % Task Space PID (Jacobian Transpose Control)
                    if ~isfield(ctr, 'Kp_tsk_pid') || isempty(ctr.Kp_tsk_pid)
                        ctr.Kp_tsk_pid = [1500; 1500; 1500; 100; 100; 100];
                        ctr.Ki_tsk_pid = zeros(6, 1);
                        ctr.Kd_tsk_pid = [150; 150; 150; 10; 10; 10];
                    end
                    f_ctrl = ctr.Kp_tsk_pid.*err_m + ctr.Ki_tsk_pid.*ctr.errsum_tsk + ctr.Kd_tsk_pid.*errdot_m;
                    tau_ctrl = J' * f_ctrl;
                end
            else
                % ==========================================
                % JOINT SPACE CONTROL
                % ==========================================
                err_m = des.q_pos - fbk.q_pos;
                errdot_m = des.q_vel - fbk.q_vel;
                
                if ~isfield(ctr, 'errsum') || isempty(ctr.errsum) || length(ctr.errsum) ~= kin.n
                    ctr.errsum = zeros(kin.n, 1);
                end
                ctr.errsum = ctr.errsum + err_m * ctr.tcyc;
                
                if ctr.algo == 1
                    % Joint Space CTM / IDC
                    a_ctrl = ctr.Kp_jnt_idc.*err_m + ctr.Ki_jnt_idc.*ctr.errsum + ctr.Kd_jnt_idc.*errdot_m;
                    qdd_ctrl = a_ctrl + des.q_acc;
                    tau_ctrl = robotics.engines.DynamicsEngine.inverseDynamicsMNEA(...
                        kin, fbk.q_pos, fbk.q_vel, fbk.q_vel, qdd_ctrl, g0, dyn.pj_j);
                else
                    % Joint Space PID
                    tau_ctrl = ctr.Kp_jnt_pid.*err_m + ctr.Ki_jnt_pid.*ctr.errsum + ctr.Kd_jnt_pid.*errdot_m;
                end
            end
            
            % Compensation Torque
            tau_comp = zeros(kin.n, 1);
            if ctr.algo == 0
                if isfield(ctr, 'comp_grv') && ctr.comp_grv == 1
                    tau_comp = robotics.engines.DynamicsEngine.getTauG(kin, fbk.q_pos, g0, dyn.m_j, dyn.dj_j);
                end
            end
            
            % Command Torque
            tau = tau_ctrl + tau_comp;
        end
    end
end
