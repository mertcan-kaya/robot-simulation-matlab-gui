classdef ControlEngine
    % CONTROLENGINE Encapsulates robot control algorithms 
    % including PID, Computed Torque Method (CTM), and Passivity.
    
    methods (Static)
        function tau = computeTorque(ctr, kin, dyn, des, fbk)
            % Compute the required joint torques based on the selected control algorithm
            
            % Error Calculation
            err_m = des.q_pos - fbk.q_pos; % e
            errdot_m = des.q_vel - fbk.q_vel; % edot
            
            ctr.errsum = ctr.errsum + err_m * ctr.tcyc;
            
            % Control Torque
            if ctr.algo == 1
                % CTM (Computed Torque Method)
                a_ctrl = ctr.Kp_jnt_idc.*err_m + ctr.Ki_jnt_idc.*ctr.errsum + ctr.Kd_jnt_idc.*errdot_m;
                qdd_ctrl = a_ctrl + des.q_acc;
                tau_ctrl = robotics.engines.DynamicsEngine.inverseDynamicsMNEA(kin, fbk.q_pos, fbk.q_vel, fbk.q_vel, qdd_ctrl, kin.g0, dyn.pj_j);
            else
                % PID (Proportional-Integral-Derivative)
                tau_ctrl = ctr.Kp_jnt_pid.*err_m + ctr.Ki_jnt_pid.*ctr.errsum + ctr.Kd_jnt_pid.*errdot_m;
            end
            
            % Compensation Torque
            tau_comp = zeros(kin.n, 1);
            if ctr.algo == 0
                if ctr.comp_grv == 1
                    tau_comp = robotics.engines.DynamicsEngine.getTauG(kin, fbk.q_pos, kin.g0, dyn.m_j, dyn.dj_j);
                end
            end
            
            % Command Torque
            tau = tau_ctrl + tau_comp;
        end
    end
end
