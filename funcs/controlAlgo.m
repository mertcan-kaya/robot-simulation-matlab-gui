function tau = controlAlgo(ctr,kin,dyn,des,fbk)
    
    % Error Calculation
    err_m = des.q_pos - fbk.q_pos; % e
    errdot_m = des.q_vel - fbk.q_vel; % edot
%     if ctr.algo == 2 % Passivity
% %         errdot_r = errdot_m + ctr.Lm_jnt_psv.*err_m; %s
%         ref.q_vel = des.q_vel + ctr.Lm_jnt_psv.*err_m;
%         ref.q_acc = des.q_acc + ctr.Lm_jnt_psv.*errdot_m;
%         errdot_r = ref.q_vel - fbk.q_pos; % s
%     else
        ctr.errsum = ctr.errsum + err_m*ctr.tcyc;
    % end
    
    % Control Torque
    % if ctr.algo == 2
    %     % Adaptive
    %     tau_pd = ctr.Kp_jnt_psv.*err_m + ctr.Kd_jnt_psv.*errdot_r;
    %     [tau_pass,pj_bar] = ANEA(ctr,kin,fbk.q_pos,fbk.q_vel,ref.q_vel,ref.q_acc,kin.g0,ctr.pj_bar,ctr.Pdiag);
    %     tau_ctrl = tau_pass + tau_pd;
    % elseif ctr.algo == 1
    if ctr.algo == 1
        % CTM
        a_ctrl = ctr.Kp_jnt_idc.*err_m + ctr.Ki_jnt_idc.*ctr.errsum + ctr.Kd_jnt_idc.*errdot_m;
        qdd_ctrl = a_ctrl + des.q_acc;
        tau_ctrl = MNEA(kin,fbk.q_pos,fbk.q_vel,fbk.q_vel,qdd_ctrl,kin.g0,dyn.pj_j);
    else
        % PID
        tau_ctrl = ctr.Kp_jnt_pid.*err_m + ctr.Ki_jnt_pid.*ctr.errsum + ctr.Kd_jnt_pid.*errdot_m;
    end
    
    % Compensation Torque
    tau_comp = zeros(kin.n,1);
    if ctr.algo == 0
        if ctr.comp_grv == 1
            tau_comp = getTauG(kin,fbk.q_pos,kin.g0,dyn.m_j,dyn.dj_j);
        end
    end
    
    % Command Torque
    tau = tau_ctrl + tau_comp;

end
