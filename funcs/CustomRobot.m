classdef CustomRobot < RobotModel
    properties
        Name = 'Custom Robot'
    end
    
    methods
        function kin = getKinematicParameters(obj, ee_att)
            DH = zeros(8,4);
    
            kin.qELim = zeros(1,2);
            kin.j_type = [1;1;1;1;1;1;1];
            
            kin.a_j = DH(:,1);
            kin.alpha_j = DH(:,2);
            kin.d_j = DH(:,3);
            kin.theta_O_j = DH(:,4);
            kin.n = size(DH,1)-1;
            kin.np = 10;
            kin.r1 = 1;
            kin.r2 = 2;
            kin.zj_j = [0;0;1];
            
            [kin.q_posLim,kin.q_posSafeLim,kin.q_velLim,kin.q_velSafeLim,kin.q_accLim] = obj.getJointLimits();
            [kin.t_posLim,kin.t_posSafeLim,kin.t_velLim,kin.t_velSafeLim,kin.t_accLim] = obj.getTaskLimits(DH);
        end
        
        function [q_posLim, q_posSafeLim, q_velLim, q_velSafeLim, q_accLim] = getJointLimits(obj)
            q_min = [-pi   -pi   -pi   -pi   -pi   -pi   -pi]'; % [rad]
            q_min_ = q_min + ones(7,1)*deg2rad(5);
            q_max = [ pi    pi    pi    pi    pi    pi    pi]'; % [rad]
            q_max_ = q_max - ones(7,1)*deg2rad(5);
            dq_max = [pi/2 pi/2 pi/2 pi pi pi pi]';
            dq_max_ = dq_max - ones(7,1)*deg2rad(10);
            dq_min = -dq_max;
            dq_min_ = -dq_max_;
            ddq_max = [10 10 10 20 20 20 20]';
            ddq_min = -ddq_max; 
            
            q_posLim = [q_min,q_max];
            q_posSafeLim = [q_min_,q_max_];
            q_velLim = [dq_min,dq_max];
            q_velSafeLim = [dq_min_,dq_max_];
            q_accLim = [ddq_min,ddq_max];
        end
        
        function [t_posLim, t_posSafeLim, t_velLim, t_velSafeLim, t_accLim] = getTaskLimits(obj, DH)
            x_max = sum(DH(:,1))+sum(DH(:,3)); % [m]
            x_max_ = x_max - 0.1;
            x_min = -x_max;
            x_min_ = x_min + 0.1;
            y_max = sum(DH(:,1))+sum(DH(:,3)); % [m]
            y_max_ = y_max - 0.1;
            y_min = -y_max;
            y_min_ = y_min + 0.1;
            z_max = sum(DH(:,1))+sum(DH(:,3)); % [m]
            z_max_ = z_max - 0.1;
            z_min = -sum(DH(:,1))-sum(DH(:,3)); % [m]
            z_min_ = z_min + 0.1;
            
            dt_max = 1;
            dt_max_ = dt_max - 0.1;
            dt_min = -dt_max;
            dt_min_ = -dt_max_;
            ddt_max = 0.1;
            ddt_min = -ddt_max; 
            
            t_posLim = [x_min,x_max;y_min,y_max;z_min,z_max];
            t_posSafeLim = [x_min_,x_max_;y_min_,y_max_;z_min_,z_max_];
            t_velLim = [dt_min,dt_max];
            t_velSafeLim = [dt_min_,dt_max_];
            t_accLim = [ddt_min,ddt_max];
        end
        
        function dyn = getInertialParameters(obj)
            m_j = zeros(7,1);
            rj_jcj = zeros(3,1,7);
            Ij_cj = zeros(3,3,7);
                                       
            n = length(m_j);
            np = 10;
        
            dyn.m_j = m_j;
            dyn.rj_jcj = rj_jcj;
            dyn.Ij_cj = Ij_cj;
            dyn.Ij_j = zeros(3,3,n);
            dyn.dj_j = zeros(3,1,n);
            dyn.pj_j = zeros(np,1,n);
            for j = 1:n
                S_rj_jcj = SkewSym(rj_jcj(:,:,j));
                dyn.Ij_j(:,:,j) = Ij_cj(:,:,j) - dyn.m_j(j)*S_rj_jcj*S_rj_jcj;
                dyn.dj_j(:,:,j) = dyn.m_j(j)*rj_jcj(:,:,j);
            
                dyn.pj_j(:,:,j) = [SymVec(dyn.Ij_j(:,:,j));dyn.dj_j(:,:,j);dyn.m_j(j)];
            end
        end
        
        function ctr = getDefaultControlParams(obj, algo_id)
            ctr.tcyc = 0.001;
            ctr.Kp_jnt_pid = zeros(7,1);
            ctr.Ki_jnt_pid = zeros(7,1);
            ctr.Kd_jnt_pid = zeros(7,1);
        end
    end
end
