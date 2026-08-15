classdef CustomRobot < robotics.models.RobotModel
    properties
        Name = 'Custom Robot'
        customParams = []
    end
    
    methods
        function obj = CustomRobot(customParams)
            if nargin > 0
                obj.customParams = customParams;
            end
        end
        
        function kin = getKinematicParameters(obj, ee_att)
            if ~isempty(obj.customParams) && isfield(obj.customParams, 'kin')
                kin = obj.customParams.kin;
                DH = [kin.a_j, kin.alpha_j, kin.d_j, kin.theta_O_j];
            else
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
            end
            
            [kin.q_posLim,kin.q_posSafeLim,kin.q_velLim,kin.q_velSafeLim,kin.q_accLim] = obj.getJointLimits(kin.n, kin.j_type);
            [kin.t_posLim,kin.t_posSafeLim,kin.t_velLim,kin.t_velSafeLim,kin.t_accLim] = obj.getTaskLimits(DH);
        end
        
        function [q_posLim, q_posSafeLim, q_velLim, q_velSafeLim, q_accLim] = getJointLimits(obj, n, j_type)
            if nargin < 2
                if ~isempty(obj.customParams) && isfield(obj.customParams, 'kin') && isfield(obj.customParams.kin, 'n')
                    n = obj.customParams.kin.n;
                else
                    n = 7;
                end
            end
            if nargin < 3
                if ~isempty(obj.customParams) && isfield(obj.customParams, 'kin') && isfield(obj.customParams.kin, 'j_type')
                    j_type = obj.customParams.kin.j_type;
                else
                    j_type = ones(n, 1);
                end
            end
            
            % Revolute joint limits
            qR_min = -pi * ones(n, 1);
            qR_min_ = qR_min + deg2rad(5);
            qR_max = pi * ones(n, 1);
            qR_max_ = qR_max - deg2rad(5);
            dqR_max = deg2rad(200) * ones(n, 1);
            dqR_max_ = dqR_max - deg2rad(10);
            dqR_min = -dqR_max;
            dqR_min_ = -dqR_max_;
            ddqR_max = deg2rad(2000) * ones(n, 1);
            ddqR_min = -ddqR_max;
            
            % Prismatic joint limits
            qP_min = -0.1 * ones(n, 1);
            qP_min_ = qP_min + 0.01;
            qP_max = 0.1 * ones(n, 1);
            qP_max_ = qP_max - 0.01;
            dqP_max = 1 * ones(n, 1);
            dqP_max_ = dqP_max - 0.1;
            dqP_min = -dqP_max;
            dqP_min_ = -dqP_max_;
            ddqP_max = 1 * ones(n, 1);
            ddqP_min = -ddqP_max;
            
            q_posLim = zeros(n, 2);
            q_posSafeLim = zeros(n, 2);
            q_velLim = zeros(n, 2);
            q_velSafeLim = zeros(n, 2);
            q_accLim = zeros(n, 2);
            
            for j = 1:n
                if j_type(j) == 1
                    q_posLim(j,:) = [qR_min(j), qR_max(j)];
                    q_posSafeLim(j,:) = [qR_min_(j), qR_max_(j)];
                    q_velLim(j,:) = [dqR_min(j), dqR_max(j)];
                    q_velSafeLim(j,:) = [dqR_min_(j), dqR_max_(j)];
                    q_accLim(j,:) = [ddqR_min(j), ddqR_max(j)];
                else
                    q_posLim(j,:) = [qP_min(j), qP_max(j)];
                    q_posSafeLim(j,:) = [qP_min_(j), qP_max_(j)];
                    q_velLim(j,:) = [dqP_min(j), dqP_max(j)];
                    q_velSafeLim(j,:) = [dqP_min_(j), dqP_max_(j)];
                    q_accLim(j,:) = [ddqP_min(j), ddqP_max(j)];
                end
            end
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
            if ~isempty(obj.customParams) && isfield(obj.customParams, 'm_j')
                m_j = obj.customParams.m_j;
                rj_jcj = obj.customParams.rj_jcj;
                Ij_cj = obj.customParams.Ij_cj;
            else
                m_j = zeros(7,1);
                rj_jcj = zeros(3,1,7);
                Ij_cj = zeros(3,3,7);
            end
            
            n = length(m_j);
            np = 10;
        
            dyn.m_j = m_j;
            dyn.rj_jcj = rj_jcj;
            dyn.Ij_cj = Ij_cj;
            dyn.Ij_j = zeros(3,3,n);
            dyn.dj_j = zeros(3,1,n);
            dyn.pj_j = zeros(np,1,n);
            for j = 1:n
                S_rj_jcj = robotics.math.SkewSym(rj_jcj(:,:,j));
                dyn.Ij_j(:,:,j) = Ij_cj(:,:,j) - dyn.m_j(j)*S_rj_jcj*S_rj_jcj;
                dyn.dj_j(:,:,j) = dyn.m_j(j)*rj_jcj(:,:,j);
            
                dyn.pj_j(:,:,j) = [robotics.math.SymVec(dyn.Ij_j(:,:,j));dyn.dj_j(:,:,j);dyn.m_j(j)];
            end
        end
        
        function ctr = getDefaultControlParams(obj, algo_id)
            if ~isempty(obj.customParams) && isfield(obj.customParams, 'kin') && isfield(obj.customParams.kin, 'n')
                n = obj.customParams.kin.n;
            else
                n = 7;
            end
            ctr.tcyc = 0.001;
            ctr.Kp_jnt_pid = zeros(n,1);
            ctr.Ki_jnt_pid = zeros(n,1);
            ctr.Kd_jnt_pid = zeros(n,1);
        end
    end
end





