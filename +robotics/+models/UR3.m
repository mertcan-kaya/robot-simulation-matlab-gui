classdef UR3 < robotics.models.RobotModel
    properties
        Name = 'Universal Robots UR3'
    end
    
    methods
        function kin = getKinematicParameters(obj, ee_att)
            DH = [ 0       0        0.1519      0
                   0       -pi/2    0           0
                   0.24365 0        0           0
                   0.21325 0        0.11235     0
                   0       pi/2     0.08535     0
                   0       -pi/2    0           0
                   0       0        0.0819      0];
    
            kin.qELim = zeros(1,2);
            if ee_att == 1
                DH(end,:) = DH(end,:) + [0,0,0.1874,0];
                kin.qELim = [0,pi/2];
            end
            kin.j_type = [1;1;1;1;1;1];
            
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
            q_min = [-pi   -pi   -pi   -pi   -pi   -pi]'; % [rad]
            q_min_ = q_min + ones(6,1)*deg2rad(5);
            q_max = [ pi    pi    pi    pi    pi    pi]'; % [rad]
            q_max_ = q_max - ones(6,1)*deg2rad(5);
            dq_max = [pi/2 pi/2 pi/2 pi pi pi]';
            dq_max_ = dq_max - ones(6,1)*deg2rad(10);
            dq_min = -dq_max;
            dq_min_ = -dq_max_;
            ddq_max = [10 10 10 20 20 20]';
            ddq_min = -ddq_max; 
            
            q_posLim = [q_min,q_max];
            q_posSafeLim = [q_min_,q_max_];
            q_velLim = [dq_min,dq_max];
            q_velSafeLim = [dq_min_,dq_max_];
            q_accLim = [ddq_min,ddq_max];
        end
        
        function [t_posLim, t_posSafeLim, t_velLim, t_velSafeLim, t_accLim] = getTaskLimits(obj, DH)
            x_max = DH(3,1)+DH(4,1)+DH(7,3); % [m]
            x_max_ = x_max - 0.1;
            x_min = -x_max;
            x_min_ = x_min + 0.1;
            y_max = DH(3,1)+DH(4,1)+DH(7,3);
            y_max_ = y_max - 0.1;
            y_min = -y_max;
            y_min_ = y_min + 0.1;
            z_max = DH(1,3)+DH(3,1)+DH(4,1)+DH(7,3);
            z_max_ = z_max - 0.1;
            z_min = DH(1,3)-DH(3,1)-DH(4,1)-DH(7,3);
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
            m_j = [2.0;3.42;1.26;0.8;0.8;0.35];
        
            rj_jcj = zeros(3,1,6);
            rj_jcj(:,:,1) = [0;0;0.1198];
            rj_jcj(:,:,2) = [-0.121825;0.0;0.12];
            rj_jcj(:,:,3) = [-0.106625;0.0;0.0275];
            rj_jcj(:,:,4) = [0;0;-0.085];
            rj_jcj(:,:,5) = [0;0;-0.085];
            rj_jcj(:,:,6) = [0;0;-0.082];
    
            Ij_cj = zeros(3,3,6);
            Ij_cj(:,:,1) = robotics.math.symmetrize(0.008093163429399999,0.0,0.0 ...
                                      ,0.008093163429399999,0.0 ...
                                       ,0.005625);
            Ij_cj(:,:,2) = robotics.math.symmetrize(0.021728483221103233,0.0,0.0 ...
                                      ,0.021728483221103233,0.0 ...
                                       ,0.00961875);
            Ij_cj(:,:,3) = robotics.math.symmetrize(0.006546806443776375,0.0,0.0 ...
                                      ,0.006546806443776375,0.0 ...
                                       ,0.00354375);
            Ij_cj(:,:,4) = robotics.math.symmetrize(0.0016106408557434,0.0,0.0 ...
                                      ,0.0016106408557434,0.0 ...
                                       ,0.00225);
            Ij_cj(:,:,5) = robotics.math.symmetrize(0.0015721733711303997,0.0,0.0 ...
                                      ,0.0015721733711303997,0.0 ...
                                       ,0.00225);
            Ij_cj(:,:,6) = robotics.math.symmetrize(0.00013626661215999998,0.0,0.0 ...
                                      ,0.00013626661215999998,0.0 ...
                                       ,0.0001792);
                                       
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
            ctr.tcyc = 0.008;
            ctr.Kp_jnt_pid = [100;100;100;60;50;25];
            ctr.Ki_jnt_pid = [0;0;0;0;0;0];
            ctr.Kd_jnt_pid = [10;10;10;10;7.5;5];
        end
        
        function [q_pos, err] = computeAnalyticIK(obj, invGeoConfig, kin, RGoal, tGoal, qPrev)
            err = 0;
            a_i = kin.a_j;
            alpha_i = kin.alpha_j;
            d_i = kin.d_j;
            theta_i_O = kin.theta_O_j;
            TI_0 = invGeoConfig.TI_0;

            T0_E = TI_0\robotics.math.SO3R3_SE3(RGoal,tGoal);
            
            p15x = T0_E(1,4) - T0_E(1,3)*d_i(7);
            p15y = T0_E(2,4) - T0_E(2,3)*d_i(7);
            p15xy = sqrt(p15y^2+p15x^2);
            
            if p15x < 0
                alpha1 = atan2(p15y,p15x)-pi;
            else
                alpha1 = -atan2(p15y,p15x);
            end
            if d_i(4)/p15xy > 1
                q_pos = qPrev;
                err = 1;
                return
            end
            
            alpha2 = asin(d_i(4)/p15xy);
            q1 = alpha2 + alpha1;
            
            if abs(qPrev(1) - (q1 + 2*pi)) < abs(qPrev(1) - q1)
                q1 = q1 + 2*pi;
            elseif abs(qPrev(1) - (q1 - 2*pi)) < abs(qPrev(1) - q1)
                q1 = q1 - 2*pi;
            end
            
            c1 = cos(q1);
            s1 = sin(q1);
            
            if (T0_E(2,4)*c1 - T0_E(1,4)*s1 - d_i(4))/d_i(7) > 1
                q_pos = qPrev;
                err = 1;
                return
            end
        
            q5 = acos((T0_E(2,4)*c1 - T0_E(1,4)*s1 - d_i(4))/d_i(7));
            q5 = -q5;
            s5 = sin(q5);
        
            q6 = pi+atan2((T0_E(2,2)*c1 - T0_E(1,2)*s1)/s5,(T0_E(1,1)*s1 - T0_E(2,1)*c1)/s5);
            if abs(qPrev(6) - (q6 + 2*pi)) < abs(qPrev(6) - q6)
                q6 = q6 + 2*pi;
            elseif abs(qPrev(6) - (q6 - 2*pi)) < abs(qPrev(6) - q6)
                q6 = q6 - 2*pi;
            end
            
            T0_1 = fwdGeo4Inv(a_i(1),alpha_i(1),d_i(1),theta_i_O(1)+q1);
            T4_5 = fwdGeo4Inv(a_i(5),alpha_i(5),d_i(5),theta_i_O(5)+q5);
            T5_E = fwdGeo4Inv(a_i(6),alpha_i(6),d_i(7),theta_i_O(6)+q6);
        
            T0_5 = T0_E/T5_E;
            T1_5 = T0_1\T0_5;
            T1_4 = T1_5/T4_5;
        
            p14x = T1_4(1,4);
            p14z = T1_4(3,4);
            p14xzsq = p14x^2 + p14z^2;
            p14xz = sqrt(p14xzsq);
        
            p15x = T1_5(1,4);
            p15z = T1_5(3,4);
            p15xzsq = p15x^2 + p15z^2;
        
            gamma = acos((a_i(3)^2 - a_i(4)^2 + p14xzsq) / (2*a_i(3)*p14xz));
            beta = acos((a_i(4)^2 - a_i(3)^2 + p14xzsq) / (2*a_i(4)*p14xz));
        
            if imag(gamma) > 0 || imag(beta) > 0
                q_pos = qPrev;
                err = 1;
                return
            end
        
            if p14x < 0
                phi = pi-atan2(p14z,p14x);
                q2 = gamma+phi-pi/2;
            else
                phi = atan2(p14z,p14x);
                q2 = gamma+phi-pi/2;
                q2 = -q2;
            end
         
            if abs(qPrev(2) - (q2 + 2*pi)) < abs(qPrev(2) - q2)
                q2 = q2 + 2*pi;
            elseif abs(qPrev(2) - (q2 - 2*pi)) < abs(qPrev(2) - q2)
                q2 = q2 - 2*pi;
            end
            
            q3 = gamma + beta;
            if p14x < 0
                q3 = -q3;
            end
            
            epsilon = acos((p14xzsq + d_i(5)^2 - p15xzsq) / (2*d_i(5)*p14xz));
            
            q4 = pi-beta-epsilon;
            if p14x < 0
                q4 = -q4;
            end
            
            q_pos = [q1;q2;q3;q4;q5;q6];
            
            function Th_i = fwdGeo4Inv(a_i,alpha_i,d_i,theta_i)
                Rot_x_h = [1 0 0 0; 0 cos(alpha_i) -sin(alpha_i) 0; 0 sin(alpha_i) cos(alpha_i) 0; 0 0 0 1];
                Trn_x_h = [1 0 0 a_i; 0 1 0 0; 0 0 1 0; 0 0 0 1];
                Rot_z_i = [cos(theta_i) -sin(theta_i) 0 0; sin(theta_i) cos(theta_i) 0 0; 0 0 1 0; 0 0 0 1];
                Trn_z_i = [1 0 0 0; 0 1 0 0; 0 0 1 d_i; 0 0 0 1];
                Th_i = Rot_x_h * Trn_x_h * Rot_z_i * Trn_z_i;
            end
        end
    end
end





