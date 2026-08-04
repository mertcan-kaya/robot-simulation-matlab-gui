classdef UnitreeZ1 < RobotModel
    properties
        Name = 'Unitree Z1'
    end
    
    methods
        function kin = getKinematicParameters(obj, ee_att)
            DH = [ 0        0       0.1035      0
                   0        -pi/2   0           pi
                   0.350    0       0           pi-0.2563
                   0.2253   0       0           0.2563
                   0.072    pi/2    0           pi/2
                   0        pi/2    0           0
                   0        0       0.0982      0];
    
            kin.qELim = zeros(1,2);
            if ee_att == 1
                DH(end,:) = DH(end,:) + [0,0,0.149,0];
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
            q_min = [-2.6180 0 -2.8798 -1.3963 -1.4835 -2.7925]';
            q_min_ = q_min + ones(6,1)*deg2rad(5);
            q_max = [2.6180 3.1416 0 1.3963 1.4835 2.7925]';
            q_max_ = q_max - ones(6,1)*deg2rad(5);
            dq_max = [pi/2 pi/2 pi/2 pi/2 pi/2 pi/2]';
            dq_max_ = dq_max - ones(6,1)*deg2rad(10);
            dq_min = -dq_max;
            dq_min_ = -dq_max_;
            ddq_max = [10 10 10 10 10 10]';
            ddq_min = -ddq_max; 
            
            q_posLim = [q_min,q_max];
            q_posSafeLim = [q_min_,q_max_];
            q_velLim = [dq_min,dq_max];
            q_velSafeLim = [dq_min_,dq_max_];
            q_accLim = [ddq_min,ddq_max];
        end
        
        function [t_posLim, t_posSafeLim, t_velLim, t_velSafeLim, t_accLim] = getTaskLimits(obj, DH)
            x_max = DH(3,1)+DH(4,1)+DH(5,1)+DH(7,3);
            x_max_ = x_max - 0.1;
            x_min = -x_max;
            x_min_ = x_min + 0.1;
            y_max = DH(3,1)+DH(4,1)+DH(5,1)+DH(7,3);
            y_max_ = y_max - 0.1;
            y_min = -y_max;
            y_min_ = y_min + 0.1;
            z_max = DH(1,3)+DH(3,1)+DH(4,1)+DH(5,1)+DH(7,3);
            z_max_ = z_max - 0.1;
            z_min = DH(1,3)-DH(3,1)-DH(4,1)-DH(5,1)-DH(7,3);
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
            m_j = [0.67332551;1.19132258;0.83940874;0.56404563;0.38938492;0.28875807];
        
            rj_jcj = zeros(3,1,6);
            rj_jcj(:,:,1) = [2.47e-06;-0.00025198;0.02317169];
            rj_jcj(:,:,2) = [-0.11012601;0.00240029;0.00158266];
            rj_jcj(:,:,3) = [0.10609208;-0.00541815;0.03476383];
            rj_jcj(:,:,4) = [0.04366681;0.00364738;-0.00170192];
            rj_jcj(:,:,5) = [0.03121533;0.0;0.00646316];
            rj_jcj(:,:,6) = [0.0241569 -0.00017355 -0.00143876];
    
            Ij_cj = zeros(3,3,6);
            Ij_cj(:,:,1) = symmetrize(0.00128328,-6e-08,-4e-07 ...
                                      ,0.00071931,5e-07 ...
                                       ,0.00083936);
            Ij_cj(:,:,2) = symmetrize(0.00102138,0.00062358,5.13e-06 ...
                                      ,0.02429457,-2.1e-06 ...
                                       ,0.02466114);
            Ij_cj(:,:,3) = symmetrize(0.00108061,-8.669e-05,-0.00208102 ...
                                      ,0.00954238,-1.332e-05 ...
                                       ,0.00886621);
            Ij_cj(:,:,4) = symmetrize(0.00031576,8.13e-05,4.091e-05 ...
                                      ,0.00092996,-5.96e-06 ...
                                       ,0.00097912);
            Ij_cj(:,:,5) = symmetrize(0.00017605,4e-07,5.689e-05 ...
                                      ,0.00055896,-1.3e-07 ...
                                       ,0.0005386);
            Ij_cj(:,:,6) = symmetrize(0.00018328,1.22e-06,5.4e-07 ...
                                      ,0.0001475,8e-08 ...
                                       ,0.0001468);
                                       
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
            ctr.Kp_jnt_pid = [100;100;100;60;50;25];
            ctr.Ki_jnt_pid = [0;0;0;0;0;0];
            ctr.Kd_jnt_pid = [10;10;10;10;7.5;5];
        end
    end
end
