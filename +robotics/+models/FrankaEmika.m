classdef FrankaEmika < robotics.models.RobotModel
    properties
        Name = 'Franka Emika Robot'
    end
    
    methods
        function kin = getKinematicParameters(obj, ee_att)
            DH = [ 0       0     0.333 0
                   0       -pi/2 0     0
                   0       pi/2  0.316 0
                   0.0825  pi/2  0     0
                   -0.0825 -pi/2 0.384 0
                   0       pi/2  0     0
                   0.088   pi/2  0     0
                   0       0     0.107 0    ];
            
            kin.qELim = zeros(1,2);
            if ee_att == 1
                DH(end,:) = DH(end,:) + [0,0,0.115,-pi/4];
                kin.qELim = [0,0.040];
            end
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
        
        function ctr = getDefaultControlParams(obj, algo_id)
            if nargin < 2
                algo_id = 0;
            end
            ctr = robotics.engines.GainTuningEngine.computeOptimalGains(obj, algo_id, 0, 0.001);
        end
        
        function tau_frc = getFrictionTorque(obj, q_vel)
            tau_frc = robotics.friction.frictionFERModel(q_vel);
        end
        
        function [q_posLim, q_posSafeLim, q_velLim, q_velSafeLim, q_accLim] = getJointLimits(obj)
            q_min = [-2.8973   -1.7628   -2.8973   -3.0718   -2.8973   -0.0175  -2.8973]'; % [rad]
            q_min_ = q_min + ones(7,1)*deg2rad(5);
            q_max = [ 2.8973    1.7628    2.8973  -0.0698    2.8973    3.7525    2.8973]'; % [rad]
            q_max_ = q_max - ones(7,1)*deg2rad(5);
            dq_max = [2.1750 2.1750 2.1750 2.1750 2.6100 2.6100 2.6100]';
            dq_max_ = dq_max - ones(7,1)*deg2rad(10);
            dq_min = -dq_max;
            dq_min_ = -dq_max_;
            ddq_max = [15 7.5 10 12.5 15 20 20]';
            ddq_min = -ddq_max; 
            
            q_posLim = [q_min,q_max];
            q_posSafeLim = [q_min_,q_max_];
            q_velLim = [dq_min,dq_max];
            q_velSafeLim = [dq_min_,dq_max_];
            q_accLim = [ddq_min,ddq_max];
        end
        
        function [t_posLim, t_posSafeLim, t_velLim, t_velSafeLim, t_accLim] = getTaskLimits(obj, DH)
            x_max = DH(3,3)+DH(5,3)+DH(8,3); % [m]
            x_max_ = x_max - 0.1; 
            x_min = -x_max; 
            x_min_ = x_min + 0.1;
            y_max = DH(3,3)+DH(5,3)+DH(8,3); 
            y_max_ = y_max - 0.1;
            y_min = -y_max;
            y_min_ = y_min + 0.1;
            z_max = DH(1,3)+DH(3,3)+DH(5,3)+DH(8,3);
            z_max_ = z_max - 0.1;
            z_min = DH(1,3)-DH(3,3)-DH(5,3)-DH(8,3);
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
            m_j = [4.970684;0.646926;3.228604;3.587895;1.225946;1.666555;7.35522e-01];
        
            rj_jcj = zeros(3,1,7);
            rj_jcj(:,:,1) = [3.875e-03;2.081e-03;0];
            rj_jcj(:,:,2) = [-3.141e-03;-2.872e-02;3.495e-03];
            rj_jcj(:,:,3) = [2.7518e-02;3.9252e-02;-6.6502e-02];
            rj_jcj(:,:,4) = [-5.317e-02;1.04419e-01;2.7454e-02];
            rj_jcj(:,:,5) = [-1.1953e-02;4.1065e-02;-3.8437e-02];
            rj_jcj(:,:,6) = [6.0149e-02;-1.4117e-02;-1.0517e-02];
            rj_jcj(:,:,7) = [1.0517e-02;-4.252e-03;6.1597e-02];
            
            Ij_cj = zeros(3,3,7);
            Ij_cj(:,:,1) = robotics.math.symmetrize(7.0337e-01 ,-1.3900e-04 ,6.7720e-03 ...
                                                 ,7.0661e-01  ,1.9169e-02 ...
                                                              ,9.1170e-03);
            Ij_cj(:,:,2) = robotics.math.symmetrize(7.9620e-03 ,-3.9250e-03 ,1.0254e-02 ...
                                                 ,2.8110e-02  ,7.0400e-04 ...
                                                              ,2.5995e-02);
            Ij_cj(:,:,3) = robotics.math.symmetrize(3.7242e-02 ,-4.7610e-03 ,-1.1396e-02 ...
                                                 ,3.6155e-02  ,-1.2805e-02 ...
                                                              ,1.0830e-02);
            Ij_cj(:,:,4) = robotics.math.symmetrize(2.5853e-02 ,7.7960e-03  ,-1.3320e-03 ...
                                                 ,1.9552e-02  ,8.6410e-03 ...
                                                              ,2.8323e-02);
            Ij_cj(:,:,5) = robotics.math.symmetrize(3.5549e-02 ,-2.1170e-03 ,-4.0370e-03 ...
                                                 ,2.9474e-02  ,2.2900e-04 ...
                                                              ,8.6270e-03);
            Ij_cj(:,:,6) = robotics.math.symmetrize(1.9640e-03 ,1.0900e-04  ,-1.1580e-03 ...
                                                 ,4.3540e-03  ,3.4100e-04 ...
                                                              ,5.4330e-03);
            Ij_cj(:,:,7) = robotics.math.symmetrize(1.2516e-02 ,-4.2800e-04 ,-1.1960e-03 ...
                                                 ,1.0027e-02  ,-7.4100e-04 ...
                                                              ,4.8150e-03);
                                                              
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
    end
end






