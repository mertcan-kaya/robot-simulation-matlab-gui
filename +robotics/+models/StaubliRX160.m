classdef StaubliRX160 < robotics.models.RobotModel
    properties
        Name = 'Staubli RX160'
    end
    
    methods
        function kin = getKinematicParameters(obj, ee_att)
            DH = [ 0     0     0.550 0
                   0.150 -pi/2 0     -pi/2
                   0.825 0     0     pi/2
                   0     pi/2  0.625 0
                   0     -pi/2 0     0
                   0     pi/2  0     0
                   0     0     0.110 0    ];
    
            if ee_att == 1
                DH(end,:) = DH(end,:) + [0,0,0.0333,0];
            elseif ee_att == 2
                DH(end,:) = DH(end,:) + [0,0,0.0333+0.150,0];
            elseif ee_att == 3
                DH(end,:) = DH(end,:) + [0,0,0.0333+0.181,0];
            end
            
            kin.qELim = zeros(1,2);
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
            q_min = [-2.967060 -2.4 -2.62 -4.71 -1.83 -4.71]';
            q_min_ = q_min + ones(6,1)*deg2rad(5);
            q_max = [2.967060 2.4 2.62 4.71 2.09 4.71]';
            q_max_ = q_max - ones(6,1)*deg2rad(5);
            dq_max = deg2rad([400;400;430;540;475;760]);
            dq_max_ = dq_max - ones(6,1)*deg2rad(10);
            dq_min = -dq_max;
            dq_min_ = -dq_max_;
            ddq_max = [7.9;6.5;10.5;25.2;19.6;41.7];
            ddq_min = -ddq_max; 
            
            q_posLim = [q_min,q_max];
            q_posSafeLim = [q_min_,q_max_];
            q_velLim = [dq_min,dq_max];
            q_velSafeLim = [dq_min_,dq_max_];
            q_accLim = [ddq_min,ddq_max];
        end
        
        function [t_posLim, t_posSafeLim, t_velLim, t_velSafeLim, t_accLim] = getTaskLimits(obj, DH)
            x_max = DH(2,1)+DH(3,1)+DH(4,3)+DH(7,3);
            x_max_ = x_max - 0.1;
            x_min = -x_max;
            x_min_ = x_min + 0.1;
            y_max = DH(2,1)+DH(3,1)+DH(4,3)+DH(7,3);
            y_max_ = y_max - 0.1;
            y_min = -y_max;
            y_min_ = y_min + 0.1;
            z_max = DH(1,3)+DH(3,1)+DH(4,3)+DH(7,3);
            z_max_ = z_max - 0.1;
            z_min = DH(1,3)-DH(3,1)-DH(4,3)-DH(7,3);
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
            m_j = [45.357763; 51.266495; 19.754401; 15.287896; 0.548088; 0.038484];
        
            rj_jcj = zeros(3,1,6);
            rj_jcj(:,:,1) = [0.085479;-0.002547;-0.040488];
            rj_jcj(:,:,2) = [-0.000002;0.264164;0.347704];
            rj_jcj(:,:,3) = [-0.000255;0.016478;-0.003819];
            rj_jcj(:,:,4) = [-0.015515;0.000164;0.340348];
            rj_jcj(:,:,5) = [0.000000;-0.000347;0.023671];
            rj_jcj(:,:,6) = [-0.000245;0.000000;-0.007626];
    
            Ij_cj = zeros(3,3,6);
            Ij_cj(:,:,1) = robotics.math.symmetrize(0.839141,0.022637,0.162457 ...
                                                ,1.030146,0.027416 ...
                                                        ,1.064698);
            Ij_cj(:,:,2) = robotics.math.symmetrize(5.272800,0.000010,0.000017 ...
                                                ,5.536809,-0.008261 ...
                                                        ,0.486191);
            Ij_cj(:,:,3) = robotics.math.symmetrize(0.249388,-0.004901,0.004825  ...
                                                ,0.211780,0.000574  ...
                                                        ,0.238488);
            Ij_cj(:,:,4) = robotics.math.symmetrize(0.331593,0.000055,-0.010688 ...
            	                                    ,0.320895,0.000058 ...
                                                            ,0.086557);
            Ij_cj(:,:,5) = robotics.math.symmetrize(0.000876,0.000000,0.000000 ...
            	                                ,0.000889,0.000005 ...
                                                        ,0.000412);
            Ij_cj(:,:,6) = robotics.math.symmetrize(0.000011,0.000000,0.000000 ...
            	                                ,0.000011,0.000000 ...
                                                        ,0.000021);
                                       
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
            if nargin < 2
                algo_id = 0;
            end
            ctr = robotics.engines.GainTuningEngine.computeOptimalGains(obj, algo_id, 0, 0.004);
        end
        
        function tau_spr = getSpringTorque(obj, q_pos)
            tau_spr = robotics.friction.springModel(q_pos);
        end
        
        function [q_pos, err] = computeAnalyticIK(obj, invGeoConfig, kin, RGoal, tGoal, qPrev)
            [q_pos, err] = obj.invGeoAlgebraic(invGeoConfig, kin, RGoal, tGoal, qPrev, 0);
        end
    end
end





