classdef (Abstract) RobotModel < handle
    % ROBOTMODEL Abstract base class defining the standard interface for all robot arms.
    %   Subclasses implement kinematic parameters (DH tables), joint/task limits,
    %   inertial parameters (mass, center of mass, inertia tensor), controller presets,
    %   and specialized inverse kinematics routines (analytical or numerical).
    
    properties (Abstract)
        Name
    end
    
    methods (Abstract)
        % GETKINEMATICPARAMETERS Returns robot kinematic struct (MDH parameters, limits).
        kin = getKinematicParameters(obj, ee_att)
        
        % GETJOINTLIMITS Returns position, velocity, and acceleration joint limits.
        [q_posLim, q_posSafeLim, q_velLim, q_velSafeLim, q_accLim] = getJointLimits(obj)
        
        % GETTASKLIMITS Returns position, velocity, and acceleration Cartesian task space limits.
        [t_posLim, t_posSafeLim, t_velLim, t_velSafeLim, t_accLim] = getTaskLimits(obj, DH)
        
        % GETINERTIALPARAMETERS Returns link mass, center of mass, and spatial inertia parameters.
        dyn = getInertialParameters(obj)
        
        % GETDEFAULTCONTROLPARAMS Returns default feedback and feedforward controller gains.
        ctr = getDefaultControlParams(obj, algo_id)
    end
    
    methods
        function tau_frc = getFrictionTorque(obj, q_vel)
            % GETFRICTIONTORQUE Computes joint friction torques.
            tau_frc = robotics.friction.frictionFERModelUni(q_vel);
        end
        
        function tau_spr = getSpringTorque(obj, q_pos)
            % GETSPRINGTORQUE Computes gravity compensation spring torques.
            tau_spr = zeros(size(q_pos));
        end
        
        function [qDes, err] = computeInverseKinematics(obj, invGeoConfig, kin, RGoal, tGoal, q0)
            % COMPUTEINVERSEKINEMATICS Dispatches to analytic or numerical inverse kinematics.
            err = 0;
            if invGeoConfig.inv_geo_type == 1
                qDes = obj.invGeoNumeric(invGeoConfig, kin, RGoal, tGoal, q0);
            else
                % Try analytic IK first
                [qDes, err] = obj.computeAnalyticIK(invGeoConfig, kin, RGoal, tGoal, q0);
                % Fallback to numeric if analytic fails or is unavailable
                if err == 1
                    qDes = obj.invGeoNumeric(invGeoConfig, kin, RGoal, tGoal, q0);
                end
            end
        end
        
        function [qDes, err] = computeAnalyticIK(obj, invGeoConfig, kin, RGoal, tGoal, q0)
            % COMPUTEANALYTICIK Base method for closed-form analytical inverse kinematics.
            err = 1;
            qDes = q0;
        end
        
        function qDes = invGeoNumeric(obj, invGeoConfig, kin, RGoal, pGoal, q0)
            a_i = kin.a_j;
            alpha_i = kin.alpha_j;
            d_i = kin.d_j;
            thetaPlus_i = kin.theta_O_j;
            TI_0 = invGeoConfig.TI_0;
            q_posLim = kin.q_posLim;

            errNormPrev = 2;
            errNorm = 1;
            itr = 0;
            tol = 1e-5;

            if invGeoConfig.inv_geo_trn == 0
                kp = invGeoConfig.kp_inv;
                kr = invGeoConfig.kr_inv;
            else
                kp = invGeoConfig.kp_trn;
                kr = invGeoConfig.kr_trn;
            end
            k = [kp;kp;kp;kr;kr;kr];

            J = zeros(6,kin.n);

            qItr = q0;
            while abs(errNorm-errNormPrev) > tol
                
                if itr > 5000
                    break;
                end

                TItr = robotics.engines.KinematicsEngine.getTransMatrix(TI_0,a_i,alpha_i,d_i,thetaPlus_i,kin.j_type,qItr);

                for i = 1:kin.n
                    Jv = cross(TItr(1:3,3,i+1),(TItr(1:3,4,kin.n+2)-TItr(1:3,4,i+1)));
                    Jw = TItr(1:3,3,i+1);
                    J(:,i) = [Jv;Jw];
                end

                pItr = TItr(1:3,4,kin.n+2);
                pErr = pGoal-pItr;
                rErr = 0.5*(cross(TItr(1:3,1,kin.n+2),RGoal(1:3,1))+cross(TItr(1:3,2,kin.n+2),RGoal(1:3,2))+cross(TItr(1:3,3,kin.n+2),RGoal(1:3,3)));
                xErr = [pErr;rErr];
                errNormPrev = errNorm;
                errNorm = norm(xErr);
                
                if invGeoConfig.inv_geo_trn == 0
                    qItr = qItr + J'*(k.*xErr);
                else
                    qItr = qItr + J\(k.*xErr);
                end

                itr = itr + 1;
            end

            for i = 1:kin.n
                if qItr(i) < q_posLim(i,1)
                    qItr(i) = q_posLim(i,1);
                elseif qItr(i) > q_posLim(i,2)
                    qItr(i) = q_posLim(i,2);
                end
            end

            qDes = qItr;
        end
        
        function [q_i, err] = invGeoAlgebraic(obj, invGeoConfig, kin, RGoal, tGoal, q_ref, config)
            err = 0;

            a_i = kin.a_j;
            alpha_i = kin.alpha_j;
            d_i = kin.d_j;
            thetaPlus_i = kin.theta_O_j;
            pI_0 = invGeoConfig.TI_0(1:3,4);
            q_posLim = kin.q_posLim;

            Px = tGoal(1,1) - RGoal(1,3) * d_i(7) - pI_0(1);
            Py = tGoal(2,1) - RGoal(2,3) * d_i(7) - pI_0(2);
            Pz = tGoal(3,1) - RGoal(3,3) * d_i(7) - (pI_0(3) + d_i(1));

            % theta 1
            q1 = atan2(Py,Px);

            if q1 < q_posLim(1,1)-thetaPlus_i(1)
                q1 = q_posLim(1,1)-thetaPlus_i(1);
            elseif q1 > q_posLim(1,2)-thetaPlus_i(1)
                q1 = q_posLim(1,2)-thetaPlus_i(1);
            end

            % theta 2 & 3
            W = -d_i(4);
            X = -cos(q1)*Px - sin(q1)*Py + a_i(2);
            Y = Pz;
            Z1 = -a_i(3);
            Z2 = 0;

            B1 = 2*(Z1*Y+Z2*X);
            B2 = 2*(Z1*X-Z2*Y);
            B3 = W^2-X^2-Y^2-Z1^2-Z2^2;

            C2 = (B2*B3-B1*sqrt(B1^2+B2^2-B3^2))/(B1^2+B2^2);
            S2 = (B1*B3+B2*sqrt(B1^2+B2^2-B3^2))/(B1^2+B2^2);

            if imag(C2) > 0 || imag(S2) > 0
                q_i = q_ref;
                err = 1;
                return
            end
            q2 = atan2(S2,C2);

            if q2 < q_posLim(2,1)-thetaPlus_i(2)
                q2 = q_posLim(2,1)-thetaPlus_i(2);
            elseif q2 > q_posLim(2,2)-thetaPlus_i(2)
                q2 = q_posLim(2,2)-thetaPlus_i(2);
            end
            S3 = (X*C2+Y*S2+Z1)/W;
            C3 = (X*S2-Y*C2+Z2)/W;

            q3 = atan2(S3,C3);

            if q3 < q_posLim(3,1)-thetaPlus_i(3)
                q3 = q_posLim(3,1)-thetaPlus_i(3);
            elseif q3 > q_posLim(3,2)-thetaPlus_i(3)
                q3 = q_posLim(3,2)-thetaPlus_i(3);
            end

            %% b) Computation of theta4, theta5, theta6
            R3_0 = (robotics.math.getRi_j(alpha_i(1),thetaPlus_i(1)+q1)*robotics.math.getRi_j(alpha_i(2),thetaPlus_i(2)+q2)*robotics.math.getRi_j(alpha_i(3),thetaPlus_i(3)+q3))';

            FGH = R3_0*RGoal;

            F = FGH(:,1);
            G = FGH(:,2);
            H = FGH(:,3);

            % theta 4
            q4a = atan2(H(3),H(1));

            if config == 1
                if (abs(q_ref(4) - (q4a - 2*pi)) < abs(q_ref(4) - q4a))
                    q4 = q4a-2*pi;
                else
                    q4 = q4a+pi;
                end
            else
                if (abs(q_ref(4) - (q4a - pi)) < abs(q_ref(4) - q4a))
                    q4 = q4a - pi;
                elseif (abs(q_ref(4) - (q4a + pi)) < abs(q_ref(4) - q4a))
                    q4 = q4a + pi;
                elseif (abs(q_ref(4) - (q4a - pi / 2)) < abs(q_ref(4) - q4a))
                    q4 = q4a - pi / 2;
                elseif (abs(q_ref(4) - (q4a + pi / 2)) < abs(q_ref(4) - q4a))
                    q4 = q4a + pi / 2;
                elseif (abs(q_ref(4) - (q4a - 2 * pi)) < abs(q_ref(4) - q4a))
                    q4 = q4a - 2 * pi;
                elseif (abs(q_ref(4) - (q4a + 2 * pi)) < abs(q_ref(4) - q4a))
                    q4 = q4a + 2 * pi;
                else
                    q4 = q4a;
                end
            end

            if q4 < q_posLim(4,1)-thetaPlus_i(4)
                q4 = q_posLim(4,1)-thetaPlus_i(4);
            elseif q4 > q_posLim(4,2)-thetaPlus_i(4)
                q4 = q_posLim(4,2)-thetaPlus_i(4);
            end

            % theta 5
            S5 = sin(q4)*H(3)+cos(q4)*H(1);
            C5 = -H(2);

            q5 = atan2(S5,C5);

            if q5 < q_posLim(5,1)-thetaPlus_i(5)
                q5 = q_posLim(5,1)-thetaPlus_i(5);
            elseif q5 > q_posLim(5,2)-thetaPlus_i(5)
                q5 = q_posLim(5,2)-thetaPlus_i(5);
            end

            % theta 6
            S6 = cos(q4)*F(3)-sin(q4)*F(1);
            C6 = cos(q4)*G(3)-sin(q4)*G(1);

            q6a = atan2(S6,C6);
            if (abs(q_ref(6) - (q6a - 2*pi)) < abs(q_ref(6) - q6a))
                q6 = q6a - 2*pi;
            else
                q6 = atan2(S6,C6);
            end

            if q6 < q_posLim(6,1)-thetaPlus_i(6)
                q6 = q_posLim(6,1)-thetaPlus_i(6);
            elseif q6 > q_posLim(6,2)-thetaPlus_i(6)
                q6 = q_posLim(6,2)-thetaPlus_i(6);
            end

            % Result
            q_i = [q1;q2;q3;q4;q5;q6]+thetaPlus_i(1:6);
        end
    end
end





