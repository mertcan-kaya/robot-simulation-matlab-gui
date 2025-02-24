function qDes = invGeo(app,RGoal,tGoal,q0)

    if app.inv_geo_type == 0
        if app.robot_model == 4 || app.robot_model == 5
            qDes = invGeoAlgebraic(app,RGoal,tGoal,q0,0);
        elseif app.robot_model == 2
            qDes = invGeoAnaUR3(app,RGoal,tGoal,q0);
        end
    elseif  app.inv_geo_type == 1
        qDes = invGeoNumeric(app,RGoal,tGoal,q0);
    else
        err = 0;
        if app.robot_model == 4 || app.robot_model == 5
            [qDes,err] = invGeoAlgebraic(app,RGoal,tGoal,q0,0);
        elseif app.robot_model == 2
            [qDes,err] = invGeoAnaUR3(app,RGoal,tGoal,q0);
        end
        if err == 1 || (app.robot_model ~= 2 || app.robot_model ~= 4 || app.robot_model ~= 5)
            qDes = invGeoNumeric(app,RGoal,tGoal,q0);
        end
    end

    function [q_pos,err] = invGeoAnaUR3(app,RGoal,tGoal,qPrev)
    
        err = 0;
    
        a_i = app.kin.a_j;
        alpha_i = app.kin.alpha_j;
        d_i = app.kin.d_j;
        theta_i_O = app.kin.theta_O_j;
        TI_0 = app.TI_0;
        % q_posLim = app.kin.q_posLim;

        T0_E = TI_0\SO3R3_SE3(RGoal,tGoal);
        
        p15x = T0_E(1,4) - T0_E(1,3)*d_i(7);
        p15y = T0_E(2,4) - T0_E(2,3)*d_i(7);
    %     p15z = T0_E(3,4) - T0_E(3,3)*d_i(7) - d_i(1);
        
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
    %     alpha1
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
    
        % double c3 = (p13x*p13x + p13y*p13y - a2*a2 - a3*a3) / (2.0*a2*a3);
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
    
            Rot_x_h = Rot_x(alpha_i);
            Trn_x_h	= Trn_x(a_i);
            Rot_z_i = Rot_z(theta_i);
            Trn_z_i = Trn_z(d_i);
    
            Th_i = Rot_x_h * Trn_x_h * Rot_z_i * Trn_z_i;
    
            function output = Rot_x(input)
                output = [	1	0           0           0
                            0	cos(input)  -sin(input)	0
                            0	sin(input)	cos(input)	0
                            0	0           0           1 ];
            end
    
            function output = Rot_z(input)
                output = [  cos(input)	-sin(input)	0	0
                            sin(input)	 cos(input)	0	0
                            0            0          1	0
                            0            0          0	1 ];
            end
    
            function output = Trn_x(input)
                output = [  1 0 0 input
                            0 1 0 0
                            0 0 1 0
                            0 0 0 1     ];
            end
    
            function output = Trn_z(input)
                output = [  1 0 0 0
                            0 1 0 0
                            0 0 1 input
                            0 0 0 1     ];
            end
        end
    end

    function [q_i,err] = invGeoAlgebraic(app,RGoal,tGoal,q_ref,config)

        err = 0;

        a_i = app.kin.a_j;
        alpha_i = app.kin.alpha_j;
        d_i = app.kin.d_j;
        thetaPlus_i = app.kin.theta_O_j;
        pI_0 = app.TI_0(1:3,4);
        q_posLim = app.kin.q_posLim;

        Px = tGoal(1,1) - RGoal(1,3) * d_i(7) - pI_0(1);
        Py = tGoal(2,1) - RGoal(2,3) * d_i(7) - pI_0(2);
        Pz = tGoal(3,1) - RGoal(3,3) * d_i(7) - (pI_0(3) + d_i(1));

        % theta 1
    %         if robot_model ~= 1
            q1 = atan2(Py,Px);
    %         else
    %             % implement type 2
    %         end

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

    %     e = [-1,1];

    %     q2i = zeros(1,2);
    %     for i = 1:2
    %         C2 = (B2*B3-e(i)*B1*sqrt(B1^2+B2^2-B3^2))/(B1^2+B2^2);
    %         S2 = (B1*B3+e(i)*B2*sqrt(B1^2+B2^2-B3^2))/(B1^2+B2^2);
    % 
    %         if imag(C2) > 0 || imag(S2) > 0
    %             q_i = q_ref;
    %             err = 1
    %             return
    %         end
    %         q2i(i) = atan2(S2,C2);
    %     end
    % 
    % %     q2a = q2i(1);
    %     q2b = q2i(2);
    %     q2 = q2b;

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

        R3_0 = (getRi_j(alpha_i(1),thetaPlus_i(1)+q1)*getRi_j(alpha_i(2),thetaPlus_i(2)+q2)*getRi_j(alpha_i(3),thetaPlus_i(3)+q3))';

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

    function qDes = invGeoNumeric(app,RGoal,pGoal,q0)

        a_i = app.kin.a_j;
        alpha_i = app.kin.alpha_j;
        d_i = app.kin.d_j;
        thetaPlus_i = app.kin.theta_O_j;
        TI_0 = app.TI_0;
        q_posLim = app.kin.q_posLim;

        errNormPrev = 2;
        errNorm = 1;
        itr = 0;
        tol = 1e-5;

        if app.inv_geo_trn == 0
            kp = app.kp_inv;
            kr = app.kr_inv;
        else
            kp = app.kp_trn;
            kr = app.kr_trn;
        end
        k = [kp;kp;kp;kr;kr;kr];

        J = zeros(6,app.kin.n);

        qItr = q0;
        while abs(errNorm-errNormPrev) > tol
            
            if itr > 5000
                break;
            end

            TItr = getTransMatrix(TI_0,a_i,alpha_i,d_i,thetaPlus_i,app.kin.j_type,qItr);

            for i = 1:app.kin.n
                Jv = cross(TItr(1:3,3,i+1),(TItr(1:3,4,app.kin.n+2)-TItr(1:3,4,i+1)));
                Jw = TItr(1:3,3,i+1);
                J(:,i) = [Jv;Jw];
            end

            pItr = TItr(1:3,4,app.kin.n+2);
            pErr = pGoal-pItr;
            rErr = 0.5*(cross(TItr(1:3,1,app.kin.n+2),RGoal(1:3,1))+cross(TItr(1:3,2,app.kin.n+2),RGoal(1:3,2))+cross(TItr(1:3,3,app.kin.n+2),RGoal(1:3,3)));
            xErr = [pErr;rErr];
            errNormPrev = errNorm;
            errNorm = norm(xErr);
            
            if app.inv_geo_trn == 0
                qItr = qItr + J'*(k.*xErr);
            else
                qItr = qItr + J\(k.*xErr);
            end

            itr = itr + 1;
        end

        for i = 1:app.kin.n
            if qItr(i) < q_posLim(i,1)
                qItr(i) = q_posLim(i,1);
            elseif qItr(i) > q_posLim(i,2)
                qItr(i) = q_posLim(i,2);
            end
        end

        qDes = qItr;
        
    end

end