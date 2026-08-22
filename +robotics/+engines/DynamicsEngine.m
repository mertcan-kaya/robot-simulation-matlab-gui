classdef DynamicsEngine
    % DYNAMICSENGINE Static algorithms for forward and inverse robot dynamics.
    %   Encapsulates recursive dynamic solvers including Recursive Newton-Euler (RNEA),
    %   Articulated-Body Forward Dynamics (ABA/RNEA), Modified Newton-Euler (MNEA),
    %   Adaptive Newton-Euler (ANEA), and Analytical Gravity Compensation.
    
    methods (Static)
        function ws = createDynamicsWorkspace(n)
            % CREATEDYNAMICSWORKSPACE Allocates reusable workspace buffers for forwardDynamics.
            %   ws = CREATEDYNAMICSWORKSPACE(N) allocates fixed memory buffers for an N-DoF
            %   serial manipulator to eliminate heap allocation during high-frequency integration.
            
            ws.n = n;
            ws.Rj_i = zeros(3,3,n);
            ws.gammaj_j = zeros(6,1,n);
            ws.betai_i = zeros(6,1,n+1);
            ws.Gammai_i = zeros(6,6,n+1);
            ws.H_j = zeros(n,1);
            ws.Xj_i = zeros(6,6,n);
            ws.betaSi_i = zeros(6,1,n+1);
            ws.GammaSi_i = zeros(6,6,n+1);
            ws.q_acc = zeros(n, 1);
            ws.tau_spr_zero = zeros(n, 1);
            ws.tau_frc_zero = zeros(n, 1);
            ws.wi_i_zero = zeros(3, 1);
            ws.ri_j = [];
        end

        function q_acc = forwardDynamics(robot, kin, dyn, tau, q_pos, q_vel, ws)
            % FORWARDDYNAMICS Computes joint accelerations from applied joint torques.
            %   q_acc = FORWARDDYNAMICS(ROBOT, KIN, DYN, TAU, Q_POS, Q_VEL, WS) solves the
            %   forward dynamics equation M(q)*q_acc + C(q,q_vel)*q_vel + G(q) = tau
            %   using the recursive articulated forward dynamics formulation (Featherstone / RNEA).
            %
            %   Inputs:
            %       robot - RobotModel instance (for spring and friction evaluation)
            %       kin   - struct: Kinematic parameters (DH tables, axes, gravity)
            %       dyn   - struct: Inertial parameters (masses m_j, CoM d_j, inertias I_j)
            %       tau   - (Nx1) double: Actuator joint torques [N*m]
            %       q_pos - (Nx1) double: Current joint positions [rad or m]
            %       q_vel - (Nx1) double: Current joint velocities [rad/s or m/s]
            %       ws    - (optional) struct: Reusable workspace buffers from createDynamicsWorkspace
            %
            %   Outputs:
            %       q_acc - (Nx1) double: Resulting joint accelerations [rad/s^2 or m/s^2]
            %
            %   See also createDynamicsWorkspace, inverseDynamicsMNEA, getTauG.
            
            n = kin.n;
            persistent cached_ws
            if nargin < 7 || isempty(ws)
                if isempty(cached_ws) || cached_ws.n ~= n
                    cached_ws = robotics.engines.DynamicsEngine.createDynamicsWorkspace(n);
                end
                ws = cached_ws;
            end
            
            Phij_j = [zeros(3,1); kin.zj_j];
        
            if isfield(dyn, 'spring_on') && dyn.spring_on == 1
                tau_spr = robot.getSpringTorque(q_pos);
            else
                tau_spr = ws.tau_spr_zero;
            end
        
            if isfield(dyn, 'friction_on') && dyn.friction_on == 1
                tau_frc = robot.getFrictionTorque(q_vel);
            else
                tau_frc = ws.tau_frc_zero;
            end
        
            tau_j = tau - (tau_spr + tau_frc);
            
            % Check if ri_j is pre-cached
            if isfield(kin, 'ri_j') && ~isempty(kin.ri_j)
                ri_j = kin.ri_j;
            else
                if isempty(ws.ri_j) || size(ws.ri_j, 3) < n
                    ws.ri_j = zeros(3, 1, n);
                    for j = 1:n
                        ws.ri_j(:,:,j) = robotics.math.getri_j_vec(kin.alpha_j(j), kin.a_j(j), kin.d_j(j));
                    end
                end
                ri_j = ws.ri_j;
            end
        
            % i) First forward recursive computations for i = 1, ..., n
            wi_i = ws.wi_i_zero;
            for j = 1:n
                th = kin.theta_O_j(j) + q_pos(j);
                ca = cos(kin.alpha_j(j)); sa = sin(kin.alpha_j(j));
                ct = cos(th);             st = sin(th);
                
                % Direct SO(3) transpose Rj_i = (Rot_x(alpha) * Rot_z(theta))'
                R_curr = [ ct,       st*ca,    st*sa;
                          -st,       ct*ca,    ct*sa;
                            0,      -sa,       ca   ];
                ws.Rj_i(:,:,j) = R_curr;
                
                % Fast inlined analytical cross products
                rij = ri_j(:,:,j);
                w_x_rij = [wi_i(2)*rij(3) - wi_i(3)*rij(2);
                           wi_i(3)*rij(1) - wi_i(1)*rij(3);
                           wi_i(1)*rij(2) - wi_i(2)*rij(1)];
                w_x_w_x_rij = [wi_i(2)*w_x_rij(3) - wi_i(3)*w_x_rij(2);
                               wi_i(3)*w_x_rij(1) - wi_i(1)*w_x_rij(3);
                               wi_i(1)*w_x_rij(2) - wi_i(2)*w_x_rij(1)];
                
                Rwi = R_curr * wi_i;
                qvz = q_vel(j) * kin.zj_j;
                Rwi_x_qvz = [Rwi(2)*qvz(3) - Rwi(3)*qvz(2);
                             Rwi(3)*qvz(1) - Rwi(1)*qvz(3);
                             Rwi(1)*qvz(2) - Rwi(2)*qvz(1)];
                
                ws.gammaj_j(:,:,j) = [R_curr * w_x_w_x_rij; Rwi_x_qvz];
                wi_i = Rwi + qvz;
                
                dj = dyn.dj_j(:,:,j);
                w_x_dj = [wi_i(2)*dj(3) - wi_i(3)*dj(2);
                          wi_i(3)*dj(1) - wi_i(1)*dj(3);
                          wi_i(1)*dj(2) - wi_i(2)*dj(1)];
                w_x_w_x_dj = [wi_i(2)*w_x_dj(3) - wi_i(3)*w_x_dj(2);
                              wi_i(3)*w_x_dj(1) - wi_i(1)*w_x_dj(3);
                              wi_i(1)*w_x_dj(2) - wi_i(2)*w_x_dj(1)];
                
                Iw = dyn.Ij_j(:,:,j) * wi_i;
                w_x_Iw = [wi_i(2)*Iw(3) - wi_i(3)*Iw(2);
                          wi_i(3)*Iw(1) - wi_i(1)*Iw(3);
                          wi_i(1)*Iw(2) - wi_i(2)*Iw(1)];
                
                ws.betai_i(:,:,j+1) = -[w_x_w_x_dj; w_x_Iw];
                
                % Fast Gammai_i construction
                Sdj = [ 0, -dj(3), dj(2); dj(3), 0, -dj(1); -dj(2), dj(1), 0 ];
                ws.Gammai_i(:,:,j+1) = [dyn.m_j(j)*eye(3), -Sdj; Sdj, dyn.Ij_j(:,:,j)];
            end
        
            % ii) Backward recursive computations for i = n, ..., 1
            ws.betaSi_i(:,:,n+1)   = ws.betai_i(:,:,n+1);
            ws.GammaSi_i(:,:,n+1)  = ws.Gammai_i(:,:,n+1);
            for j = n:-1:1
                GammaS_j1 = ws.GammaSi_i(:,:,j+1);
                betaS_j1 = ws.betaSi_i(:,:,j+1);
                
                H_j_val = Phij_j'*GammaS_j1*Phij_j;
                ws.H_j(j) = H_j_val;
                invH = 1.0 / H_j_val;
                
                GammaS_Phi = GammaS_j1*Phij_j;
                KKi_i = GammaS_j1 - (GammaS_Phi * (invH * GammaS_Phi'));
                
                alphaj_j = KKi_i*ws.gammaj_j(:,:,j) + GammaS_Phi * (invH * (tau_j(j) + Phij_j'*betaS_j1)) - betaS_j1;
                
                th = kin.theta_O_j(j) + q_pos(j);
                rj_i = [-kin.a_j(j)*cos(th); kin.a_j(j)*sin(th); -kin.d_j(j)];
                ws.Xj_i(:,:,j) = robotics.math.SO3R3_R66_twist(ws.Rj_i(:,:,j), rj_i);
                X = ws.Xj_i(:,:,j);
                
                ws.betaSi_i(:,:,j)     = ws.betai_i(:,:,j) - X'*alphaj_j;
                ws.GammaSi_i(:,:,j)    = ws.Gammai_i(:,:,j) + X'*KKi_i*X;
            end
        
            % iii) Second forward recursive computations for i = 1, ..., n
            q_acc = ws.q_acc;
            if ~isfield(kin, 'g0')
                kin.g0 = -[0;0;9.81];
            end
            aai_i = [-kin.g0; ws.wi_i_zero];
            for j = 1:n
                aaj_i       = ws.Xj_i(:,:,j)*aai_i;
                q_acc(j,1)  = (1/ws.H_j(j))*(-Phij_j'*ws.GammaSi_i(:,:,j+1)*(aaj_i ...
                            + ws.gammaj_j(:,:,j)) + tau_j(j) + Phij_j'*ws.betaSi_i(:,:,j+1));
                aai_i       = aaj_i + Phij_j*q_acc(j) + ws.gammaj_j(:,:,j);
            end
        end
        
        function tau_j = inverseDynamicsMNEA(kin, q, qd, qRd, qRdd, g, pj_j)
            % INVERSEDYNAMICSMNEA Computes inverse dynamics using Modified Newton-Euler (MNEA).
            %   tau_j = INVERSEDYNAMICSMNEA(KIN, Q, QD, QRD, QRDD, G, PJ_J) evaluates the
            %   inverse dynamics torques using linearly parameterized inertial parameters
            %   and reference trajectory acceleration terms.
            %
            %   Inputs:
            %       kin   - struct: Kinematics parameters (.n, .np, .zj_j, .alpha_j, etc.)
            %       q     - (Nx1) double: Joint positions [rad or m]
            %       qd    - (Nx1) double: Joint velocities [rad/s or m/s]
            %       qRd   - (Nx1) double: Reference joint velocities [rad/s or m/s]
            %       qRdd  - (Nx1) double: Reference joint accelerations [rad/s^2 or m/s^2]
            %       g     - (3x1) double: Gravity acceleration vector in base frame [m/s^2]
            %       pj_j  - (npx1xn) double: Base inertial parameters vector
            %
            %   Outputs:
            %       tau_j - (Nx1) double: Joint torque vector [N*m]
            %
            %   See also inverseDynamicsANEA, forwardDynamics.
            
            n = kin.n;
            np = kin.np;
            zj_j = kin.zj_j;
            Rj_i = zeros(3,3,n+1);
            
            if isfield(kin, 'ri_j') && ~isempty(kin.ri_j)
                ri_j = kin.ri_j;
            else
                ri_j = zeros(3,1,n+1);
                for j = 1:n+1
                    ri_j(:,:,j) = robotics.math.getri_j_vec(kin.alpha_j(j), kin.a_j(j), kin.d_j(j));
                end
            end
            
            tau_j = zeros(n,1);
            
            wi_i = zeros(3,1);
            wRi_i = zeros(3,1);
            Ui_i = zeros(3,3);
            wdi_i = zeros(3,1);
            Psifi_i = zeros(3,np,n+1);
            Psini_i = zeros(3,np,n+1);
            mui_i = -g;
            for j = 1:n
                Rj_i(:,:,j) = robotics.math.getRi_j(kin.alpha_j(j), kin.theta_O_j(j) + q(j))';
                
                wdi_i = Rj_i(:,:,j)*wdi_i + (cross(Rj_i(:,:,j)*wRi_i, qd(j)*zj_j) + cross(Rj_i(:,:,j)*wi_i, qRd(j)*zj_j))/2 + qRdd(j)*zj_j;
                mui_i = Rj_i(:,:,j)*(mui_i + Ui_i*ri_j(:,:,j));
        
                wi_i = Rj_i(:,:,j)*wi_i + qd(j)*zj_j;
                wRi_i = Rj_i(:,:,j)*wRi_i + qRd(j)*zj_j;
        
                Ui_i = robotics.math.SkewSym(wdi_i) + (robotics.math.SkewSym(wi_i)*robotics.math.SkewSym(wRi_i) + robotics.math.SkewSym(wRi_i)*robotics.math.SkewSym(wi_i))/2;
                Oh_h = robotics.math.DotMat(wdi_i) + (robotics.math.SkewSym(wRi_i)*robotics.math.DotMat(wi_i) + robotics.math.SkewSym(wi_i)*robotics.math.DotMat(wRi_i))/2;
        
                Psifi_i(:,:,j+1) = [zeros(3,6), Ui_i, mui_i];
                Psini_i(:,:,j+1) = [Oh_h, -robotics.math.SkewSym(mui_i), zeros(3,1)];
            end
            
            fi_i = zeros(3,1);
            ni_i = zeros(3,1);
            for j = n:-1:1
                ni_i = Psini_i(:,:,j+1)*pj_j(:,:,j) + Rj_i(:,:,j+1)'*(robotics.math.SkewSym(Rj_i(:,:,j+1)*ri_j(:,:,j+1))*fi_i + ni_i);
                fi_i = Psifi_i(:,:,j+1)*pj_j(:,:,j) + Rj_i(:,:,j+1)'*fi_i;
                tau_j(j) = zj_j'*ni_i;
            end
        end

        function [tau_fj, phatj_bar] = inverseDynamicsANEA(ctr, kin, q, qd, qRd, qRdd, g, p, Pdiag)
            % INVERSEDYNAMICSANEA Evaluates Adaptive Newton-Euler inverse dynamics with parameter estimation.
            %   [tau_fj, phatj_bar] = INVERSEDYNAMICSANEA(CTR, KIN, Q, QD, QRD, QRDD, G, P, PDIAG)
            %   updates the estimated inertial parameters phat online via composite adaptive
            %   laws and computes feedforward control torques tau_fj.
            %
            %   Inputs:
            %       ctr       - struct: Controller parameters (e.g. sample period .tcyc)
            %       kin       - struct: Kinematics parameters (.n, .np, .zj_j, etc.)
            %       q         - (Nx1) double: Joint positions [rad or m]
            %       qd        - (Nx1) double: Joint velocities [rad/s or m/s]
            %       qRd       - (Nx1) double: Reference joint velocities [rad/s or m/s]
            %       qRdd      - (Nx1) double: Reference joint accelerations [rad/s^2 or m/s^2]
            %       g         - (3x1) double: Gravity acceleration vector in base frame [m/s^2]
            %       p         - ((N*np)x1) double: Current estimated parameter vector
            %       Pdiag     - ((N*np)x1) double: Adaptation gain matrix diagonal
            %
            %   Outputs:
            %       tau_fj    - (Nx1) double: Adaptive feedforward control torque vector [N*m]
            %       phatj_bar - ((N*np)x1) double: Updated inertial parameter estimate vector
            
            n = kin.n;
            np = kin.np;
            zj_j = kin.zj_j;
            Rj_i = zeros(3,3,n+1);
            
            if isfield(kin, 'ri_j') && ~isempty(kin.ri_j)
                ri_j = kin.ri_j;
            else
                ri_j = zeros(3,1,n+1);
                for j = 1:n+1
                    ri_j(:,:,j) = robotics.math.getri_j_vec(kin.alpha_j(j), kin.a_j(j), kin.d_j(j));
                end
            end
            
            tau_fj = zeros(n,1);
            phatj_bar = zeros(n*np,1);
            
            wi_i = zeros(3,1);
            wRi_i = zeros(3,1);
            Ui_i = zeros(3,3);
            wdi_i = zeros(3,1);
            Psifi_i = zeros(3,np,n+1);
            Psini_i = zeros(3,np,n+1);
            swi_i = zeros(3,1);
            svi_i = zeros(3,1);
            phatj_j = zeros(np,1,n);
            mui_i = -g;
            for j = 1:n
                Rj_i(:,:,j) = robotics.math.getRi_j(kin.alpha_j(j), kin.theta_O_j(j) + q(j))';
                
                wdi_i = Rj_i(:,:,j)*wdi_i + (cross(Rj_i(:,:,j)*wRi_i, qd(j)*zj_j) + cross(Rj_i(:,:,j)*wi_i, qRd(j)*zj_j))/2 + qRdd(j)*zj_j;
                mui_i = Rj_i(:,:,j)*(mui_i + Ui_i*ri_j(:,:,j));
        
                wi_i = Rj_i(:,:,j)*wi_i + qd(j)*zj_j;
                wRi_i = Rj_i(:,:,j)*wRi_i + qRd(j)*zj_j;
        
                Ui_i = robotics.math.SkewSym(wdi_i) + (robotics.math.SkewSym(wi_i)*robotics.math.SkewSym(wRi_i) + robotics.math.SkewSym(wRi_i)*robotics.math.SkewSym(wi_i))/2;
                Oh_h = robotics.math.DotMat(wdi_i) + (robotics.math.SkewSym(wRi_i)*robotics.math.DotMat(wi_i) + robotics.math.SkewSym(wi_i)*robotics.math.DotMat(wRi_i))/2;
        
                Psifi_i(:,:,j+1) = [zeros(3,6), Ui_i, mui_i];
                Psini_i(:,:,j+1) = [Oh_h, -robotics.math.SkewSym(mui_i), zeros(3,1)];
        
                svi_i = Rj_i(:,:,j)*(svi_i + cross(swi_i, ri_j(:,:,j)));
                swi_i = wRi_i - wi_i;
                
                sigmai_i = Psifi_i(:,:,j+1)'*svi_i + Psini_i(:,:,j+1)'*swi_i;
                Pi = Pdiag((j-1)*np+1:j*np);
                phati_iprev = p((j-1)*np+1:j*np, 1);
                phatj_j(:,:,j) = phati_iprev + Pi.*sigmai_i*ctr.tcyc;
                phatj_bar((j-1)*np+1:j*np) = phatj_j(:,:,j);
            end
            
            fi_fi = zeros(3,1);
            ni_fi = zeros(3,1);
            for j = n:-1:1
                ni_fi = Psini_i(:,:,j+1)*phatj_j(:,:,j) + Rj_i(:,:,j+1)'*(robotics.math.SkewSym(Rj_i(:,:,j+1)*ri_j(:,:,j+1))*fi_fi + ni_fi);
                fi_fi = Psifi_i(:,:,j+1)*phatj_j(:,:,j) + Rj_i(:,:,j+1)'*fi_fi;
                tau_fj(j) = zj_j'*ni_fi;
            end
        end
        
        function tau_gj = getTauG(kin, q, g, m_j, dj_j)
            % GETTAUG Computes analytical gravity compensation torques.
            %   tau_gj = GETTAUG(KIN, Q, G, M_J, DJ_J) computes the gravity torque vector
            %   G(q) via backward link-by-link recursion for gravity cancellation.
            %
            %   Inputs:
            %       kin    - struct: Kinematics parameters (.n, .zj_j, .alpha_j, etc.)
            %       q      - (Nx1) double: Current joint position vector [rad or m]
            %       g      - (3x1) double: Gravity acceleration in base frame [m/s^2]
            %       m_j    - (Nx1) double: Link mass vector [kg]
            %       dj_j   - (3x1xN) double: Link Center-of-Mass (CoM) position vectors [m]
            %
            %   Outputs:
            %       tau_gj - (Nx1) double: Gravity compensation torque vector [N*m]
            
            n = kin.n;
            Rj_i = zeros(3,3,n+1);
            ri_j = zeros(3,1,n+1);
            
            % output declaration
            tau_gj = zeros(n,1);
            
            % forward recursion (link 1 to n)
            gi_i = zeros(3,1,n+1);
            gi_i(:,:,1) = -g;
            for j = 1:n
                % take transpose one time for each step
                Rj_i(:,:,j) = robotics.math.getRi_j(kin.alpha_j(j), kin.theta_O_j(j)+q(j))';
                gi_i(:,:,j+1) = Rj_i(:,:,j)*gi_i(:,:,j);

                ri_j(:,:,j) = robotics.math.getri_j_vec(kin.alpha_j(j), kin.a_j(j), kin.d_j(j));
            end
            
            % backward recursion (link n to 1)
            fi_fi = zeros(3,1);
            ni_fi = zeros(3,1);
            for j = n:-1:1
                R_next = Rj_i(:,:,j+1);
                ri_next = ri_j(:,:,j+1);
                R_ri = R_next * ri_next;
                
                % Fast inlined cross products
                cross_R_ri_fi = [R_ri(2)*fi_fi(3) - R_ri(3)*fi_fi(2);
                                 R_ri(3)*fi_fi(1) - R_ri(1)*fi_fi(3);
                                 R_ri(1)*fi_fi(2) - R_ri(2)*fi_fi(1)];
                
                g_curr = gi_i(:,:,j+1);
                dj_curr = dj_j(:,:,j);
                cross_dj_g = [dj_curr(2)*g_curr(3) - dj_curr(3)*g_curr(2);
                              dj_curr(3)*g_curr(1) - dj_curr(1)*g_curr(3);
                              dj_curr(1)*g_curr(2) - dj_curr(2)*g_curr(1)];
                
                ni_fi = cross_dj_g + R_next'*(cross_R_ri_fi + ni_fi);
                fi_fi = g_curr*m_j(j) + R_next'*fi_fi;
                tau_gj(j) = kin.zj_j'*ni_fi;
            end
        end

        function M = computeMassMatrix(kin, dyn, q)
            % COMPUTEMASSMATRIX Computes the generalized inertia/mass matrix M(q) via RNEA.
            %   M = COMPUTEMASSMATRIX(KIN, DYN, Q) returns the symmetric (NxN) mass matrix.
            n = kin.n;
            M = zeros(n, n);
            zero_v = zeros(n, 1);
            zero_g = zeros(3, 1);
            
            for j = 1:n
                unit_acc = zeros(n, 1);
                unit_acc(j) = 1.0;
                M(:, j) = robotics.engines.DynamicsEngine.inverseDynamicsMNEA(...
                    kin, q, zero_v, zero_v, unit_acc, zero_g, dyn.pj_j);
            end
            
            % Enforce numerical symmetry
            M = 0.5 * (M + M');
        end

        function Ek = computeKineticEnergy(kin, dyn, q, q_vel)
            % COMPUTEKINETICENERGY Evaluates instantaneous kinetic energy 0.5 * q_vel' * M(q) * q_vel.
            %   EK = COMPUTEKINETICENERGY(KIN, DYN, Q, Q_VEL)
            M = robotics.engines.DynamicsEngine.computeMassMatrix(kin, dyn, q);
            Ek = 0.5 * (q_vel(:)' * M * q_vel(:));
        end
    end
end
