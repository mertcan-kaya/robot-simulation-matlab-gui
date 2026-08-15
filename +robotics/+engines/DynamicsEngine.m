classdef DynamicsEngine
    % DYNAMICSENGINE Encapsulates algorithms for forward and inverse dynamics.
    
    methods (Static)
        function q_acc = forwardDynamics(robot, kin, dyn, tau, q_pos, q_vel)
            % Migrated from robotAlgo.m (Recursive Newton-Euler for forward dynamics)
            n = kin.n;
            Phij_j = [zeros(3,1); kin.zj_j];
        
            if isfield(dyn, 'spring_on') && dyn.spring_on == 1
                tau_spr = robot.getSpringTorque(q_pos);
            else
                tau_spr = zeros(n, 1);
            end
        
            if isfield(dyn, 'friction_on') && dyn.friction_on == 1
                tau_frc = robot.getFrictionTorque(q_vel);
            else
                tau_frc = zeros(n, 1);
            end
        
            tau_j = tau - (tau_spr + tau_frc);
        
            % i) First forward recursive computations for i = 1, ..., n
            Rj_i        = zeros(3,3,n);
            wi_i        = zeros(3,1);
            gammaj_j    = zeros(6,1,n);
            betai_i     = zeros(6,1,n+1);
            Gammai_i    = zeros(6,6,n+1);
            for j = 1:n
                Rj_i(:,:,j)         = robotics.math.getRi_j(kin.alpha_j(j), kin.theta_O_j(j) + q_pos(j))';
                gammaj_j(:,:,j)     = [Rj_i(:,:,j)*(cross(wi_i, cross(wi_i, robotics.math.getri_j_vec(kin.alpha_j(j), kin.a_j(j), kin.d_j(j)))));
                                        cross(Rj_i(:,:,j)*wi_i, q_vel(j)*kin.zj_j)];
                wi_i                = Rj_i(:,:,j)*wi_i + q_vel(j)*kin.zj_j;
                betai_i(:,:,j+1)    = -[cross(wi_i, cross(wi_i, dyn.dj_j(:,:,j)));
                                        cross(wi_i, dyn.Ij_j(:,:,j)*wi_i)];
                Gammai_i(:,:,j+1)   = [dyn.m_j(j)*eye(3), -robotics.math.SkewSym(dyn.dj_j(:,:,j)); robotics.math.SkewSym(dyn.dj_j(:,:,j)), dyn.Ij_j(:,:,j)];
            end
        
            % ii) Backward recursive computations for i = n, ..., 1
            H_j                 = zeros(n,1);
            Xj_i                = zeros(6,6,n);
            betaSi_i            = zeros(6,1,n+1);
            betaSi_i(:,:,n+1)   = betai_i(:,:,n+1);
            GammaSi_i           = zeros(6,6,n+1);
            GammaSi_i(:,:,n+1)  = Gammai_i(:,:,n+1);
            for j = n:-1:1
                H_j(j)              = Phij_j'*GammaSi_i(:,:,j+1)*Phij_j;
                KKi_i               = GammaSi_i(:,:,j+1) - GammaSi_i(:,:,j+1)*Phij_j*(1/H_j(j))*Phij_j'*GammaSi_i(:,:,j+1);
                alphaj_j            = KKi_i*gammaj_j(:,:,j) + GammaSi_i(:,:,j+1)*Phij_j*(1/H_j(j)) ...
                                    *(tau_j(j) + Phij_j'*betaSi_i(:,:,j+1)) - betaSi_i(:,:,j+1);
                Xj_i(:,:,j)         = robotics.math.SO3R3_R66_twist(Rj_i(:,:,j), robotics.math.getrj_i_vec(kin.theta_O_j(j) + q_pos(j), kin.a_j(j), kin.d_j(j)));
                betaSi_i(:,:,j)     = betai_i(:,:,j) - Xj_i(:,:,j)'*alphaj_j;
                GammaSi_i(:,:,j)    = Gammai_i(:,:,j) + Xj_i(:,:,j)'*KKi_i*Xj_i(:,:,j);
            end
        
            % iii) Second forward recursive computations for i = 1, ..., n
            q_acc = zeros(n, 1);
            if ~isfield(kin, 'g0')
                kin.g0 = -[0;0;9.81];
            end
            aai_i = [-kin.g0; zeros(3,1)];
            for j = 1:n
                aaj_i       = Xj_i(:,:,j)*aai_i;
                q_acc(j,1)  = (1/H_j(j))*(-Phij_j'*GammaSi_i(:,:,j+1)*(aaj_i ...
                            + gammaj_j(:,:,j)) + tau_j(j) + Phij_j'*betaSi_i(:,:,j+1));
                aai_i       = aaj_i + Phij_j*q_acc(j) + gammaj_j(:,:,j);
            end
        end
        
        function tau_j = inverseDynamicsMNEA(kin, q, qd, qRd, qRdd, g, pj_j)
            % Migrated from MNEA.m
            n = kin.n;
            np = kin.np;
            zj_j = kin.zj_j;
            Rj_i = zeros(3,3,n+1);
            
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
                mui_i = Rj_i(:,:,j)*(mui_i + Ui_i*kin.ri_j(:,:,j));
        
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
                ni_i = Psini_i(:,:,j+1)*pj_j(:,:,j) + Rj_i(:,:,j+1)'*(robotics.math.SkewSym(Rj_i(:,:,j+1)*kin.ri_j(:,:,j+1))*fi_i + ni_i);
                fi_i = Psifi_i(:,:,j+1)*pj_j(:,:,j) + Rj_i(:,:,j+1)'*fi_i;
                tau_j(j) = zj_j'*ni_i;
            end
        end

        function [tau_fj, phatj_bar] = inverseDynamicsANEA(ctr, kin, q, qd, qRd, qRdd, g, p, Pdiag)
            % Migrated from ANEA.m
            n = kin.n;
            np = kin.np;
            zj_j = kin.zj_j;
            Rj_i = zeros(3,3,n+1);
            
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
                mui_i = Rj_i(:,:,j)*(mui_i + Ui_i*kin.ri_j(:,:,j));
        
                wi_i = Rj_i(:,:,j)*wi_i + qd(j)*zj_j;
                wRi_i = Rj_i(:,:,j)*wRi_i + qRd(j)*zj_j;
        
                Ui_i = robotics.math.SkewSym(wdi_i) + (robotics.math.SkewSym(wi_i)*robotics.math.SkewSym(wRi_i) + robotics.math.SkewSym(wRi_i)*robotics.math.SkewSym(wi_i))/2;
                Oh_h = robotics.math.DotMat(wdi_i) + (robotics.math.SkewSym(wRi_i)*robotics.math.DotMat(wi_i) + robotics.math.SkewSym(wi_i)*robotics.math.DotMat(wRi_i))/2;
        
                Psifi_i(:,:,j+1) = [zeros(3,6), Ui_i, mui_i];
                Psini_i(:,:,j+1) = [Oh_h, -robotics.math.SkewSym(mui_i), zeros(3,1)];
        
                svi_i = Rj_i(:,:,j)*(svi_i + cross(swi_i, kin.ri_j(:,:,j)));
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
                ni_fi = Psini_i(:,:,j+1)*phatj_j(:,:,j) + Rj_i(:,:,j+1)'*(robotics.math.SkewSym(Rj_i(:,:,j+1)*kin.ri_j(:,:,j+1))*fi_fi + ni_fi);
                fi_fi = Psifi_i(:,:,j+1)*phatj_j(:,:,j) + Rj_i(:,:,j+1)'*fi_fi;
                tau_fj(j) = zj_j'*ni_fi;
            end
        end
        
        function tau_gj = getTauG(kin, q, g, m_j, dj_j)
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
            
            % backward recursion recursion (link n to 1)
            fi_fi = zeros(3,1);
            ni_fi = zeros(3,1);
            for j = n:-1:1
                ni_fi = -robotics.math.SkewSym(gi_i(:,:,j+1))*dj_j(:,:,j) + Rj_i(:,:,j+1)'*(robotics.math.SkewSym(Rj_i(:,:,j+1)*ri_j(:,:,j+1))*fi_fi+ni_fi);
                fi_fi = gi_i(:,:,j+1)*m_j(j) + Rj_i(:,:,j+1)'*fi_fi;
                tau_gj(j) = kin.zj_j'*ni_fi;
            end
        end
    end
end
