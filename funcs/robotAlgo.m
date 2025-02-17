function q_acc = robotAlgo(robot_model,kin,dyn,tau,q_pos,q_vel)

    n = kin.n;
    
    Phij_j = [zeros(3,1);kin.zj_j];

    if dyn.spring_on == 1 && (robot_model == 4 || robot_model == 5)
        tau_spr = springModel(q_pos);
    else
        tau_spr = zeros(n,1);
    end

    if dyn.friction_on == 1
        if robot_model == 1
            tau_frc = frictionFERModel(q_vel);
        else
            tau_frc = frictionFERModelUni(q_vel);
        end
    else
        tau_frc = zeros(n,1);
    end

    tau_j = tau - (tau_spr + tau_frc);

    % i) First forward recursive computations for i = 1, ..., n
    Rj_i        = zeros(3,3,n);
    wi_i        = zeros(3,1);
    gammaj_j    = zeros(6,1,n);
    betai_i     = zeros(6,1,n+1);
    Gammai_i    = zeros(6,6,n+1);
    for j = 1:n
        Rj_i(:,:,j)         = getRi_j(kin.alpha_j(j),kin.theta_O_j(j)+q_pos(j))';
        gammaj_j(:,:,j)     = [Rj_i(:,:,j)*(cross(wi_i,cross(wi_i,getri_j_vec(kin.alpha_j(j),kin.a_j(j),kin.d_j(j)))))
                                cross(Rj_i(:,:,j)*wi_i,q_vel(j)*kin.zj_j)];
        wi_i                = Rj_i(:,:,j)*wi_i + q_vel(j)*kin.zj_j;
        betai_i(:,:,j+1)    = -[cross(wi_i,cross(wi_i,dyn.dj_j(:,:,j)));
                                cross(wi_i,dyn.Ij_j(:,:,j)*wi_i)];
        Gammai_i(:,:,j+1)   = [dyn.m_j(j)*eye(3),-SkewSym(dyn.dj_j(:,:,j));SkewSym(dyn.dj_j(:,:,j)),dyn.Ij_j(:,:,j)];
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
                            *(tau_j(j)+Phij_j'*betaSi_i(:,:,j+1)) - betaSi_i(:,:,j+1);
        Xj_i(:,:,j)         = SO3R3_R66_twist(Rj_i(:,:,j),getrj_i_vec(kin.theta_O_j(j)+q_pos(j),kin.a_j(j),kin.d_j(j)));
        betaSi_i(:,:,j)     = betai_i(:,:,j) - Xj_i(:,:,j)'*alphaj_j;
        GammaSi_i(:,:,j)    = Gammai_i(:,:,j) + Xj_i(:,:,j)'*KKi_i*Xj_i(:,:,j);
    end

    % iii) Second forward recursive computations for i = 1, ..., n
    q_acc = zeros(n,1);
    aai_i = [-kin.g0;zeros(3,1)];
    for j = 1:n
        aaj_i       = Xj_i(:,:,j)*aai_i;
        q_acc(j,1)  = (1/H_j(j))*(-Phij_j'*GammaSi_i(:,:,j+1)*(aaj_i ...
                    + gammaj_j(:,:,j)) + tau_j(j) + Phij_j'*betaSi_i(:,:,j+1));
        aai_i       = aaj_i + Phij_j*q_acc(j) + gammaj_j(:,:,j);
    end
    
end