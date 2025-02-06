function tau_gj = getTauG(kin,q,g,m_j,dj_j)

    n = kin.n;
    Rj_i = zeros(3,3,n+1);
    ri_j = zeros(3,1,n+1);
    
    % output declaration
    tau_gj = zeros(n,1);
    
    % forward recursion (link 1 to n)
    gi_i = zeros(3,1,n+1);
    gi_i(:,:,1) = -g;
    for j = 1:n
        % take transpoze one time for each step
        Rj_i(:,:,j) = getRi_j(kin.alpha_j(j),kin.theta_O_j(j)+q(j))';
        gi_i(:,:,j+1) = Rj_i(:,:,j)*gi_i(:,:,j);

        ri_j(:,:,j) = getri_j_vec(kin.alpha_j(j),kin.a_j(j),kin.d_j(j));
    end
    
    % backward recursion recursion (link n to 1)
    fi_fi = zeros(3,1);
    ni_fi = zeros(3,1);
    for j = n:-1:1
        ni_fi = -SkewSym(gi_i(:,:,j+1))*dj_j(:,:,j) + Rj_i(:,:,j+1)'*(SkewSym(Rj_i(:,:,j+1)*ri_j(:,:,j+1))*fi_fi+ni_fi);
        fi_fi = gi_i(:,:,j+1)*m_j(j) + Rj_i(:,:,j+1)'*fi_fi;
        tau_gj(j) = kin.zj_j'*ni_fi;
    end
end