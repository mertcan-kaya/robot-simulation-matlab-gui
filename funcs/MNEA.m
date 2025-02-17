function tau_j = MNEA(kin,q,qd,qRd,qRdd,g,pj_j)

    n = kin.n;
    np = kin.np;
    zj_j = kin.zj_j;
    Rj_i = zeros(3,3,n+1);
    
    % output declaration
    tau_j = zeros(n,1);
    
    % forward recursion (link 1 to n)
    wi_i = zeros(3,1);
    wRi_i = zeros(3,1);
    Ui_i = zeros(3,3);
    wdi_i = zeros(3,1);
    Psifi_i = zeros(3,np,n+1);
    Psini_i = zeros(3,np,n+1);
    mui_i = -g;
    for j = 1:n
        % take transpoze one time for each step
        Rj_i(:,:,j) = getRi_j(kin.alpha_j(j),kin.theta_O_j(j)+q(j))';
        
        wdi_i = Rj_i(:,:,j)*wdi_i + (cross(Rj_i(:,:,j)*wRi_i,qd(j)*zj_j) + cross(Rj_i(:,:,j)*wi_i,qRd(j)*zj_j))/2 + qRdd(j)*zj_j;
        mui_i = Rj_i(:,:,j)*(mui_i + Ui_i*kin.ri_j(:,:,j));

        % calculate normal and reference ang vel
        wi_i = Rj_i(:,:,j)*wi_i + qd(j)*zj_j;
        wRi_i = Rj_i(:,:,j)*wRi_i + qRd(j)*zj_j;

        Ui_i = SkewSym(wdi_i) + (SkewSym(wi_i)*SkewSym(wRi_i) + SkewSym(wRi_i)*SkewSym(wi_i))/2;
        Oh_h = DotMat(wdi_i) + (SkewSym(wRi_i)*DotMat(wi_i) + SkewSym(wi_i)*DotMat(wRi_i))/2;

        Psifi_i(:,:,j+1) = [zeros(3,6),Ui_i,mui_i];
        Psini_i(:,:,j+1) = [Oh_h,-SkewSym(mui_i),zeros(3,1)];
    end
    
    % backward recursion recursion (link n to 1)
    fi_i = zeros(3,1);
    ni_i = zeros(3,1);
    for j = n:-1:1
        ni_i = Psini_i(:,:,j+1)*pj_j(:,:,j) + Rj_i(:,:,j+1)'*(SkewSym(Rj_i(:,:,j+1)*kin.ri_j(:,:,j+1))*fi_i + ni_i);
        fi_i = Psifi_i(:,:,j+1)*pj_j(:,:,j) + Rj_i(:,:,j+1)'*fi_i;
        tau_j(j) = zj_j'*ni_i;
    end

end