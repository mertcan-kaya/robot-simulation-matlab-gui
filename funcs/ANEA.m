function [tau_fj,phatj_bar] = ANEA(ctr,kin,q,qd,qRd,qRdd,g,p,Pdiag)

    n = kin.n;
    np = kin.np;
    zj_j = kin.zj_j;
    Rj_i = zeros(3,3,n+1);
    
    % output declaration
    tau_fj = zeros(n,1);
    phatj_bar = zeros(n*np,1);
    
    % forward recursion (link 1 to n)
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
        % take transpose one time for each step
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

        % calculate task space sliding variables
        svi_i = Rj_i(:,:,j)*(svi_i + cross(swi_i,kin.ri_j(:,:,j)));
        swi_i = wRi_i - wi_i;
        
        sigmai_i = Psifi_i(:,:,j+1)'*svi_i + Psini_i(:,:,j+1)'*swi_i;
        Pi = Pdiag((j-1)*np+1:j*np);
        phati_iprev = p((j-1)*np+1:j*np,1);
        phatj_j(:,:,j) = phati_iprev + Pi.*sigmai_i*ctr.tcyc;
        phatj_bar((j-1)*np+1:j*np) = phatj_j(:,:,j);
    end
    
    % backward recursion recursion (link n to 1)
    fi_fi = zeros(3,1);
    ni_fi = zeros(3,1);
    for j = n:-1:1
        ni_fi = Psini_i(:,:,j+1)*phatj_j(:,:,j) + Rj_i(:,:,j+1)'*(SkewSym(Rj_i(:,:,j+1)*kin.ri_j(:,:,j+1))*fi_fi+ni_fi);
        fi_fi = Psifi_i(:,:,j+1)*phatj_j(:,:,j) + Rj_i(:,:,j+1)'*fi_fi;
        tau_fj(j) = zj_j'*ni_fi;
    end
end