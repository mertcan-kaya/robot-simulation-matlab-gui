function tau = springModel(q)

    r = 0.08;
    L = 0.728;

    Pc  = 5314;
    a0  = deg2rad(-0.8458);
    k   = 23950;
        
    x0 = sqrt(r^2+(r+L)^2-2*r*(r+L)*cos(a0+q(2)))-L;

    B1E = r*sin(a0+q(2));
    A2G = ((L+r)*B1E)./(L+x0);

    % F = k*x+Pc;

    F1 = k*x0;
    F2 = Pc*ones(1,length(x0));

    tau1 = F1.*A2G;
    tau2 = F2.*A2G;

    tau = zeros(6,1);
    tau(2) = tau1 + tau2;

end