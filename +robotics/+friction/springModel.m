function tau = springModel(q)
% SPRINGMODEL Evaluates nonlinear spring compensation torque for Staubli robots.
    r = 0.08;
    L = 0.728;
    Pc = 5314;
    a0 = deg2rad(-0.8458);
    k = 23950;
        
    q2 = q(2);
    x0 = sqrt(r^2 + (r+L)^2 - 2*r*(r+L)*cos(a0 + q2)) - L;
    B1E = r * sin(a0 + q2);
    A2G = ((L + r) * B1E) / (L + x0);

    tau = zeros(6, 1);
    tau(2) = (k * x0 + Pc) * A2G;
end