function tau_frc = frictionFERModelUni(qd)
% FRICTIONFERMODELUNI Evaluates universal FER nonlinear friction model for N-DoF robots.
    persistent phi1 phi2 phi3 c0
    if isempty(phi1)
        phi1 = [5.4615e-01; 0.87224; 6.4068e-01; 1.2794e+00; 8.3904e-01; 3.0301e-01; 5.6489e-01];
        phi2 = [5.1181e+00; 9.0657e+00; 1.0136e+01; 5.5903e+00; 8.3469e+00; 1.7133e+01; 1.0336e+01];
        phi3 = [3.9533e-02; 2.5882e-02; -4.6070e-02; 3.6194e-02; 2.6226e-02; -2.1047e-02; 3.5526e-03];
        c0 = phi1 ./ (1 + exp(-phi2 .* phi3));
    end
    
    n = length(qd);
    if n <= 7
        tau_frc = phi1(1:n) ./ (1 + exp(-phi2(1:n) .* (qd(:) + phi3(1:n)))) - c0(1:n);
    else
        p1 = repmat(phi1, ceil(n/7), 1); p1 = p1(1:n);
        p2 = repmat(phi2, ceil(n/7), 1); p2 = p2(1:n);
        p3 = repmat(phi3, ceil(n/7), 1); p3 = p3(1:n);
        c = repmat(c0, ceil(n/7), 1); c = c(1:n);
        tau_frc = p1 ./ (1 + exp(-p2 .* (qd(:) + p3))) - c;
    end
end