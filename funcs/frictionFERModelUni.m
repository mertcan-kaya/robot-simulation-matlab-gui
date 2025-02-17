function tau_frc = frictionFERModelUni(qd)

    n = length(qd);

    phi1 = [5.4615e-01;0.87224;6.4068e-01;1.2794e+00;8.3904e-01;3.0301e-01;5.6489e-01];
    phi2 = [5.1181e+00;9.0657e+00;1.0136e+01;5.5903e+00;8.3469e+00;1.7133e+01;1.0336e+01];
    phi3 = [3.9533e-02;2.5882e-02;-4.6070e-02;3.6194e-02;2.6226e-02;-2.1047e-02;3.5526e-03];

    tau_frc = zeros(n,1);
    for j = 1:n
        tau_frc(j,1) = phi1(j)/(1+exp(-phi2(j)*(qd(j)+phi3(j))))-phi1(j)/(1+exp(-phi2(j)*phi3(j)));
    end

end