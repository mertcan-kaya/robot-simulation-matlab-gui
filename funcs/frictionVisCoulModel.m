function tau_frc = frictionVisCoulModel(qd)

    tau_frc = zeros(6,1);

    tau_frc(1) = 111.2*qd(1) + 35.98*sign(qd(1)) + 0.9811;
    tau_frc(2) = 87.28*qd(2) + 55.28*sign(qd(2)) - 4.794;
    tau_frc(3) = 36.49*qd(3) + 27.51*sign(qd(3)) - 7.222;
    tau_frc(4) = 9.596*qd(4) + 7.647*sign(qd(4)) + 0.01566;

    % Coupling
    tau_frc_a = 15.42*qd(5) + 4.747*sign(qd(5)) + 0.7782;
    tau_frc_b = 0.1652*qd(6) + 0.5175*sign(qd(6)) + 0.007565;
    tau_frc_c = 6.564*(qd(5)+qd(6)) + 2.805*sign(qd(5)+qd(6)) - 0.2884;

    tau_frc(5) = tau_frc_a + tau_frc_c;
    tau_frc(6) = tau_frc_b + tau_frc_c;
        
end