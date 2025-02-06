function tau_frc = frictionViscousModel(coupling,qd)

    tau_frc = zeros(6,1);

    tau_frc(1) = 133.7*qd(1);
    tau_frc(2) = 121.8*qd(2);
    tau_frc(3) = 53.69*qd(3);
    tau_frc(4) = 14.38*qd(4);
    if coupling == 1
        % Coupling
        tau_frc_a = 18.39*qd(5);
        tau_frc_b = 0.4887*qd(6);
        tau_frc_c = 7.44*(qd(5)+qd(6));

        tau_frc(5) = tau_frc_a + tau_frc_c;
        tau_frc(6) = tau_frc_b + tau_frc_c;
    else
        tau_frc(5) = 18.39*qd(5);
        tau_frc(6) = 14.71*qd(6);
    end

end