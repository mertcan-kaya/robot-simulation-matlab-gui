function tau_frc = frictionPWModel(qd)

    tau_frc = zeros(6,1);

    tau = zeros(7,1);
    vel = zeros(7,1);
    vel(1:6) = qd;
    vel(7) = qd(5)+qd(6);        

    v = [111.2  ;87.28  ;36.49  ;9.596  ;15.42  ;0.1652 ;6.564];    % Viscous
    c = [35.98  ;55.28  ;27.51  ;7.647  ;4.747  ;0.5175 ;2.805];    % Coulumb
    o = [0.9811 ;-4.794 ;-7.222 ;0.01566;0.7782 ;0.007565;-0.2884]; % Offset
    n = [461.2  ;688.0  ;383.8  ;85.91  ;55.11  ;5.265  ;37.50];     % Negative slop
    p = [480.8  ;592.1  ;239.4  ;86.22  ;70.67  ;5.416  ;31.73];     % Positive slop

    for i = 1:7
        if (vel(i) > -1e-1 && vel(i) <= 0)
            tau(i) = n(i)*vel(i);
        elseif  (vel(i) > 0 && vel(i) <= 1e-1)
            tau(i) = p(i)*vel(i);
        else
            tau(i) = v(i)*vel(i)+c(i)*sign(vel(i))+o(i);
        end
    end

    tau_frc(1) = tau(1);
    tau_frc(2) = tau(2);
    tau_frc(3) = tau(3);
    tau_frc(4) = tau(4);
    tau_frc(5) = tau(5) + tau(7);
    tau_frc(6) = tau(6) + tau(7);

end