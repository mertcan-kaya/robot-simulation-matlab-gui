function E_Rinv = getE_RinvMatrix(r_posEA, eulerSet)
% GETE_RINVMATRIX Computes the inverse rotational mapping matrix E^(-1)(R) for Euler angles.
%   E_Rinv = GETE_RINVMATRIX(R_POSEA, EULERSET) computes the (3x3) mapping matrix such that
%   the Euler rates phidot = E_Rinv * omega, where omega is the angular velocity in R^3.
%
%   Supported Euler Angle Sets:
%       1: ZYZ
%       2: ZYX (Roll-Pitch-Yaw)
%       3: XYZ
%       4: ZXZ

    phi = r_posEA(1);
    tht = r_posEA(2);

    E_Rinv = zeros(3,3);
    switch eulerSet
        case 1 % ZYZ
            E_Rinv = [ -cos(tht)*cos(phi)/sin(tht), -cos(tht)*sin(phi)/sin(tht), 1.0;
                       -sin(phi),                    cos(phi),                   0.0;
                        cos(phi)/sin(tht),           sin(phi)/sin(tht),          0.0 ];
        case 2 % ZYX
            E_Rinv = [  cos(phi)*sin(tht)/cos(tht),  sin(tht)*sin(phi)/cos(tht), 1.0;
                       -sin(phi),                    cos(phi),                   0.0;
                        cos(phi)/cos(tht),           sin(phi)/cos(tht),          0.0 ];
        case 3 % XYZ
            E_Rinv = [  1.0,  sin(phi)*sin(tht)/cos(tht), -cos(phi)*sin(tht)/cos(tht);
                        0.0,  cos(phi),                    sin(phi);
                        0.0, -sin(phi)/cos(tht),           cos(phi)/cos(tht) ];
        otherwise % ZXZ (case 4)
            E_Rinv = [ -cos(tht)*sin(phi)/sin(tht),  cos(tht)*cos(phi)/sin(tht), 1.0;
                        cos(phi),                    sin(phi),                   0.0;
                        sin(phi)/sin(tht),          -cos(phi)/sin(tht),          0.0 ];
    end
end
