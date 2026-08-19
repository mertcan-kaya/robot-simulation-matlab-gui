function E_R = getE_Rmatrix(r_posEA, eulerSet)
% GETE_RMATRIX Computes the rotational mapping matrix E(R) for Euler angles.
%   E_R = GETE_RMATRIX(R_POSEA, EULERSET) computes the (3x3) mapping matrix such that
%   the angular velocity omega = E_R * phidot, where phidot is the rate of change
%   of Euler angles.
%
%   Supported Euler Angle Sets:
%       1: ZYZ
%       2: ZYX (Roll-Pitch-Yaw)
%       3: XYZ
%       4: ZXZ

    phi = r_posEA(1);
    tht = r_posEA(2);

    E_R = zeros(3,3);
    switch eulerSet
        case 1 % ZYZ
            E_R = [ 0.0,       -sin(phi),  cos(phi)*sin(tht);
                    0.0,        cos(phi),  sin(phi)*sin(tht);
                    1.0,        0.0,       cos(tht) ];
        case 2 % ZYX
            E_R = [ 0.0,       -sin(phi),  cos(tht)*cos(phi);
                    0.0,        cos(phi),  cos(tht)*sin(phi);
                    1.0,        0.0,      -sin(tht) ];
        case 3 % XYZ
            E_R = [ 1.0,        0.0,       sin(tht);
                    0.0,        cos(phi), -cos(tht)*sin(phi);
                    0.0,        sin(phi),  cos(phi)*cos(tht) ];
        otherwise % ZXZ (case 4)
            E_R = [ 0.0,        cos(phi),  sin(phi)*sin(tht);
                    0.0,        sin(phi), -cos(phi)*sin(tht);
                    1.0,        0.0,       cos(tht) ];
    end
end
