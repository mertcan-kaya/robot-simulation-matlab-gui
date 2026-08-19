function E_RDot = getE_RDotMatrix(r_posEA, r_velEA, eulerSet)
% GETE_RDOTMATRIX Computes the convective derivative d/dt(E(R)) of the Euler mapping matrix.
%   E_RDot = GETE_RDOTMATRIX(R_POSEA, R_VELEA, EULERSET) computes the time-derivative
%   matrix for calculating Cartesian angular acceleration:
%       alphadot = E_R * phiddot + E_RDot * phidot
%
%   Supported Euler Angle Sets:
%       1: ZYZ
%       2: ZYX (Roll-Pitch-Yaw)
%       3: XYZ
%       4: ZXZ

    phi = r_posEA(1);
    tht = r_posEA(2);

    phidot = r_velEA(1);
    thtdot = r_velEA(2);

    E_RDot = zeros(3,3);
    switch eulerSet
        case 1 % ZYZ
            E_RDot = [ 0.0, -phidot*cos(phi),  thtdot*cos(phi)*cos(tht) - phidot*sin(phi)*sin(tht);
                       0.0, -phidot*sin(phi),  thtdot*cos(tht)*sin(phi) + phidot*cos(phi)*sin(tht);
                       0.0,  0.0,             -thtdot*sin(tht) ];
        case 2 % ZYX
            E_RDot = [ 0.0, -phidot*cos(phi), -phidot*cos(tht)*sin(phi) - thtdot*cos(phi)*sin(tht);
                       0.0, -phidot*sin(phi),  phidot*cos(phi)*cos(tht) - thtdot*sin(phi)*sin(tht);
                       0.0,  0.0,             -thtdot*cos(tht) ];
        case 3 % XYZ
            E_RDot = [ 0.0,  0.0,              thtdot*cos(tht);
                       0.0, -phidot*sin(phi), -phidot*cos(phi)*cos(tht) + thtdot*sin(phi)*sin(tht);
                       0.0,  phidot*cos(phi), -phidot*cos(tht)*sin(phi) - thtdot*cos(phi)*sin(tht) ];
        otherwise % ZXZ (case 4)
            E_RDot = [ 0.0, -phidot*sin(phi),  thtdot*cos(tht)*sin(phi) + phidot*cos(phi)*sin(tht);
                       0.0,  phidot*cos(phi), -thtdot*cos(phi)*cos(tht) + phidot*sin(phi)*sin(tht);
                       0.0,  0.0,             -thtdot*sin(tht) ];
    end
end
