function R_mat = getRotMatfromEA(EA_vec, EA_set)
% GETROTMATFROMEA Converts Euler / Cardan angle vector to 3x3 rotation matrix.
%   R_mat = GETROTMATFROMEA(EA_VEC, EA_SET) computes the SO(3) rotation matrix
%   given an angle triplet [phi, theta, psi] and a chosen sequence identifier:
%
%   Supported Sequences (EA_set):
%       1 - Z-Y-Z Euler angles (phi, theta, psi)
%       2 - Z-Y-X Cardan / Yaw-Pitch-Roll (phi, theta, psi)
%       3 - X-Y-Z Cardan angles (phi, theta, psi)
%       4 - Z-X-Z Euler angles (phi, theta, psi) [Default/Otherwise]
%
%   Inputs:
%       EA_vec - (3x1) or (1x3) double: [phi, theta, psi] in radians
%       EA_set - (1x1) double/integer: Sequence identifier (1, 2, 3, or 4)
%
%   Outputs:
%       R_mat  - (3x3) double: Rotation matrix in SO(3)
%
%   See also getEulerPosVec, SO3R3_SE3.
    
	CPHI = cos(EA_vec(1));
	CTHT = cos(EA_vec(2));
	CPSI = cos(EA_vec(3));

	SPHI = sin(EA_vec(1));
	STHT = sin(EA_vec(2));
	SPSI = sin(EA_vec(3));

    switch EA_set
        case 1 % ZYZ
            R_mat = [   CTHT*CPHI*CPSI-SPHI*SPSI, -CPSI*SPHI-CTHT*CPHI*SPSI , CPHI*STHT
                        CPHI*SPSI+CTHT*CPSI*SPHI, CPHI*CPSI-CTHT*SPHI*SPSI  , STHT*SPHI
                        -CPSI*STHT              , STHT*SPSI                 , CTHT      ];
        case 2 % ZYX
            R_mat = [   CTHT*CPHI   , CPHI*SPSI*STHT-CPSI*SPHI  , SPSI*SPHI+CPSI*CPHI*STHT
                        CTHT*SPHI   , CPSI*CPHI+SPSI*STHT*SPHI  , CPSI*STHT*SPHI-CPHI*SPSI
                        -STHT   	, CTHT*SPSI                 , CPSI*CTHT             ];
        case 3 % XYZ
            R_mat = [   CTHT*CPSI               , -CTHT*SPSI                , STHT
                        CPHI*SPSI+CPSI*SPHI*STHT, CPHI*CPSI-SPHI*STHT*SPSI  , -CTHT*SPHI
                        SPHI*SPSI-CPHI*CPSI*STHT, CPSI*SPHI+CPHI*STHT*SPSI  , CPHI*CTHT ];
        otherwise % ZXZ
            R_mat = [   CPHI*CPSI-CTHT*SPHI*SPSI, -CPHI*SPSI-CTHT*CPSI*SPHI , STHT*SPHI
                        CPSI*SPHI+CTHT*CPHI*SPSI, CTHT*CPHI*CPSI-SPHI*SPSI  , -CPHI*STHT
                        STHT*SPSI               , CPSI*STHT                 , CTHT      ];
    end
    
end