function Ri_j = getRi_j(alpha, theta)
% GETRI_J Computes the Modified Denavit-Hartenberg (MDH) relative rotation matrix.
%   Ri_j = GETRI_J(ALPHA, THETA) computes the 3x3 rotation matrix from frame i
%   to frame j according to Modified DH parameters:
%       Ri_j = Rot_x(alpha) * Rot_z(theta)
%
%   Inputs:
%       alpha - double scalar: Link twist angle in radians
%       theta - double scalar: Joint angle in radians
%
%   Outputs:
%       Ri_j  - (3x3) double: Relative rotation matrix in SO(3)
%
%   See also getri_j_vec, getMDHTransform.

    Ri_j = rot_x(alpha)*rot_z(theta);

    function output = rot_x(input)
        output = [	1	0           0
                    0	cos(input)  -sin(input)
                    0	sin(input)	cos(input)];
    end
    
    function output = rot_z(input)
        output = [  cos(input)	-sin(input)	0
                    sin(input)	 cos(input)	0
                    0            0          1];
    end

end