function X = SO3R3_R66_twist(R, r)
% SO3R3_R66_TWIST Computes the (6x6) spatial motion twist transformation matrix.
%   X = SO3R3_R66_TWIST(R, r) constructs the 6x6 spatial adjoint matrix for
%   transforming spatial velocities/accelerations across frames:
%
%       X = [ R , [r]_x * R ]
%           [ 0 ,    R      ]
%
%   Inputs:
%       R - (3x3) double: Rotation matrix in SO(3)
%       r - (3x1) double: Translation vector in R^3 [m]
%
%   Outputs:
%       X - (6x6) double: Spatial motion transformation matrix
%
%   See also SkewSym, SO3R3_SE3.

    X = zeros(6,6);
    X(1:3, 1:3) = R;
    X(4:6, 4:6) = R;
    
    % Analytical [r]_x * R
    X(1, 4:6) = -r(3)*R(2,:) + r(2)*R(3,:);
    X(2, 4:6) =  r(3)*R(1,:) - r(1)*R(3,:);
    X(3, 4:6) = -r(2)*R(1,:) + r(1)*R(2,:);
end