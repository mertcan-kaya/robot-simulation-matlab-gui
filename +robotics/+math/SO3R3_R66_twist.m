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

    X = [R, robotics.math.SkewSym(r)*R; zeros(3), R];
end