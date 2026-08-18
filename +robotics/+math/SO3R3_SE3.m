function T = SO3R3_SE3(R, r)
% SO3R3_SE3 Embeds rotation matrix R and position vector r into SE(3).
%   T = SO3R3_SE3(R, r) returns the 4x4 homogeneous transformation matrix
%   constructed from an SO(3) rotation matrix R and an R^3 translation vector r:
%
%       T = [ R , r ]
%           [ 0 , 1 ]
%
%   Inputs:
%       R - (3x3) double: Rotation matrix in SO(3)
%       r - (3x1) double: Translation/position vector in R^3 [m]
%
%   Outputs:
%       T - (4x4) double: Homogeneous transformation matrix in SE(3)
%
%   See also SE3_SO3R3, getRotMatfromEA.

    T = [R, r; zeros(1,3), 1];
end