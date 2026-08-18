function [R, r] = SE3_SO3R3(T)
% SE3_SO3R3 Extracts rotation matrix R and position vector r from SE(3).
%   [R, r] = SE3_SO3R3(T) extracts the 3x3 rotation matrix R in SO(3) and
%   the 3x1 translation vector r in R^3 from a 4x4 homogeneous transformation matrix T.
%
%   Inputs:
%       T - (4x4) double: Homogeneous transformation matrix in SE(3)
%
%   Outputs:
%       R - (3x3) double: Rotation matrix in SO(3)
%       r - (3x1) double: Translation/position vector in R^3 [m]
%
%   See also SO3R3_SE3, getEulerPosVec.

    R = T(1:3,1:3);
    r = T(1:3,4);
end