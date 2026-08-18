function rj_i = getrj_i_vec(theta, a, d)
% GETRJ_I_VEC Computes the Modified DH relative vector r_j^i.
%   rj_i = GETRJ_I_VEC(THETA, A, D) computes the (3x1) relative translation
%   vector from frame j to frame i expressed in frame j coordinates:
%       rj_i = [ -a*cos(theta); a*sin(theta); -d ]
%
%   Inputs:
%       theta - double scalar: Joint angle in radians
%       a     - double scalar: Link length in meters
%       d     - double scalar: Link offset in meters
%
%   Outputs:
%       rj_i  - (3x1) double: Translation vector in R^3 [m]
%
%   See also getri_j_vec, getRi_j.

    rj_i = [-a*cos(theta); a*sin(theta); -d];
end