function ri_j = getri_j_vec(alpha, a, d)
% GETRI_J_VEC Computes the Modified DH link offset vector r_i^j.
%   ri_j = GETRI_J_VEC(ALPHA, A, D) computes the (3x1) relative translation
%   vector from frame i to frame j expressed in frame i coordinates:
%       ri_j = [ a; -d*sin(alpha); d*cos(alpha) ]
%
%   Inputs:
%       alpha - double scalar: Link twist angle in radians
%       a     - double scalar: Link length in meters
%       d     - double scalar: Link offset in meters
%
%   Outputs:
%       ri_j  - (3x1) double: Translation vector in R^3 [m]
%
%   See also getRi_j, getrj_i_vec.

    ri_j = [a; -d*sin(alpha); d*cos(alpha)];
end