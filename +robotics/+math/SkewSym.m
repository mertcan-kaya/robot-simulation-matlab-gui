function Sr = SkewSym(r)
% SKEWSYM Computes the (3x3) skew-symmetric cross-product matrix [r]_x.
%   Sr = SKEWSYM(r) returns the 3x3 matrix such that Sr*v = cross(r, v)
%   for any vector v in R^3:
%
%       Sr = [  0   , -r(3) ,  r(2) ]
%            [  r(3),  0    , -r(1) ]
%            [ -r(2),  r(1) ,  0    ]
%
%   Inputs:
%       r  - (3x1) or (1x3) double: Vector in R^3
%
%   Outputs:
%       Sr - (3x3) double: Skew-symmetric matrix in so(3)
%
%   See also SO3R3_R66_twist.

    Sr = [  0   , -r(3) , r(2)
            r(3), 0     , -r(1)
           -r(2), r(1)  , 0     ];
end