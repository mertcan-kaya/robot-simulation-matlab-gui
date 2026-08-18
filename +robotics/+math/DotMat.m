function D = DotMat(r)
% DOTMAT Computes the (3x6) dot product matrix for symmetric tensor multiplication.
%   D = DOTMAT(r) constructs a 3x6 matrix such that D * SymVec(I) = I * r
%   for any 3x3 symmetric matrix (e.g. inertia tensor) I.
%
%   Inputs:
%       r - (3x1) or (1x3) double: Vector in R^3
%
%   Outputs:
%       D - (3x6) double: Dot product matrix
%
%   See also SymVec, SkewSym.

    D = [   r(1)	, r(2)  , r(3)  , 0 	, 0     , 0
            0       , r(1)  , 0     , r(2)  , r(3)  , 0
            0       , 0     , r(1)  , 0     , r(2)  , r(3)  ];
end