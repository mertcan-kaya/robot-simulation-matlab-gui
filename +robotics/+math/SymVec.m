function L = SymVec(I)
% SYMVEC Vectorizes a (3x3) symmetric matrix into a 6x1 column vector.
%   L = SYMVEC(I) extracts the 6 unique components of a symmetric 3x3 tensor:
%       L = [ I(1,1); I(1,2); I(1,3); I(2,2); I(2,3); I(3,3) ]
%
%   Inputs:
%       I - (3x3) double: Symmetric matrix (e.g. inertia matrix)
%
%   Outputs:
%       L - (6x1) double: Unique elements vector
%
%   See also DotMat, symmetrize.

    L = [I(1,1); I(1,2); I(1,3); I(2,2); I(2,3); I(3,3)];
end