function I = symmetrize(Ixx, Ixy, Ixz, Iyy, Iyz, Izz)
% SYMMETRIZE Reconstructs a 3x3 symmetric matrix from its 6 unique components.
%   I = SYMMETRIZE(Ixx, Ixy, Ixz, Iyy, Iyz, Izz) constructs a 3x3 symmetric
%   matrix (e.g. spatial inertia tensor):
%
%       I = [ Ixx , Ixy , Ixz ]
%           [ Ixy , Iyy , Iyz ]
%           [ Ixz , Iyz , Izz ]
%
%   Inputs:
%       Ixx, Ixy, Ixz, Iyy, Iyz, Izz - double scalars: Inertia tensor components [kg*m^2]
%
%   Outputs:
%       I - (3x3) double: Symmetric inertia matrix
%
%   See also SymVec, DotMat.

    I = [Ixx, Ixy, Ixz;
         Ixy, Iyy, Iyz;
         Ixz, Iyz, Izz];
end