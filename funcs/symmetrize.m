function I = symmetrize(Ixx,Ixy,Ixz,Iyy,Iyz,Izz)
    I = [Ixx Ixy Ixz
         Ixy Iyy Iyz
         Ixz Iyz Izz];
end