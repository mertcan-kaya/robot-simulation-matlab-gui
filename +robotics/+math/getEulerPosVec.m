function r_pos = getEulerPosVec(R_mat, eulerSet)

    sx = R_mat(1,1);
    sy = R_mat(2,1);
    sz = R_mat(3,1);
    nx = R_mat(1,2);
    ny = R_mat(2,2);
    nz = R_mat(3,2);
    ax = R_mat(1,3);
    ay = R_mat(2,3);
    az = R_mat(3,3);

    switch eulerSet
        case 1 % ZYZ
            phi = atan2(ay, ax);
            tht = atan2(cos(phi) * ax + sin(phi) * ay, az);
            psi = atan2(cos(phi) * sy - sin(phi) * sx, cos(phi) * ny - sin(phi) * nx);
        case 2 % ZYX
            phi = atan2(sy, sx);
            tht = atan2(-sz, cos(phi) * sx + sin(phi) * sy);
            psi = atan2(-cos(phi) * ay + sin(phi) * ax, cos(phi) * ny - sin(phi) * nx);
        case 3 % XYZ
            phi = atan2(-ay, az);
            tht = atan2(ax, cos(phi) * az - sin(phi) * ay);
            psi = atan2(cos(phi) * sy + sin(phi) * sz, cos(phi) * ny + sin(phi) * nz);
        otherwise % ZXZ
            phi = atan2(-ax, ay);
            tht = atan2(sin(phi) * ax - cos(phi) * ay, az);
            psi = atan2(-cos(phi) * nx - sin(phi) * ny, cos(phi) * sx + sin(phi) * sy);
    end

    r_pos = [phi;tht;psi];

end
    