function TI_i = getTransMatrix(TI_0,a_j,alpha_j,d_j,theta_j,j_type,q_j)
    
    k = length(q_j);

    TI_i = zeros(4,4,k+2);

    TI_i(:,:,1) = TI_0;

    for j = 1:k
        Rot_x_i = Rot_x(alpha_j(j));
        Trn_x_i	= Trn_x(a_j(j));
        Rot_z_j = Rot_z(theta_j(j)+j_type(j)*q_j(j));
        Trn_z_j = Trn_z(d_j(j)+(1-j_type(j))*q_j(j));

        TI_i(:,:,j+1) = TI_i(:,:,j) * Rot_x_i * Trn_x_i * Rot_z_j * Trn_z_j;
    end
    Rot_x_i = Rot_x(alpha_j(k+1));
    Trn_x_i	= Trn_x(a_j(k+1));
    Rot_z_j = Rot_z(theta_j(k+1));
    Trn_z_j = Trn_z(d_j(k+1));

    TI_i(:,:,k+2) = TI_i(:,:,k+1) * Rot_x_i * Trn_x_i * Rot_z_j * Trn_z_j;
    
    function output = Trn_x(input)
        output = [  1 0 0 input
                    0 1 0 0
                    0 0 1 0
                    0 0 0 1     ];
    end
    
    function output = Trn_z(input)
        output = [  1 0 0 0
                    0 1 0 0
                    0 0 1 input
                    0 0 0 1     ];
    end
    
    function output = Rot_x(input)
        output = [	1	0           0           0
                    0	cos(input)  -sin(input)	0
                    0	sin(input)	cos(input)	0
                    0	0           0           1 ];
    end
    
    function output = Rot_z(input)
        output = [  cos(input)	-sin(input)	0	0
                    sin(input)	 cos(input)	0	0
                    0            0          1	0
                    0            0          0	1 ];
    end
    
end
