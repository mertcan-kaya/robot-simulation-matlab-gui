function plotAxesOutApp(app)

    hold(app.UIAxes,'on')

    TI_h = getTransMatrix(app.TI_0,app.kin.a_j,app.kin.alpha_j,app.kin.d_j,app.kin.theta_O_j,app.q_pos);

    nv0 = [app.ms.s0.V,ones(size(app.ms.s0.V(:,1)))]*TI_h(:,:,1)';
    nv1 = [app.ms.s1.V,ones(size(app.ms.s1.V(:,1)))]*TI_h(:,:,2)';
    nv2 = [app.ms.s2.V,ones(size(app.ms.s2.V(:,1)))]*TI_h(:,:,3)';
    nv3 = [app.ms.s3.V,ones(size(app.ms.s3.V(:,1)))]*TI_h(:,:,4)';
    nv4 = [app.ms.s4.V,ones(size(app.ms.s4.V(:,1)))]*TI_h(:,:,5)';
    nv5 = [app.ms.s5.V,ones(size(app.ms.s5.V(:,1)))]*TI_h(:,:,6)';
    nv6 = [app.ms.s6.V,ones(size(app.ms.s6.V(:,1)))]*TI_h(:,:,7)';
    if app.kin.n == 7
        nv7 = [app.ms.s7.V,ones(size(app.ms.s7.V(:,1)))]*TI_h(:,:,8)';
    end
    if app.coord_frame_on == 1
        nvF = [app.ms.sF.V,ones(size(app.ms.sF.V(:,1)))]*TI_h(:,:,end)';
    end

    set(app.Pobj.p0,'Vertices',nv0(:,1:3))
    set(app.Pobj.p1,'Vertices',nv1(:,1:3))
    set(app.Pobj.p2,'Vertices',nv2(:,1:3))
    set(app.Pobj.p3,'Vertices',nv3(:,1:3))
    set(app.Pobj.p4,'Vertices',nv4(:,1:3))
    set(app.Pobj.p5,'Vertices',nv5(:,1:3))
    set(app.Pobj.p6,'Vertices',nv6(:,1:3))
    if app.kin.n == 7
        set(app.Pobj.p7,'Vertices',nv7(:,1:3))
    end
    if app.coord_frame_on == 1
        set(app.Pobj.pF,'Vertices',nvF(:,1:3))
    end

    hold(app.UIAxes,'off')

%     drawnow
    drawnow limitrate

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
    
    function TI_i = getTransMatrix(TI_0,a_j,alpha_j,d_j,theta_j,q_j)
        
        k = length(q_j);
    
        TI_i = zeros(4,4,k+1);
    
        TI_i(:,:,1) = TI_0;
    
        for j = 1:k
            Rot_x_i = Rot_x(alpha_j(j));
            Trn_x_i	= Trn_x(a_j(j));
            Rot_z_j = Rot_z(theta_j(j)+q_j(j));
            Trn_z_j = Trn_z(d_j(j));
    
            TI_i(:,:,j+1) = TI_i(:,:,j) * Rot_x_i * Trn_x_i * Rot_z_j * Trn_z_j;
        end
        Rot_x_i = Rot_x(alpha_j(k+1));
        Trn_x_i	= Trn_x(a_j(k+1));
        Rot_z_j = Rot_z(theta_j(k+1));
        Trn_z_j = Trn_z(d_j(k+1));

        TI_i(:,:,k+2) = TI_i(:,:,k+1) * Rot_x_i * Trn_x_i * Rot_z_j * Trn_z_j;
        
    end

end