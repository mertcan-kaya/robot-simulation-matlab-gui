function plotAxesOutApp(app)

    hold(app.UIAxes,'on')

    TactI_h = getTransMatrix(app.TI_0,app.kin.a_j,app.kin.alpha_j,app.kin.d_j,app.kin.theta_O_j,app.q_posAct);
    TdesI_h = getTransMatrix(app.TI_0,app.kin.a_j,app.kin.alpha_j,app.kin.d_j,app.kin.theta_O_j,app.q_posDes);

    if app.ghost_on == 1
        nvf1 = [app.ms.s1.V,ones(size(app.ms.s1.V(:,1)))]*TdesI_h(:,:,2)';
        nvf2 = [app.ms.s2.V,ones(size(app.ms.s2.V(:,1)))]*TdesI_h(:,:,3)';
        nvf3 = [app.ms.s3.V,ones(size(app.ms.s3.V(:,1)))]*TdesI_h(:,:,4)';
        nvf4 = [app.ms.s4.V,ones(size(app.ms.s4.V(:,1)))]*TdesI_h(:,:,5)';
        nvf5 = [app.ms.s5.V,ones(size(app.ms.s5.V(:,1)))]*TdesI_h(:,:,6)';
        nvf6 = [app.ms.s6.V,ones(size(app.ms.s6.V(:,1)))]*TdesI_h(:,:,7)';
        if app.kin.n == 7
            nvf7 = [app.ms.s7.V,ones(size(app.ms.s7.V(:,1)))]*TdesI_h(:,:,8)';
        end
    end
    nvi0 = [app.ms.s0.V,ones(size(app.ms.s0.V(:,1)))]*TactI_h(:,:,1)';
    nvi1 = [app.ms.s1.V,ones(size(app.ms.s1.V(:,1)))]*TactI_h(:,:,2)';
    nvi2 = [app.ms.s2.V,ones(size(app.ms.s2.V(:,1)))]*TactI_h(:,:,3)';
    nvi3 = [app.ms.s3.V,ones(size(app.ms.s3.V(:,1)))]*TactI_h(:,:,4)';
    nvi4 = [app.ms.s4.V,ones(size(app.ms.s4.V(:,1)))]*TactI_h(:,:,5)';
    nvi5 = [app.ms.s5.V,ones(size(app.ms.s5.V(:,1)))]*TactI_h(:,:,6)';
    nvi6 = [app.ms.s6.V,ones(size(app.ms.s6.V(:,1)))]*TactI_h(:,:,7)';
    if app.kin.n == 7
        nvi7 = [app.ms.s7.V,ones(size(app.ms.s7.V(:,1)))]*TactI_h(:,:,8)';
    end
    if app.coord_frame_on == 1
        nvfF = [app.ms.sF.V,ones(size(app.ms.sF.V(:,1)))]*TdesI_h(:,:,end)';
        nviF = [app.ms.sF.V,ones(size(app.ms.sF.V(:,1)))]*TactI_h(:,:,end)';
    end

    if app.ghost_on == 1
        set(app.Pobj_d.p1,'Vertices',nvf1(:,1:3),'FaceAlpha',0.25)
        set(app.Pobj_d.p2,'Vertices',nvf2(:,1:3),'FaceAlpha',0.25)
        set(app.Pobj_d.p3,'Vertices',nvf3(:,1:3),'FaceAlpha',0.25)
        set(app.Pobj_d.p4,'Vertices',nvf4(:,1:3),'FaceAlpha',0.25)
        set(app.Pobj_d.p5,'Vertices',nvf5(:,1:3),'FaceAlpha',0.25)
        set(app.Pobj_d.p6,'Vertices',nvf6(:,1:3),'FaceAlpha',0.25)
        if app.kin.n == 7
            set(app.Pobj_d.p7,'Vertices',nvf7(:,1:3),'FaceAlpha',0.25)
        end
    end
    set(app.Pobj_f.p0,'Vertices',nvi0(:,1:3))
    set(app.Pobj_f.p1,'Vertices',nvi1(:,1:3))
    set(app.Pobj_f.p2,'Vertices',nvi2(:,1:3))
    set(app.Pobj_f.p3,'Vertices',nvi3(:,1:3))
    set(app.Pobj_f.p4,'Vertices',nvi4(:,1:3))
    set(app.Pobj_f.p5,'Vertices',nvi5(:,1:3))
    set(app.Pobj_f.p6,'Vertices',nvi6(:,1:3))
    if app.kin.n == 7
        set(app.Pobj_f.p7,'Vertices',nvi7(:,1:3))
    end
    if app.coord_frame_on == 1
        set(app.Pobj_d.pF,'Vertices',nvfF(:,1:3),'FaceAlpha',0.25)
        set(app.Pobj_f.pF,'Vertices',nviF(:,1:3))
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