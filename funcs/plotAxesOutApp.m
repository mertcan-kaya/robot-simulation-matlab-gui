function plotAxesOutApp(app)
    
    hold(app.UIAxes,'on')

    if app.running_flag == 1
        q_posAct = app.act.q_pos;
        q_posDes = app.des.q_pos;
    else
        q_posAct = app.ini.q_pos;
        q_posDes = app.fin.q_pos;
    end

    TactI_h = getTransMatrix(app.TI_0,app.kin.a_j,app.kin.alpha_j,app.kin.d_j,app.kin.theta_O_j,q_posAct);
    TdesI_h = getTransMatrix(app.TI_0,app.kin.a_j,app.kin.alpha_j,app.kin.d_j,app.kin.theta_O_j,q_posDes);

    if app.ee_att == 1 && app.robot_model == 2
        Tn_sC   = [eye(3),[0;0;0.0085];zeros(1,3),1];
        TsC_S   = [eye(3),[0;0;0.0375];zeros(1,3),1];
        TS_gC   = [eye(3),[0;0;0.0111];zeros(1,3),1];
        TgC_gB  = [eye(3),[0;0;0.0900];zeros(1,3),1];
        % TgB_E   = [eye(3),[0;0;0.0403];zeros(1,3),1];
        TgB_oKa = [rot_y(pi/4),[0.035;0;-0.035];zeros(1,3),1];
        TgB_oKb = [rot_z(pi)*rot_y(pi/4),[-0.035;0;-0.035];zeros(1,3),1];
        TgB_oFa = [rot_y(pi/2),[0.06;0;-0.015];zeros(1,3),1];
        TgB_oFb = [rot_z(pi)*rot_y(pi/2),[-0.06;0;-0.015];zeros(1,3),1];
        TgB_iKa = [rot_y(pi/2.25),[0.02;0;-0.025];zeros(1,3),1];
        TgB_iKb = [rot_z(pi)*rot_y(pi/2.25),[-0.02;0;-0.025];zeros(1,3),1];
        TgB_iFa = [rot_y(pi/2),[0.065;0;0.035];zeros(1,3),1];
        TgB_iFb = [rot_z(pi)*rot_y(pi/2),[-0.065;0;0.035];zeros(1,3),1];
        dz_e = 0.1874; % Tn_sC(3,4)+TsC_S(3,4)+TS_gC(3,4)+TgC_gB(3,4)+TgB_E(3,4);
    end
    if app.ee_att > 0 && (app.robot_model == 4 || app.robot_model == 5)
        if app.robot_model == 5
            sensor_angle = pi/4;
        else
            sensor_angle = 0;
        end
        sensor_length = 0.0333;
        gripper_length = 0.150;
        tool_lengthA = 0.100;
        tool_lengthB = 0.081;
    end

    if app.ghost_on == 1
        nvd1 = [app.ms.s1.V,ones(size(app.ms.s1.V(:,1)))]*TdesI_h(:,:,2)';
        nvd2 = [app.ms.s2.V,ones(size(app.ms.s2.V(:,1)))]*TdesI_h(:,:,3)';
        nvd3 = [app.ms.s3.V,ones(size(app.ms.s3.V(:,1)))]*TdesI_h(:,:,4)';
        nvd4 = [app.ms.s4.V,ones(size(app.ms.s4.V(:,1)))]*TdesI_h(:,:,5)';
        nvd5 = [app.ms.s5.V,ones(size(app.ms.s5.V(:,1)))]*TdesI_h(:,:,6)';
        nvd6 = [app.ms.s6.V,ones(size(app.ms.s6.V(:,1)))]*TdesI_h(:,:,7)';
        if app.kin.n == 7
            nvd7 = [app.ms.s7.V,ones(size(app.ms.s7.V(:,1)))]*TdesI_h(:,:,8)';
        end
        if app.ee_att > 0
            if app.robot_model == 1 || app.robot_model == 3
                nvdE = [app.ms.sE.V,ones(size(app.ms.sE.V(:,1)))]*TdesI_h(:,:,end)';
                if app.high_quality == 1 && app.robot_model == 1
                    nvdFl = [app.ms.sFl.V,ones(size(app.ms.sFl.V(:,1)))]*Rot_z(pi)'*Trn_y(-app.qEDes)'*TdesI_h(:,:,end)';
                    nvdFr = [app.ms.sFr.V,ones(size(app.ms.sFr.V(:,1)))]*Trn_y(app.qEDes)'*TdesI_h(:,:,end)';
                end
                if app.robot_model == 3
                    nvdE2 = [app.ms.sE2.V,ones(size(app.ms.sE2.V(:,1)))]*Rot_x(-app.qEDes)'*Trn_z(-0.1)'*TdesI_h(:,:,end)';
                end
            elseif app.robot_model == 2
                TdesI_sC = TdesI_h(:,:,end)*Tn_sC*Trn_z(-dz_e);
                TdesI_S = TdesI_sC*TsC_S;
                TdesI_gC = TdesI_S*TS_gC;
                TdesI_gB = TdesI_gC*TgC_gB;
                TdesI_oKa = TdesI_gB*TgB_oKa;
                TdesI_oKb = TdesI_gB*TgB_oKb;
                TdesI_oFa = TdesI_gB*TgB_oFa;
                TdesI_oFb = TdesI_gB*TgB_oFb;
                TdesI_iKa = TdesI_gB*TgB_iKa;
                TdesI_iKb = TdesI_gB*TgB_iKb;
                TdesI_iFa = TdesI_gB*TgB_iFa;
                TdesI_iFb = TdesI_gB*TgB_iFb;
                nvdSC = [app.ms.sSC.V,ones(size(app.ms.sSC.V(:,1)))]*TdesI_sC';
                nvdS = [app.ms.sS.V,ones(size(app.ms.sS.V(:,1)))]*TdesI_S';
                nvdGC = [app.ms.sGC.V,ones(size(app.ms.sGC.V(:,1)))]*TdesI_gC';
                nvdGB = [app.ms.sGB.V,ones(size(app.ms.sGB.V(:,1)))]*TdesI_gB';
                nvdOKa = [app.ms.sOK.V,ones(size(app.ms.sOK.V(:,1)))]*TdesI_oKa';
                nvdOKb = [app.ms.sOK.V,ones(size(app.ms.sOK.V(:,1)))]*TdesI_oKb';
                nvdOFa = [app.ms.sOF.V,ones(size(app.ms.sOF.V(:,1)))]*TdesI_oFa';
                nvdOFb = [app.ms.sOF.V,ones(size(app.ms.sOF.V(:,1)))]*TdesI_oFb';
                nvdIKa = [app.ms.sIK.V,ones(size(app.ms.sIK.V(:,1)))]*TdesI_iKa';
                nvdIKb = [app.ms.sIK.V,ones(size(app.ms.sIK.V(:,1)))]*TdesI_iKb';
                nvdIFa = [app.ms.sIF.V,ones(size(app.ms.sIF.V(:,1)))]*TdesI_iFa';
                nvdIFb = [app.ms.sIF.V,ones(size(app.ms.sIF.V(:,1)))]*TdesI_iFb';
            else
                nvdS = [app.ms.sS.V,ones(size(app.ms.sS.V(:,1)))]*Rot_z(sensor_angle)'*Trn_z(sensor_length)'*Trn_z(0.110)'*TdesI_h(:,:,7)';
            end
            if app.ee_att == 2
                nvdG = [app.ms.sG.V,ones(size(app.ms.sG.V(:,1)))]*Rot_z(sensor_angle)'*Trn_z(gripper_length)'*Trn_z(sensor_length)'*Trn_z(0.110)'*TdesI_h(:,:,7)';
            end
            if app.ee_att == 3
                nvdTa = [app.ms.sTa.V,ones(size(app.ms.sTa.V(:,1)))]*Rot_z(sensor_angle)'*Trn_z(tool_lengthA)'*Trn_z(sensor_length)'*Trn_z(0.110)'*TdesI_h(:,:,7)';
                nvdTb = [app.ms.sTb.V,ones(size(app.ms.sTb.V(:,1)))]*Rot_z(sensor_angle)'*Trn_z(tool_lengthA+tool_lengthB)'*Trn_z(sensor_length)'*Trn_z(0.110)'*TdesI_h(:,:,7)';
            end
        end
    end
    nva0 = [app.ms.s0.V,ones(size(app.ms.s0.V(:,1)))]*TactI_h(:,:,1)';
    nva1 = [app.ms.s1.V,ones(size(app.ms.s1.V(:,1)))]*TactI_h(:,:,2)';
    nva2 = [app.ms.s2.V,ones(size(app.ms.s2.V(:,1)))]*TactI_h(:,:,3)';
    nva3 = [app.ms.s3.V,ones(size(app.ms.s3.V(:,1)))]*TactI_h(:,:,4)';
    nva4 = [app.ms.s4.V,ones(size(app.ms.s4.V(:,1)))]*TactI_h(:,:,5)';
    nva5 = [app.ms.s5.V,ones(size(app.ms.s5.V(:,1)))]*TactI_h(:,:,6)';
    nva6 = [app.ms.s6.V,ones(size(app.ms.s6.V(:,1)))]*TactI_h(:,:,7)';
    if app.kin.n == 7
        nva7 = [app.ms.s7.V,ones(size(app.ms.s7.V(:,1)))]*TactI_h(:,:,8)';
    end
    if app.ee_att > 0
        if app.robot_model == 1 || app.robot_model == 3
            nvaE = [app.ms.sE.V,ones(size(app.ms.sE.V(:,1)))]*TactI_h(:,:,end)';
            if app.high_quality == 1 && app.robot_model == 1
                nvaFl = [app.ms.sFl.V,ones(size(app.ms.sFl.V(:,1)))]*Rot_z(pi)'*Trn_y(-app.qEIni)'*TactI_h(:,:,end)';
                nvaFr = [app.ms.sFr.V,ones(size(app.ms.sFr.V(:,1)))]*Trn_y(app.qEIni)'*TactI_h(:,:,end)';
            end
            if app.robot_model == 3
                nvaE2 = [app.ms.sE2.V,ones(size(app.ms.sE2.V(:,1)))]*Rot_x(-app.qEIni)'*Trn_z(-0.1)'*TactI_h(:,:,end)';
            end
        elseif app.robot_model == 2
            TactI_sC = TactI_h(:,:,end)*Tn_sC*Trn_z(-dz_e);
            TactI_S = TactI_sC*TsC_S;
            TactI_gC = TactI_S*TS_gC;
            TactI_gB = TactI_gC*TgC_gB;
            TactI_oKa = TactI_gB*TgB_oKa;
            TactI_oKb = TactI_gB*TgB_oKb;
            TactI_oFa = TactI_gB*TgB_oFa;
            TactI_oFb = TactI_gB*TgB_oFb;
            TactI_iKa = TactI_gB*TgB_iKa;
            TactI_iKb = TactI_gB*TgB_iKb;
            TactI_iFa = TactI_gB*TgB_iFa;
            TactI_iFb = TactI_gB*TgB_iFb;
            nvaSC = [app.ms.sSC.V,ones(size(app.ms.sSC.V(:,1)))]*TactI_sC';
            nvaS = [app.ms.sS.V,ones(size(app.ms.sS.V(:,1)))]*TactI_S';
            nvaGC = [app.ms.sGC.V,ones(size(app.ms.sGC.V(:,1)))]*TactI_gC';
            nvaGB = [app.ms.sGB.V,ones(size(app.ms.sGB.V(:,1)))]*TactI_gB';
            nvaOKa = [app.ms.sOK.V,ones(size(app.ms.sOK.V(:,1)))]*TactI_oKa';
            nvaOKb = [app.ms.sOK.V,ones(size(app.ms.sOK.V(:,1)))]*TactI_oKb';
            nvaOFa = [app.ms.sOF.V,ones(size(app.ms.sOF.V(:,1)))]*TactI_oFa';
            nvaOFb = [app.ms.sOF.V,ones(size(app.ms.sOF.V(:,1)))]*TactI_oFb';
            nvaIKa = [app.ms.sIK.V,ones(size(app.ms.sIK.V(:,1)))]*TactI_iKa';
            nvaIKb = [app.ms.sIK.V,ones(size(app.ms.sIK.V(:,1)))]*TactI_iKb';
            nvaIFa = [app.ms.sIF.V,ones(size(app.ms.sIF.V(:,1)))]*TactI_iFa';
            nvaIFb = [app.ms.sIF.V,ones(size(app.ms.sIF.V(:,1)))]*TactI_iFb';
        else
            nvaS = [app.ms.sS.V,ones(size(app.ms.sS.V(:,1)))]*Rot_z(sensor_angle)'*Trn_z(sensor_length)'*Trn_z(0.110)'*TactI_h(:,:,7)';
        end
        if app.ee_att == 2
            nvaG = [app.ms.sG.V,ones(size(app.ms.sG.V(:,1)))]*Rot_z(sensor_angle)'*Trn_z(gripper_length)'*Trn_z(sensor_length)'*Trn_z(0.110)'*TactI_h(:,:,7)';
        end
        if app.ee_att == 3
            nvaTa = [app.ms.sTa.V,ones(size(app.ms.sTa.V(:,1)))]*Rot_z(sensor_angle)'*Trn_z(tool_lengthA)'*Trn_z(sensor_length)'*Trn_z(0.110)'*TactI_h(:,:,7)';
            nvaTb = [app.ms.sTb.V,ones(size(app.ms.sTb.V(:,1)))]*Rot_z(sensor_angle)'*Trn_z(tool_lengthA+tool_lengthB)'*Trn_z(sensor_length)'*Trn_z(0.110)'*TactI_h(:,:,7)';
        end
    end
    if app.coord_frame_on == 1
        nvdF = [app.ms.sF.V,ones(size(app.ms.sF.V(:,1)))]*TdesI_h(:,:,end)';
        nvaF = [app.ms.sF.V,ones(size(app.ms.sF.V(:,1)))]*TactI_h(:,:,end)';
    end

    if app.ghost_on == 1
        set(app.Pobj_d.p1,'Vertices',nvd1(:,1:3),'FaceAlpha',0.25)
        set(app.Pobj_d.p2,'Vertices',nvd2(:,1:3),'FaceAlpha',0.25)
        set(app.Pobj_d.p3,'Vertices',nvd3(:,1:3),'FaceAlpha',0.25)
        set(app.Pobj_d.p4,'Vertices',nvd4(:,1:3),'FaceAlpha',0.25)
        set(app.Pobj_d.p5,'Vertices',nvd5(:,1:3),'FaceAlpha',0.25)
        set(app.Pobj_d.p6,'Vertices',nvd6(:,1:3),'FaceAlpha',0.25)
        if app.kin.n == 7
            set(app.Pobj_d.p7,'Vertices',nvd7(:,1:3),'FaceAlpha',0.25)
        end
        if app.ee_att > 0
            if app.robot_model == 1 || app.robot_model == 3
                set(app.Pobj_d.pE,'Vertices',nvdE(:,1:3),'FaceAlpha',0.25)
                if app.high_quality == 1 && app.robot_model == 1
                    set(app.Pobj_d.pFl,'Vertices',nvdFl(:,1:3),'FaceAlpha',0.25)
                    set(app.Pobj_d.pFr,'Vertices',nvdFr(:,1:3),'FaceAlpha',0.25)
                end
                if app.robot_model == 3
                    set(app.Pobj_d.pE2,'Vertices',nvdE2(:,1:3),'FaceAlpha',0.25)
                end
            elseif app.robot_model == 2
                set(app.Pobj_d.pSC,'Vertices',nvdSC(:,1:3),'FaceAlpha',0.25)
                set(app.Pobj_d.pS,'Vertices',nvdS(:,1:3),'FaceAlpha',0.25)
                set(app.Pobj_d.pGC,'Vertices',nvdGC(:,1:3),'FaceAlpha',0.25)
                set(app.Pobj_d.pGB,'Vertices',nvdGB(:,1:3),'FaceAlpha',0.25)
                set(app.Pobj_d.pOKa,'Vertices',nvdOKa(:,1:3),'FaceAlpha',0.25)
                set(app.Pobj_d.pOKb,'Vertices',nvdOKb(:,1:3),'FaceAlpha',0.25)
                set(app.Pobj_d.pOFa,'Vertices',nvdOFa(:,1:3),'FaceAlpha',0.25)
                set(app.Pobj_d.pOFb,'Vertices',nvdOFb(:,1:3),'FaceAlpha',0.25)
                set(app.Pobj_d.pIKa,'Vertices',nvdIKa(:,1:3),'FaceAlpha',0.25)
                set(app.Pobj_d.pIKb,'Vertices',nvdIKb(:,1:3),'FaceAlpha',0.25)
                set(app.Pobj_d.pIFa,'Vertices',nvdIFa(:,1:3),'FaceAlpha',0.25)
                set(app.Pobj_d.pIFb,'Vertices',nvdIFb(:,1:3),'FaceAlpha',0.25)
            else
                set(app.Pobj_d.pS,'Vertices',nvdS(:,1:3),'FaceAlpha',0.25)
            end
            if app.ee_att == 2
                set(app.Pobj_d.pG,'Vertices',nvdG(:,1:3),'FaceAlpha',0.25)
            end
            if app.ee_att == 3
                set(app.Pobj_d.pTa,'Vertices',nvdTa(:,1:3),'FaceAlpha',0.25)
                set(app.Pobj_d.pTb,'Vertices',nvdTb(:,1:3),'FaceAlpha',0.25)
            end
        end
    end
    set(app.Pobj_f.p0,'Vertices',nva0(:,1:3))
    set(app.Pobj_f.p1,'Vertices',nva1(:,1:3))
    set(app.Pobj_f.p2,'Vertices',nva2(:,1:3))
    set(app.Pobj_f.p3,'Vertices',nva3(:,1:3))
    set(app.Pobj_f.p4,'Vertices',nva4(:,1:3))
    set(app.Pobj_f.p5,'Vertices',nva5(:,1:3))
    set(app.Pobj_f.p6,'Vertices',nva6(:,1:3))
    if app.kin.n == 7
        set(app.Pobj_f.p7,'Vertices',nva7(:,1:3))
    end
    if app.ee_att > 0
        if app.robot_model == 1 || app.robot_model == 3
            set(app.Pobj_f.pE,'Vertices',nvaE(:,1:3))
            if app.high_quality == 1 && app.robot_model == 1
                set(app.Pobj_f.pFl,'Vertices',nvaFl(:,1:3))
                set(app.Pobj_f.pFr,'Vertices',nvaFr(:,1:3))
            end
            if app.robot_model == 3
                set(app.Pobj_f.pE2,'Vertices',nvaE2(:,1:3))
            end
        elseif app.robot_model == 2
            set(app.Pobj_f.pSC,'Vertices',nvaSC(:,1:3))
            set(app.Pobj_f.pS,'Vertices',nvaS(:,1:3))
            set(app.Pobj_f.pGC,'Vertices',nvaGC(:,1:3))
            set(app.Pobj_f.pGB,'Vertices',nvaGB(:,1:3))
            set(app.Pobj_f.pOKa,'Vertices',nvaOKa(:,1:3))
            set(app.Pobj_f.pOKb,'Vertices',nvaOKb(:,1:3))
            set(app.Pobj_f.pOFa,'Vertices',nvaOFa(:,1:3))
            set(app.Pobj_f.pOFb,'Vertices',nvaOFb(:,1:3))
            set(app.Pobj_f.pIKa,'Vertices',nvaIKa(:,1:3))
            set(app.Pobj_f.pIKb,'Vertices',nvaIKb(:,1:3))
            set(app.Pobj_f.pIFa,'Vertices',nvaIFa(:,1:3))
            set(app.Pobj_f.pIFb,'Vertices',nvaIFb(:,1:3))
        elseif app.robot_model == 3
            set(app.Pobj_f.pE,'Vertices',nvaE(:,1:3))
            set(app.Pobj_f.pE2,'Vertices',nvaE2(:,1:3))
        else
            set(app.Pobj_f.pS,'Vertices',nvaS(:,1:3))
        end
        if app.ee_att == 2
            set(app.Pobj_f.pG,'Vertices',nvaG(:,1:3))
        end
        if app.ee_att == 3
            set(app.Pobj_f.pTa,'Vertices',nvaTa(:,1:3))
            set(app.Pobj_f.pTb,'Vertices',nvaTb(:,1:3))
        end
    end
    if app.coord_frame_on == 1
        set(app.Pobj_d.pF,'Vertices',nvdF(:,1:3),'FaceAlpha',0.25)
        set(app.Pobj_f.pF,'Vertices',nvaF(:,1:3))
    end

    hold(app.UIAxes,'off')

%     drawnow
    drawnow limitrate

    function output = rot_z(input)
        output = [  cos(input)	-sin(input)	0
                    sin(input)	 cos(input)	0
                    0            0          1];
    end
    
    function output = rot_y(input)
        output = [  cos(input)	0   sin(input)
                    0           1   0
                    -sin(input)	0	cos(input)];
    end

    function output = Trn_x(input)
        output = [  1 0 0 input
                    0 1 0 0
                    0 0 1 0
                    0 0 0 1     ];
    end
    
    function output = Trn_y(input)
        output = [  1 0 0 0
                    0 1 0 input
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