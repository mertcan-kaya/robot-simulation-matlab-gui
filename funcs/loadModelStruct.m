function loadModelStruct(app)

    cla(app.UIAxes)
    light(app.UIAxes,'Position',[-1 0 1])
    
    % Load CAD files
    if app.robot_model == 5 || app.robot_model == 4
        if app.high_quality == 1
            temp = load('meshes\rx160\visual\base_link.mat');       app.ms.s0 = temp.modelstruct;
            temp = load('meshes\rx160\visual\link_1.mat');          app.ms.s1 = temp.modelstruct;
            temp = load('meshes\rx160\visual\link_2.mat');          app.ms.s2 = temp.modelstruct;
            temp = load('meshes\rx160\visual\link_3.mat');          app.ms.s3 = temp.modelstruct;
            if app.robot_model == 5
                temp = load('meshes\rx160\visual\link_4l.mat');     app.ms.s4 = temp.modelstruct;
            else
                temp = load('meshes\rx160\visual\link_4.mat');      app.ms.s4 = temp.modelstruct;
            end
            temp = load('meshes\rx160\visual\link_5.mat');          app.ms.s5 = temp.modelstruct;
            temp = load('meshes\rx160\visual\link_6.mat');          app.ms.s6 = temp.modelstruct;
            if app.ee_att > 0
                temp = load('meshes\rx160\visual\ati_delta.mat');   app.ms.sS = temp.modelstruct;
            end
            if app.ee_att == 2
                temp = load('meshes\rx160\visual\gripper.mat');     app.ms.sG = temp.modelstruct;
            end
        else
            temp = load('meshes\rx160\collision\base_link.mat');    app.ms.s0 = temp.modelstruct;
            temp = load('meshes\rx160\collision\link_1.mat');       app.ms.s1 = temp.modelstruct;
            temp = load('meshes\rx160\collision\link_2.mat');       app.ms.s2 = temp.modelstruct;
            temp = load('meshes\rx160\collision\link_3.mat');       app.ms.s3 = temp.modelstruct;
            if app.robot_model == 5
                temp = load('meshes\rx160\collision\link_4l.mat');  app.ms.s4 = temp.modelstruct;
            else
                temp = load('meshes\rx160\collision\link_4.mat');   app.ms.s4 = temp.modelstruct;
            end
            temp = load('meshes\rx160\collision\link_5.mat');       app.ms.s5 = temp.modelstruct;
            temp = load('meshes\rx160\collision\link_6.mat');       app.ms.s6 = temp.modelstruct;
            if app.ee_att > 0
                temp = load('meshes\rx160\collision\ati_delta.mat');    app.ms.sS = temp.modelstruct;
            end
            if app.ee_att == 2
                temp = load('meshes\rx160\collision\gripper.mat');      app.ms.sG = temp.modelstruct;
            end
        end
        if app.ee_att == 3
            temp = load('meshes\rx160\adaptor_holder.mat');         app.ms.sTa = temp.modelstruct;
            temp = load('meshes\rx160\rod_sphere.mat');             app.ms.sTb = temp.modelstruct;
        end
    elseif app.robot_model == 3
        temp = load('meshes\z1\z1_Link00.mat'); app.ms.s0 = temp.modelstruct;
        temp = load('meshes\z1\z1_Link01.mat'); app.ms.s1 = temp.modelstruct;
        temp = load('meshes\z1\z1_Link02.mat'); app.ms.s2 = temp.modelstruct;
        temp = load('meshes\z1\z1_Link03.mat'); app.ms.s3 = temp.modelstruct;
        temp = load('meshes\z1\z1_Link04.mat'); app.ms.s4 = temp.modelstruct;
        temp = load('meshes\z1\z1_Link05.mat'); app.ms.s5 = temp.modelstruct;
        temp = load('meshes\z1\z1_Link06.mat'); app.ms.s6 = temp.modelstruct;
        if app.ee_att == 1
            temp = load('meshes\z1\z1_GripperStator.mat');   app.ms.sE = temp.modelstruct;
            temp = load('meshes\z1\z1_GripperMover.mat');   app.ms.sE2 = temp.modelstruct;
        end
    elseif app.robot_model == 2
        temp = load('meshes\ur3\base.mat');     app.ms.s0 = temp.modelstruct;
        temp = load('meshes\ur3\shoulder.mat'); app.ms.s1 = temp.modelstruct;
        temp = load('meshes\ur3\upperarm.mat'); app.ms.s2 = temp.modelstruct;
        temp = load('meshes\ur3\forearm.mat');  app.ms.s3 = temp.modelstruct;
        temp = load('meshes\ur3\wrist1.mat');   app.ms.s4 = temp.modelstruct;
        temp = load('meshes\ur3\wrist2.mat');   app.ms.s5 = temp.modelstruct;
        temp = load('meshes\ur3\wrist3.mat');   app.ms.s6 = temp.modelstruct;
        if app.ee_att == 1
            temp = load('meshes\ur3\fts150_coupling.mat');                  app.ms.sSC = temp.modelstruct;
            temp = load('meshes\ur3\fts150.mat');                           app.ms.sS = temp.modelstruct;
            temp = load('meshes\ur3\gripper_coupling.mat');                 app.ms.sGC = temp.modelstruct;
            temp = load('meshes\ur3\arg2f_85_base_link_simp.mat');          app.ms.sGB = temp.modelstruct;
            temp = load('meshes\ur3\arg2f_85_outer_knuckle_simp.mat');      app.ms.sOK = temp.modelstruct;
            temp = load('meshes\ur3\arg2f_85_outer_finger_simp.mat');       app.ms.sOF = temp.modelstruct;
            temp = load('meshes\ur3\arg2f_85_inner_knuckle_simp.mat');      app.ms.sIK = temp.modelstruct;
            temp = load('meshes\ur3\arg2f_85_inner_finger_pad_simp.mat');   app.ms.sIF = temp.modelstruct;
        end
    else
        if app.high_quality == 1
            temp = load('meshes\fer\visual\link0.mat');   app.ms.s0 = temp.modelstruct;
            temp = load('meshes\fer\visual\link1.mat');   app.ms.s1 = temp.modelstruct;
            temp = load('meshes\fer\visual\link2.mat');   app.ms.s2 = temp.modelstruct;
            temp = load('meshes\fer\visual\link3.mat');   app.ms.s3 = temp.modelstruct;
            temp = load('meshes\fer\visual\link4.mat');   app.ms.s4 = temp.modelstruct;
            temp = load('meshes\fer\visual\link5.mat');   app.ms.s5 = temp.modelstruct;
            temp = load('meshes\fer\visual\link6.mat');   app.ms.s6 = temp.modelstruct;
            temp = load('meshes\fer\visual\link7.mat');   app.ms.s7 = temp.modelstruct;
            if app.ee_att == 1
                temp = load('meshes\fer\visual\hand.mat');   app.ms.sE = temp.modelstruct;
                temp = load('meshes\fer\visual\finger.mat');   app.ms.sFl = temp.modelstruct;
                temp = load('meshes\fer\visual\finger.mat');   app.ms.sFr = temp.modelstruct;
            end
        else
            temp = load('meshes\fer\collision\link0.mat');    app.ms.s0 = temp.modelstruct;
            temp = load('meshes\fer\collision\link1.mat');    app.ms.s1 = temp.modelstruct;
            temp = load('meshes\fer\collision\link2.mat');    app.ms.s2 = temp.modelstruct;
            temp = load('meshes\fer\collision\link3.mat');    app.ms.s3 = temp.modelstruct;
            temp = load('meshes\fer\collision\link4.mat');    app.ms.s4 = temp.modelstruct;
            temp = load('meshes\fer\collision\link5.mat');    app.ms.s5 = temp.modelstruct;
            temp = load('meshes\fer\collision\link6.mat');    app.ms.s6 = temp.modelstruct;
            temp = load('meshes\fer\collision\link7.mat');    app.ms.s7 = temp.modelstruct;
            if app.ee_att == 1
                temp = load('meshes\fer\collision\hand.mat');   app.ms.sE = temp.modelstruct;
            end
        end
    end
    
    if app.coord_frame_on == 1
        temp = load('meshes\coord_frame.mat');  app.ms.sF = temp.modelstruct;
    end

    clear temp
    
    % CAD properties
    if app.ghost_on == 1
        app.Pobj_d.p1 = patch(app.UIAxes,'Faces', app.ms.s1.F, 'Vertices', app.ms.s1.V);
        set(app.Pobj_d.p1, 'facec', 'flat');                    % Set the face color flat
        if app.robot_model == 2
            set(app.Pobj_d.p1, 'FaceVertexCData', app.ms.s1.C); % Set the color (from file)
        else
            set(app.Pobj_d.p1, 'FaceColor', app.ms.s1.C);       % Set the color (from file)
        end
        set(app.Pobj_d.p1, 'EdgeColor', 'none');                % Set the edge color
        app.Pobj_d.p2 = patch(app.UIAxes,'Faces', app.ms.s2.F, 'Vertices', app.ms.s2.V);
        set(app.Pobj_d.p2, 'facec', 'flat');                    % Set the face color flat
        if app.robot_model == 2
            set(app.Pobj_d.p2, 'FaceVertexCData', app.ms.s2.C); % Set the color (from file)
        else
            set(app.Pobj_d.p2, 'FaceColor', app.ms.s2.C);       % Set the color (from file)
        end
        set(app.Pobj_d.p2, 'EdgeColor', 'none');                % Set the edge color
        app.Pobj_d.p3 = patch(app.UIAxes,'Faces', app.ms.s3.F, 'Vertices', app.ms.s3.V);
        set(app.Pobj_d.p3, 'facec', 'flat');                    % Set the face color flat
        if app.robot_model == 2
            set(app.Pobj_d.p3, 'FaceVertexCData', app.ms.s3.C); % Set the color (from file)
        else
            set(app.Pobj_d.p3, 'FaceColor', app.ms.s3.C);       % Set the color (from file)
        end
        set(app.Pobj_d.p3, 'EdgeColor', 'none');                % Set the edge color
        app.Pobj_d.p4 = patch(app.UIAxes,'Faces', app.ms.s4.F, 'Vertices', app.ms.s4.V);
        set(app.Pobj_d.p4, 'facec', 'flat');                    % Set the face color flat
        if app.robot_model == 2
            set(app.Pobj_d.p4, 'FaceVertexCData', app.ms.s4.C); % Set the color (from file)
        else
            set(app.Pobj_d.p4, 'FaceColor', app.ms.s4.C);       % Set the color (from file)
        end
        set(app.Pobj_d.p4, 'EdgeColor', 'none');                % Set the edge color
        app.Pobj_d.p5 = patch(app.UIAxes,'Faces', app.ms.s5.F, 'Vertices', app.ms.s5.V);
        set(app.Pobj_d.p5, 'facec', 'flat');                    % Set the face color flat
        if app.robot_model == 2
            set(app.Pobj_d.p5, 'FaceVertexCData', app.ms.s5.C); % Set the color (from file)
        else
            set(app.Pobj_d.p5, 'FaceColor', app.ms.s5.C);       % Set the color (from file)
        end
        set(app.Pobj_d.p5, 'EdgeColor', 'none');                % Set the edge color
        app.Pobj_d.p6 = patch(app.UIAxes,'Faces', app.ms.s6.F, 'Vertices', app.ms.s6.V);
        set(app.Pobj_d.p6, 'facec', 'flat');                    % Set the face color flat
        if app.robot_model == 2
            set(app.Pobj_d.p6, 'FaceVertexCData', app.ms.s6.C); % Set the color (from file)
        else
            set(app.Pobj_d.p6, 'FaceColor', app.ms.s6.C);       % Set the color (from file)
        end
        set(app.Pobj_d.p6, 'EdgeColor', 'none');                % Set the edge color
        if app.robot_model == 1
            app.Pobj_d.p7 = patch(app.UIAxes,'Faces', app.ms.s7.F, 'Vertices', app.ms.s7.V);
            set(app.Pobj_d.p7, 'facec', 'flat');                % Set the face color flat
            set(app.Pobj_d.p7, 'FaceColor', app.ms.s7.C);       % Set the color (from file)
            set(app.Pobj_d.p7, 'EdgeColor', 'none');            % Set the edge color
        end
        if app.ee_att > 0
            if app.robot_model == 1 || app.robot_model == 3
                app.Pobj_d.pE = patch(app.UIAxes,'Faces', app.ms.sE.F, 'Vertices', app.ms.sE.V);
                set(app.Pobj_d.pE, 'facec', 'flat');                % Set the face color flat
                set(app.Pobj_d.pE, 'FaceColor', app.ms.sE.C);       % Set the color (from file)
                set(app.Pobj_d.pE, 'EdgeColor', 'none');            % Set the edge color
                if app.high_quality == 1 && app.robot_model == 1
                    app.Pobj_d.pFl = patch(app.UIAxes,'Faces', app.ms.sFl.F, 'Vertices', app.ms.sFl.V);
                    set(app.Pobj_d.pFl, 'facec', 'flat');                % Set the face color flat
                    set(app.Pobj_d.pFl, 'FaceColor', app.ms.sFl.C);       % Set the color (from file)
                    set(app.Pobj_d.pFl, 'EdgeColor', 'none');            % Set the edge color
                    app.Pobj_d.pFr = patch(app.UIAxes,'Faces', app.ms.sFr.F, 'Vertices', app.ms.sFr.V);
                    set(app.Pobj_d.pFr, 'facec', 'flat');                % Set the face color flat
                    set(app.Pobj_d.pFr, 'FaceColor', app.ms.sFr.C);       % Set the color (from file)
                    set(app.Pobj_d.pFr, 'EdgeColor', 'none');            % Set the edge color
                end
                if app.robot_model == 3
                    app.Pobj_d.pE2 = patch(app.UIAxes,'Faces', app.ms.sE2.F, 'Vertices', app.ms.sE2.V);
                    set(app.Pobj_d.pE2, 'facec', 'flat');                % Set the face color flat
                    set(app.Pobj_d.pE2, 'FaceColor', app.ms.sE2.C);       % Set the color (from file)
                    set(app.Pobj_d.pE2, 'EdgeColor', 'none');            % Set the edge color
                end
            elseif app.robot_model == 2
                app.Pobj_d.pSC = patch(app.UIAxes,'Faces', app.ms.sSC.F, 'Vertices' ,app.ms.sSC.V);
                set(app.Pobj_d.pSC, 'facec', 'flat');               % Set the face color flat
                set(app.Pobj_d.pSC, 'FaceVertexCData', app.ms.sSC.C);	% Set the face color
                set(app.Pobj_d.pSC, 'EdgeColor', 'none');        	% Set the edge color
                app.Pobj_d.pS = patch(app.UIAxes,'Faces', app.ms.sS.F, 'Vertices' ,app.ms.sS.V);
                set(app.Pobj_d.pS, 'facec', 'flat');               % Set the face color flat
                set(app.Pobj_d.pS, 'FaceVertexCData', app.ms.sS.C);	% Set the face color
                set(app.Pobj_d.pS, 'EdgeColor', 'none');        	% Set the edge color
                app.Pobj_d.pGC = patch(app.UIAxes,'Faces', app.ms.sGC.F, 'Vertices' ,app.ms.sGC.V);
                set(app.Pobj_d.pGC, 'facec', 'flat');               % Set the face color flat
                set(app.Pobj_d.pGC, 'FaceColor', app.ms.sGC.C);	% Set the face color
                set(app.Pobj_d.pGC, 'EdgeColor', 'none');        	% Set the edge color
                app.Pobj_d.pGB = patch(app.UIAxes,'Faces', app.ms.sGB.F, 'Vertices' ,app.ms.sGB.V);
                set(app.Pobj_d.pGB, 'facec', 'flat');               % Set the face color flat
                set(app.Pobj_d.pGB, 'FaceVertexCData', app.ms.sGB.C);	% Set the face color
                set(app.Pobj_d.pGB, 'EdgeColor', 'none');        	% Set the edge color
                app.Pobj_d.pOKa = patch(app.UIAxes,'Faces', app.ms.sOK.F, 'Vertices' ,app.ms.sOK.V);
                set(app.Pobj_d.pOKa, 'facec', 'flat');               % Set the face color flat
                set(app.Pobj_d.pOKa, 'FaceColor', app.ms.sOK.C);	% Set the face color
                set(app.Pobj_d.pOKa, 'EdgeColor', 'none');        	% Set the edge color
                app.Pobj_d.pOKb = patch(app.UIAxes,'Faces', app.ms.sOK.F, 'Vertices' ,app.ms.sOK.V);
                set(app.Pobj_d.pOKb, 'facec', 'flat');               % Set the face color flat
                set(app.Pobj_d.pOKb, 'FaceColor', app.ms.sOK.C);	% Set the face color
                set(app.Pobj_d.pOKb, 'EdgeColor', 'none');        	% Set the edge color
                app.Pobj_d.pOFa = patch(app.UIAxes,'Faces', app.ms.sOF.F, 'Vertices' ,app.ms.sOF.V);
                set(app.Pobj_d.pOFa, 'facec', 'flat');               % Set the face color flat
                set(app.Pobj_d.pOFa, 'FaceColor', app.ms.sOF.C);	% Set the face color
                set(app.Pobj_d.pOFa, 'EdgeColor', 'none');        	% Set the edge color
                app.Pobj_d.pOFb = patch(app.UIAxes,'Faces', app.ms.sOF.F, 'Vertices' ,app.ms.sOF.V);
                set(app.Pobj_d.pOFb, 'facec', 'flat');               % Set the face color flat
                set(app.Pobj_d.pOFb, 'FaceColor', app.ms.sOF.C);	% Set the face color
                set(app.Pobj_d.pOFb, 'EdgeColor', 'none');        	% Set the edge color
                app.Pobj_d.pIKa = patch(app.UIAxes,'Faces', app.ms.sIK.F, 'Vertices' ,app.ms.sIK.V);
                set(app.Pobj_d.pIKa, 'facec', 'flat');               % Set the face color flat
                set(app.Pobj_d.pIKa, 'FaceColor', app.ms.sIK.C);	% Set the face color
                set(app.Pobj_d.pIKa, 'EdgeColor', 'none');        	% Set the edge color
                app.Pobj_d.pIKb = patch(app.UIAxes,'Faces', app.ms.sIK.F, 'Vertices' ,app.ms.sIK.V);
                set(app.Pobj_d.pIKb, 'facec', 'flat');               % Set the face color flat
                set(app.Pobj_d.pIKb, 'FaceColor', app.ms.sIK.C);	% Set the face color
                set(app.Pobj_d.pIKb, 'EdgeColor', 'none');        	% Set the edge color
                app.Pobj_d.pIFa = patch(app.UIAxes,'Faces', app.ms.sIF.F, 'Vertices' ,app.ms.sIF.V);
                set(app.Pobj_d.pIFa, 'facec', 'flat');               % Set the face color flat
                set(app.Pobj_d.pIFa, 'FaceVertexCData', app.ms.sIF.C);	% Set the face color
                set(app.Pobj_d.pIFa, 'EdgeColor', 'none');        	% Set the edge color
                app.Pobj_d.pIFb = patch(app.UIAxes,'Faces', app.ms.sIF.F, 'Vertices' ,app.ms.sIF.V);
                set(app.Pobj_d.pIFb, 'facec', 'flat');               % Set the face color flat
                set(app.Pobj_d.pIFb, 'FaceVertexCData', app.ms.sIF.C);	% Set the face color
                set(app.Pobj_d.pIFb, 'EdgeColor', 'none');        	% Set the edge color
            else
                app.Pobj_d.pS = patch(app.UIAxes,'Faces', app.ms.sS.F, 'Vertices', app.ms.sS.V);
                set(app.Pobj_d.pS, 'facec', 'flat');                % Set the face color flat
                set(app.Pobj_d.pS, 'FaceColor', app.ms.sS.C);       % Set the color (from file)
                set(app.Pobj_d.pS, 'EdgeColor', 'none');            % Set the edge color
            end
            if app.ee_att == 2
                app.Pobj_d.pG = patch(app.UIAxes,'Faces', app.ms.sG.F, 'Vertices', app.ms.sG.V);
                set(app.Pobj_d.pG, 'facec', 'flat');                % Set the face color flat
                set(app.Pobj_d.pG, 'FaceColor', app.ms.sG.C);       % Set the color (from file)
                set(app.Pobj_d.pG, 'EdgeColor', 'none');            % Set the edge color
            elseif app.ee_att == 3
                app.Pobj_d.pTa = patch(app.UIAxes,'Faces', app.ms.sTa.F, 'Vertices', app.ms.sTa.V);
                set(app.Pobj_d.pTa, 'facec', 'flat');                % Set the face color flat
                set(app.Pobj_d.pTa, 'FaceColor', app.ms.sTa.C);       % Set the color (from file)
                set(app.Pobj_d.pTa, 'EdgeColor', 'none');            % Set the edge color
                app.Pobj_d.pTb = patch(app.UIAxes,'Faces', app.ms.sTb.F, 'Vertices', app.ms.sTb.V);
                set(app.Pobj_d.pTb, 'facec', 'flat');                % Set the face color flat
                set(app.Pobj_d.pTb, 'FaceColor', app.ms.sTb.C);       % Set the color (from file)
                set(app.Pobj_d.pTb, 'EdgeColor', 'none');            % Set the edge color
            end
        end
    end
    app.Pobj_f.p0 = patch(app.UIAxes,'Faces', app.ms.s0.F, 'Vertices', app.ms.s0.V);
    set(app.Pobj_f.p0, 'facec', 'flat');                    % Set the face color flat
    set(app.Pobj_f.p0, 'FaceColor', app.ms.s0.C);           % Set the color (from file)
    set(app.Pobj_f.p0, 'EdgeColor', 'none');                % Set the edge color
    app.Pobj_f.p1 = patch(app.UIAxes,'Faces', app.ms.s1.F, 'Vertices', app.ms.s1.V);
    set(app.Pobj_f.p1, 'facec', 'flat');                    % Set the face color flat
    if app.robot_model == 2
        set(app.Pobj_f.p1, 'FaceVertexCData', app.ms.s1.C); % Set the color (from file)
    else
        set(app.Pobj_f.p1, 'FaceColor', app.ms.s1.C);       % Set the color (from file)
    end
    set(app.Pobj_f.p1, 'EdgeColor', 'none');                % Set the edge color
    app.Pobj_f.p2 = patch(app.UIAxes,'Faces', app.ms.s2.F, 'Vertices', app.ms.s2.V);
    set(app.Pobj_f.p2, 'facec', 'flat');                    % Set the face color flat
    if app.robot_model == 2
        set(app.Pobj_f.p2, 'FaceVertexCData', app.ms.s2.C); % Set the color (from file)
    else
        set(app.Pobj_f.p2, 'FaceColor', app.ms.s2.C);       % Set the color (from file)
    end
    set(app.Pobj_f.p2, 'EdgeColor', 'none');                % Set the edge color
    app.Pobj_f.p3 = patch(app.UIAxes,'Faces', app.ms.s3.F, 'Vertices', app.ms.s3.V);
    set(app.Pobj_f.p3, 'facec', 'flat');                    % Set the face color flat
    if app.robot_model == 2
        set(app.Pobj_f.p3, 'FaceVertexCData', app.ms.s3.C); % Set the color (from file)
    else
        set(app.Pobj_f.p3, 'FaceColor', app.ms.s3.C);       % Set the color (from file)
    end
    set(app.Pobj_f.p3, 'EdgeColor', 'none');                % Set the edge color
    app.Pobj_f.p4 = patch(app.UIAxes,'Faces', app.ms.s4.F, 'Vertices', app.ms.s4.V);
    set(app.Pobj_f.p4, 'facec', 'flat');                    % Set the face color flat
    if app.robot_model == 2
        set(app.Pobj_f.p4, 'FaceVertexCData', app.ms.s4.C); % Set the color (from file)
    else
        set(app.Pobj_f.p4, 'FaceColor', app.ms.s4.C);       % Set the color (from file)
    end
    set(app.Pobj_f.p4, 'EdgeColor', 'none');                % Set the edge color
    app.Pobj_f.p5 = patch(app.UIAxes,'Faces', app.ms.s5.F, 'Vertices', app.ms.s5.V);
    set(app.Pobj_f.p5, 'facec', 'flat');                    % Set the face color flat
    if app.robot_model == 2
        set(app.Pobj_f.p5, 'FaceVertexCData', app.ms.s5.C); % Set the color (from file)
    else
        set(app.Pobj_f.p5, 'FaceColor', app.ms.s5.C);       % Set the color (from file)
    end
    set(app.Pobj_f.p5, 'EdgeColor', 'none');                % Set the edge color
    app.Pobj_f.p6 = patch(app.UIAxes,'Faces', app.ms.s6.F, 'Vertices', app.ms.s6.V);
    set(app.Pobj_f.p6, 'facec', 'flat');                    % Set the face color flat
    if app.robot_model == 2
        set(app.Pobj_f.p6, 'FaceVertexCData', app.ms.s6.C); % Set the color (from file)
    else
        set(app.Pobj_f.p6, 'FaceColor', app.ms.s6.C);       % Set the color (from file)
    end
    set(app.Pobj_f.p6, 'EdgeColor', 'none');                % Set the edge color
    if app.robot_model == 1
        app.Pobj_f.p7 = patch(app.UIAxes,'Faces', app.ms.s7.F, 'Vertices', app.ms.s7.V);
        set(app.Pobj_f.p7, 'facec', 'flat');                % Set the face color flat
        set(app.Pobj_f.p7, 'FaceColor', app.ms.s7.C);       % Set the color (from file)
        set(app.Pobj_f.p7, 'EdgeColor', 'none');            % Set the edge color
    end
    if app.ee_att > 0
        if app.robot_model == 1 || app.robot_model == 3
            app.Pobj_f.pE = patch(app.UIAxes,'Faces', app.ms.sE.F, 'Vertices', app.ms.sE.V);
            set(app.Pobj_f.pE, 'facec', 'flat');                % Set the face color flat
            set(app.Pobj_f.pE, 'FaceColor', app.ms.sE.C);       % Set the color (from file)
            set(app.Pobj_f.pE, 'EdgeColor', 'none');            % Set the edge color
            if app.high_quality == 1 && app.robot_model == 1
                app.Pobj_f.pFl = patch(app.UIAxes,'Faces', app.ms.sFl.F, 'Vertices', app.ms.sFl.V);
                set(app.Pobj_f.pFl, 'facec', 'flat');               % Set the face color flat
                set(app.Pobj_f.pFl, 'FaceColor', app.ms.sFl.C);     % Set the color (from file)
                set(app.Pobj_f.pFl, 'EdgeColor', 'none');           % Set the edge color
                app.Pobj_f.pFr = patch(app.UIAxes,'Faces', app.ms.sFr.F, 'Vertices', app.ms.sFr.V);
                set(app.Pobj_f.pFr, 'facec', 'flat');               % Set the face color flat
                set(app.Pobj_f.pFr, 'FaceColor', app.ms.sFr.C);     % Set the color (from file)
                set(app.Pobj_f.pFr, 'EdgeColor', 'none');           % Set the edge color
            end
            if app.robot_model == 3
                app.Pobj_f.pE2 = patch(app.UIAxes,'Faces', app.ms.sE2.F, 'Vertices', app.ms.sE2.V);
                set(app.Pobj_f.pE2, 'facec', 'flat');               % Set the face color flat
                set(app.Pobj_f.pE2, 'FaceColor', app.ms.sE2.C);     % Set the color (from file)
                set(app.Pobj_f.pE2, 'EdgeColor', 'none');           % Set the edge color
            end
        elseif app.robot_model == 2
            app.Pobj_f.pSC = patch(app.UIAxes,'Faces', app.ms.sSC.F, 'Vertices' ,app.ms.sSC.V);
            set(app.Pobj_f.pSC, 'facec', 'flat');               % Set the face color flat
            set(app.Pobj_f.pSC, 'FaceVertexCData', app.ms.sSC.C);	% Set the face color
            set(app.Pobj_f.pSC, 'EdgeColor', 'none');        	% Set the edge color
            app.Pobj_f.pS = patch(app.UIAxes,'Faces', app.ms.sS.F, 'Vertices' ,app.ms.sS.V);
            set(app.Pobj_f.pS, 'facec', 'flat');               % Set the face color flat
            set(app.Pobj_f.pS, 'FaceVertexCData', app.ms.sS.C);	% Set the face color
            set(app.Pobj_f.pS, 'EdgeColor', 'none');        	% Set the edge color
            app.Pobj_f.pGC = patch(app.UIAxes,'Faces', app.ms.sGC.F, 'Vertices' ,app.ms.sGC.V);
            set(app.Pobj_f.pGC, 'facec', 'flat');               % Set the face color flat
            set(app.Pobj_f.pGC, 'FaceColor', app.ms.sGC.C);	% Set the face color
            set(app.Pobj_f.pGC, 'EdgeColor', 'none');        	% Set the edge color
            app.Pobj_f.pGB = patch(app.UIAxes,'Faces', app.ms.sGB.F, 'Vertices' ,app.ms.sGB.V);
            set(app.Pobj_f.pGB, 'facec', 'flat');               % Set the face color flat
            set(app.Pobj_f.pGB, 'FaceVertexCData', app.ms.sGB.C);	% Set the face color
            set(app.Pobj_f.pGB, 'EdgeColor', 'none');        	% Set the edge color
            app.Pobj_f.pOKa = patch(app.UIAxes,'Faces', app.ms.sOK.F, 'Vertices' ,app.ms.sOK.V);
            set(app.Pobj_f.pOKa, 'facec', 'flat');               % Set the face color flat
            set(app.Pobj_f.pOKa, 'FaceColor', app.ms.sOK.C);	% Set the face color
            set(app.Pobj_f.pOKa, 'EdgeColor', 'none');        	% Set the edge color
            app.Pobj_f.pOKb = patch(app.UIAxes,'Faces', app.ms.sOK.F, 'Vertices' ,app.ms.sOK.V);
            set(app.Pobj_f.pOKb, 'facec', 'flat');               % Set the face color flat
            set(app.Pobj_f.pOKb, 'FaceColor', app.ms.sOK.C);	% Set the face color
            set(app.Pobj_f.pOKb, 'EdgeColor', 'none');        	% Set the edge color
            app.Pobj_f.pOFa = patch(app.UIAxes,'Faces', app.ms.sOF.F, 'Vertices' ,app.ms.sOF.V);
            set(app.Pobj_f.pOFa, 'facec', 'flat');               % Set the face color flat
            set(app.Pobj_f.pOFa, 'FaceColor', app.ms.sOF.C);	% Set the face color
            set(app.Pobj_f.pOFa, 'EdgeColor', 'none');        	% Set the edge color
            app.Pobj_f.pOFb = patch(app.UIAxes,'Faces', app.ms.sOF.F, 'Vertices' ,app.ms.sOF.V);
            set(app.Pobj_f.pOFb, 'facec', 'flat');               % Set the face color flat
            set(app.Pobj_f.pOFb, 'FaceColor', app.ms.sOF.C);	% Set the face color
            set(app.Pobj_f.pOFb, 'EdgeColor', 'none');        	% Set the edge color
            app.Pobj_f.pIKa = patch(app.UIAxes,'Faces', app.ms.sIK.F, 'Vertices' ,app.ms.sIK.V);
            set(app.Pobj_f.pIKa, 'facec', 'flat');               % Set the face color flat
            set(app.Pobj_f.pIKa, 'FaceColor', app.ms.sIK.C);	% Set the face color
            set(app.Pobj_f.pIKa, 'EdgeColor', 'none');        	% Set the edge color
            app.Pobj_f.pIKb = patch(app.UIAxes,'Faces', app.ms.sIK.F, 'Vertices' ,app.ms.sIK.V);
            set(app.Pobj_f.pIKb, 'facec', 'flat');               % Set the face color flat
            set(app.Pobj_f.pIKb, 'FaceColor', app.ms.sIK.C);	% Set the face color
            set(app.Pobj_f.pIKb, 'EdgeColor', 'none');        	% Set the edge color
            app.Pobj_f.pIFa = patch(app.UIAxes,'Faces', app.ms.sIF.F, 'Vertices' ,app.ms.sIF.V);
            set(app.Pobj_f.pIFa, 'facec', 'flat');               % Set the face color flat
            set(app.Pobj_f.pIFa, 'FaceVertexCData', app.ms.sIF.C);	% Set the face color
            set(app.Pobj_f.pIFa, 'EdgeColor', 'none');        	% Set the edge color
            app.Pobj_f.pIFb = patch(app.UIAxes,'Faces', app.ms.sIF.F, 'Vertices' ,app.ms.sIF.V);
            set(app.Pobj_f.pIFb, 'facec', 'flat');               % Set the face color flat
            set(app.Pobj_f.pIFb, 'FaceVertexCData', app.ms.sIF.C);	% Set the face color
            set(app.Pobj_f.pIFb, 'EdgeColor', 'none');        	% Set the edge color
        else
            app.Pobj_f.pS = patch(app.UIAxes,'Faces', app.ms.sS.F, 'Vertices', app.ms.sS.V);
            set(app.Pobj_f.pS, 'facec', 'flat');                % Set the face color flat
            set(app.Pobj_f.pS, 'FaceColor', app.ms.sS.C);       % Set the color (from file)
            set(app.Pobj_f.pS, 'EdgeColor', 'none');            % Set the edge color
        end
        if app.ee_att == 2
            app.Pobj_f.pG = patch(app.UIAxes,'Faces', app.ms.sG.F, 'Vertices', app.ms.sG.V);
            set(app.Pobj_f.pG, 'facec', 'flat');                % Set the face color flat
            set(app.Pobj_f.pG, 'FaceColor', app.ms.sG.C);       % Set the color (from file)
            set(app.Pobj_f.pG, 'EdgeColor', 'none');            % Set the edge color
        elseif app.ee_att == 3
            app.Pobj_f.pTa = patch(app.UIAxes,'Faces', app.ms.sTa.F, 'Vertices', app.ms.sTa.V);
            set(app.Pobj_f.pTa, 'facec', 'flat');                % Set the face color flat
            set(app.Pobj_f.pTa, 'FaceColor', app.ms.sTa.C);       % Set the color (from file)
            set(app.Pobj_f.pTa, 'EdgeColor', 'none');            % Set the edge color
            app.Pobj_f.pTb = patch(app.UIAxes,'Faces', app.ms.sTb.F, 'Vertices', app.ms.sTb.V);
            set(app.Pobj_f.pTb, 'facec', 'flat');                % Set the face color flat
            set(app.Pobj_f.pTb, 'FaceColor', app.ms.sTb.C);       % Set the color (from file)
            set(app.Pobj_f.pTb, 'EdgeColor', 'none');            % Set the edge color
        end
    end

    if app.coord_frame_on == 1
        app.Pobj_d.pF = patch(app.UIAxes,'Faces', app.ms.sF.F, 'Vertices', app.ms.sF.V);
        set(app.Pobj_d.pF, 'facec', 'flat');                % Set the face color flat
        set(app.Pobj_d.pF, 'FaceVertexCData', app.ms.sF.C);	% Set the color (from file)
        set(app.Pobj_d.pF, 'EdgeColor', 'none');            % Set the edge color
    end
    if app.coord_frame_on == 1
        app.Pobj_f.pF = patch(app.UIAxes,'Faces', app.ms.sF.F, 'Vertices', app.ms.sF.V);
        set(app.Pobj_f.pF, 'facec', 'flat');                % Set the face color flat
        set(app.Pobj_f.pF, 'FaceVertexCData', app.ms.sF.C); % Set the color (from file)
        set(app.Pobj_f.pF, 'EdgeColor', 'none');            % Set the edge color
    end
    
end
