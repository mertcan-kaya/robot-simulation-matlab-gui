classdef RobotRenderer < handle
    properties
        AxesHandle
        ms
        Pobj_d
        Pobj_f
        Pobj_r
        LineHandles = struct()
        EllipsoidSurface
        EllipsoidAxisLines
    end
    
    methods
        function obj = RobotRenderer(axesHandle)
            obj.AxesHandle = axesHandle;
        end

        function p = createMeshPatch(obj, msData, role, isVertexColor)
            if nargin < 4
                isVertexColor = (size(msData.C, 1) > 1);
            end
            
            p = patch(obj.AxesHandle, 'Faces', msData.F, 'Vertices', msData.V);
            
            if isVertexColor
                set(p, 'FaceVertexCData', msData.C, 'FaceColor', 'interp');
            else
                set(p, 'FaceColor', msData.C);
            end
            set(p, 'EdgeColor', 'none');
            set(p, 'FaceLighting', 'gouraud');
            set(p, 'BackFaceLighting', 'reverselit');

            switch role
                case 'actual'
                    set(p, 'AmbientStrength', 0.5, ...
                           'DiffuseStrength', 0.8, ...
                           'SpecularStrength', 0.35, ...
                           'SpecularExponent', 20, ...
                           'FaceAlpha', 1.0);
                case 'ghost'
                    set(p, 'AmbientStrength', 0.7, ...
                           'DiffuseStrength', 0.5, ...
                           'SpecularStrength', 0.1, ...
                           'SpecularExponent', 10, ...
                           'FaceAlpha', 0.3);
                case 'target'
                    set(p, 'AmbientStrength', 0.6, ...
                           'DiffuseStrength', 0.6, ...
                           'SpecularStrength', 0.2, ...
                           'SpecularExponent', 15, ...
                           'FaceAlpha', 0.4);
            end
        end

        
function loadMeshes(obj, robot_model, high_quality, ee_att, coord_frame_on, ghost_on, line_on, task_mode, running_flag, trj_profile)

    obj.LineHandles = struct();
    cla(obj.AxesHandle);
    if line_on == 0
        if robot_model == 0
            return;
        end
        delete(findobj(obj.AxesHandle, 'Type', 'light'));
        % Studio Dual-Point Lighting: Key Light (warm-white) + Fill Light (cool-ambient)
        light(obj.AxesHandle, 'Position', [-1, -0.5, 2], 'Color', [1.0, 0.98, 0.95], 'Style', 'infinite');
        light(obj.AxesHandle, 'Position', [1.5, 1.0, 0.5], 'Color', [0.65, 0.7, 0.8], 'Style', 'infinite');
        
        % Load CAD files
        meshSub = 'collision';
        if high_quality == 1
            meshSub = 'visual';
        end

        if robot_model == 5 || robot_model == 4
            obj.ms.s0 = robotics.graphics.MeshCache.getMesh(['meshes\rx160\' meshSub '\base_link.mat']);
            obj.ms.s1 = robotics.graphics.MeshCache.getMesh(['meshes\rx160\' meshSub '\link_1.mat']);
            obj.ms.s2 = robotics.graphics.MeshCache.getMesh(['meshes\rx160\' meshSub '\link_2.mat']);
            obj.ms.s3 = robotics.graphics.MeshCache.getMesh(['meshes\rx160\' meshSub '\link_3.mat']);
            if robot_model == 5
                obj.ms.s4 = robotics.graphics.MeshCache.getMesh(['meshes\rx160\' meshSub '\link_4l.mat']);
            else
                obj.ms.s4 = robotics.graphics.MeshCache.getMesh(['meshes\rx160\' meshSub '\link_4.mat']);
            end
            obj.ms.s5 = robotics.graphics.MeshCache.getMesh(['meshes\rx160\' meshSub '\link_5.mat']);
            obj.ms.s6 = robotics.graphics.MeshCache.getMesh(['meshes\rx160\' meshSub '\link_6.mat']);
            if ee_att > 0
                obj.ms.sS = robotics.graphics.MeshCache.getMesh(['meshes\rx160\' meshSub '\ati_delta.mat']);
            end
            if ee_att == 2
                obj.ms.sG = robotics.graphics.MeshCache.getMesh(['meshes\rx160\' meshSub '\gripper.mat']);
            end
            if ee_att == 3
                obj.ms.sTa = robotics.graphics.MeshCache.getMesh('meshes\rx160\adaptor_holder.mat');
                obj.ms.sTb = robotics.graphics.MeshCache.getMesh('meshes\rx160\rod_sphere.mat');
            end
        elseif robot_model == 3
            obj.ms.s0 = robotics.graphics.MeshCache.getMesh('meshes\z1\z1_Link00.mat');
            obj.ms.s1 = robotics.graphics.MeshCache.getMesh('meshes\z1\z1_Link01.mat');
            obj.ms.s2 = robotics.graphics.MeshCache.getMesh('meshes\z1\z1_Link02.mat');
            obj.ms.s3 = robotics.graphics.MeshCache.getMesh('meshes\z1\z1_Link03.mat');
            obj.ms.s4 = robotics.graphics.MeshCache.getMesh('meshes\z1\z1_Link04.mat');
            obj.ms.s5 = robotics.graphics.MeshCache.getMesh('meshes\z1\z1_Link05.mat');
            obj.ms.s6 = robotics.graphics.MeshCache.getMesh('meshes\z1\z1_Link06.mat');
            if ee_att == 1
                obj.ms.sE = robotics.graphics.MeshCache.getMesh('meshes\z1\z1_GripperStator.mat');
                obj.ms.sE2 = robotics.graphics.MeshCache.getMesh('meshes\z1\z1_GripperMover.mat');
            end
        elseif robot_model == 2
            obj.ms.s0 = robotics.graphics.MeshCache.getMesh('meshes\ur3\base.mat');
            obj.ms.s1 = robotics.graphics.MeshCache.getMesh('meshes\ur3\shoulder.mat');
            obj.ms.s2 = robotics.graphics.MeshCache.getMesh('meshes\ur3\upperarm.mat');
            obj.ms.s3 = robotics.graphics.MeshCache.getMesh('meshes\ur3\forearm.mat');
            obj.ms.s4 = robotics.graphics.MeshCache.getMesh('meshes\ur3\wrist1.mat');
            obj.ms.s5 = robotics.graphics.MeshCache.getMesh('meshes\ur3\wrist2.mat');
            obj.ms.s6 = robotics.graphics.MeshCache.getMesh('meshes\ur3\wrist3.mat');
            if ee_att == 1
                obj.ms.sSC = robotics.graphics.MeshCache.getMesh('meshes\ur3\fts150_coupling.mat');
                obj.ms.sS = robotics.graphics.MeshCache.getMesh('meshes\ur3\fts150.mat');
                obj.ms.sGC = robotics.graphics.MeshCache.getMesh('meshes\ur3\gripper_coupling.mat');
                obj.ms.sGB = robotics.graphics.MeshCache.getMesh('meshes\ur3\arg2f_85_base_link_simp.mat');
                obj.ms.sOK = robotics.graphics.MeshCache.getMesh('meshes\ur3\arg2f_85_outer_knuckle_simp.mat');
                obj.ms.sOF = robotics.graphics.MeshCache.getMesh('meshes\ur3\arg2f_85_outer_finger_simp.mat');
                obj.ms.sIK = robotics.graphics.MeshCache.getMesh('meshes\ur3\arg2f_85_inner_knuckle_simp.mat');
                obj.ms.sIF = robotics.graphics.MeshCache.getMesh('meshes\ur3\arg2f_85_inner_finger_pad_simp.mat');
            end
        else
            obj.ms.s0 = robotics.graphics.MeshCache.getMesh(['meshes\fer\' meshSub '\link0.mat']);
            obj.ms.s1 = robotics.graphics.MeshCache.getMesh(['meshes\fer\' meshSub '\link1.mat']);
            obj.ms.s2 = robotics.graphics.MeshCache.getMesh(['meshes\fer\' meshSub '\link2.mat']);
            obj.ms.s3 = robotics.graphics.MeshCache.getMesh(['meshes\fer\' meshSub '\link3.mat']);
            obj.ms.s4 = robotics.graphics.MeshCache.getMesh(['meshes\fer\' meshSub '\link4.mat']);
            obj.ms.s5 = robotics.graphics.MeshCache.getMesh(['meshes\fer\' meshSub '\link5.mat']);
            obj.ms.s6 = robotics.graphics.MeshCache.getMesh(['meshes\fer\' meshSub '\link6.mat']);
            obj.ms.s7 = robotics.graphics.MeshCache.getMesh(['meshes\fer\' meshSub '\link7.mat']);
            if ee_att == 1
                obj.ms.sE = robotics.graphics.MeshCache.getMesh(['meshes\fer\' meshSub '\hand.mat']);
                obj.ms.sFl = robotics.graphics.MeshCache.getMesh(['meshes\fer\' meshSub '\finger.mat']);
                obj.ms.sFr = robotics.graphics.MeshCache.getMesh(['meshes\fer\' meshSub '\finger.mat']);
            end
        end
        
        if coord_frame_on == 1
            obj.ms.sF = robotics.graphics.MeshCache.getMesh('meshes\coord_frame.mat');
        end
    
        clear temp
        
        % CAD properties
        if ghost_on == 1
            obj.Pobj_d.p1 = patch(obj.AxesHandle,'Faces', obj.ms.s1.F, 'Vertices', obj.ms.s1.V);
            set(obj.Pobj_d.p1, 'facec', 'flat');                    % Set the face color flat
            if robot_model == 2
                set(obj.Pobj_d.p1, 'FaceVertexCData', obj.ms.s1.C); % Set the color (from file)
            else
                set(obj.Pobj_d.p1, 'FaceColor', obj.ms.s1.C);       % Set the color (from file)
            end
            set(obj.Pobj_d.p1, 'EdgeColor', 'none');                % Set the edge color
            obj.Pobj_d.p2 = patch(obj.AxesHandle,'Faces', obj.ms.s2.F, 'Vertices', obj.ms.s2.V);
            set(obj.Pobj_d.p2, 'facec', 'flat');                    % Set the face color flat
            if robot_model == 2
                set(obj.Pobj_d.p2, 'FaceVertexCData', obj.ms.s2.C); % Set the color (from file)
            else
                set(obj.Pobj_d.p2, 'FaceColor', obj.ms.s2.C);       % Set the color (from file)
            end
            set(obj.Pobj_d.p2, 'EdgeColor', 'none');                % Set the edge color
            obj.Pobj_d.p3 = patch(obj.AxesHandle,'Faces', obj.ms.s3.F, 'Vertices', obj.ms.s3.V);
            set(obj.Pobj_d.p3, 'facec', 'flat');                    % Set the face color flat
            if robot_model == 2
                set(obj.Pobj_d.p3, 'FaceVertexCData', obj.ms.s3.C); % Set the color (from file)
            else
                set(obj.Pobj_d.p3, 'FaceColor', obj.ms.s3.C);       % Set the color (from file)
            end
            set(obj.Pobj_d.p3, 'EdgeColor', 'none');                % Set the edge color
            obj.Pobj_d.p4 = patch(obj.AxesHandle,'Faces', obj.ms.s4.F, 'Vertices', obj.ms.s4.V);
            set(obj.Pobj_d.p4, 'facec', 'flat');                    % Set the face color flat
            if robot_model == 2
                set(obj.Pobj_d.p4, 'FaceVertexCData', obj.ms.s4.C); % Set the color (from file)
            else
                set(obj.Pobj_d.p4, 'FaceColor', obj.ms.s4.C);       % Set the color (from file)
            end
            set(obj.Pobj_d.p4, 'EdgeColor', 'none');                % Set the edge color
            obj.Pobj_d.p5 = patch(obj.AxesHandle,'Faces', obj.ms.s5.F, 'Vertices', obj.ms.s5.V);
            set(obj.Pobj_d.p5, 'facec', 'flat');                    % Set the face color flat
            if robot_model == 2
                set(obj.Pobj_d.p5, 'FaceVertexCData', obj.ms.s5.C); % Set the color (from file)
            else
                set(obj.Pobj_d.p5, 'FaceColor', obj.ms.s5.C);       % Set the color (from file)
            end
            set(obj.Pobj_d.p5, 'EdgeColor', 'none');                % Set the edge color
            obj.Pobj_d.p6 = patch(obj.AxesHandle,'Faces', obj.ms.s6.F, 'Vertices', obj.ms.s6.V);
            set(obj.Pobj_d.p6, 'facec', 'flat');                    % Set the face color flat
            if robot_model == 2
                set(obj.Pobj_d.p6, 'FaceVertexCData', obj.ms.s6.C); % Set the color (from file)
            else
                set(obj.Pobj_d.p6, 'FaceColor', obj.ms.s6.C);       % Set the color (from file)
            end
            set(obj.Pobj_d.p6, 'EdgeColor', 'none');                % Set the edge color
            if robot_model == 1
                obj.Pobj_d.p7 = patch(obj.AxesHandle,'Faces', obj.ms.s7.F, 'Vertices', obj.ms.s7.V);
                set(obj.Pobj_d.p7, 'facec', 'flat');                % Set the face color flat
                set(obj.Pobj_d.p7, 'FaceColor', obj.ms.s7.C);       % Set the color (from file)
                set(obj.Pobj_d.p7, 'EdgeColor', 'none');            % Set the edge color
            end
            if ee_att > 0
                if robot_model == 1 || robot_model == 3
                    obj.Pobj_d.pE = patch(obj.AxesHandle,'Faces', obj.ms.sE.F, 'Vertices', obj.ms.sE.V);
                    set(obj.Pobj_d.pE, 'facec', 'flat');                % Set the face color flat
                    set(obj.Pobj_d.pE, 'FaceColor', obj.ms.sE.C);       % Set the color (from file)
                    set(obj.Pobj_d.pE, 'EdgeColor', 'none');            % Set the edge color
                    if robot_model == 1
                        obj.Pobj_d.pFl = patch(obj.AxesHandle,'Faces', obj.ms.sFl.F, 'Vertices', obj.ms.sFl.V);
                        set(obj.Pobj_d.pFl, 'facec', 'flat');                % Set the face color flat
                        set(obj.Pobj_d.pFl, 'FaceColor', obj.ms.sFl.C);       % Set the color (from file)
                        set(obj.Pobj_d.pFl, 'EdgeColor', 'none');            % Set the edge color
                        obj.Pobj_d.pFr = patch(obj.AxesHandle,'Faces', obj.ms.sFr.F, 'Vertices', obj.ms.sFr.V);
                        set(obj.Pobj_d.pFr, 'facec', 'flat');                % Set the face color flat
                        set(obj.Pobj_d.pFr, 'FaceColor', obj.ms.sFr.C);       % Set the color (from file)
                        set(obj.Pobj_d.pFr, 'EdgeColor', 'none');            % Set the edge color
                    end
                    if robot_model == 3
                        obj.Pobj_d.pE2 = patch(obj.AxesHandle,'Faces', obj.ms.sE2.F, 'Vertices', obj.ms.sE2.V);
                        set(obj.Pobj_d.pE2, 'facec', 'flat');                % Set the face color flat
                        set(obj.Pobj_d.pE2, 'FaceColor', obj.ms.sE2.C);       % Set the color (from file)
                        set(obj.Pobj_d.pE2, 'EdgeColor', 'none');            % Set the edge color
                    end
                elseif robot_model == 2
                    obj.Pobj_d.pSC = patch(obj.AxesHandle,'Faces', obj.ms.sSC.F, 'Vertices' ,obj.ms.sSC.V);
                    set(obj.Pobj_d.pSC, 'facec', 'flat');               % Set the face color flat
                    set(obj.Pobj_d.pSC, 'FaceVertexCData', obj.ms.sSC.C);	% Set the face color
                    set(obj.Pobj_d.pSC, 'EdgeColor', 'none');        	% Set the edge color
                    obj.Pobj_d.pS = patch(obj.AxesHandle,'Faces', obj.ms.sS.F, 'Vertices' ,obj.ms.sS.V);
                    set(obj.Pobj_d.pS, 'facec', 'flat');               % Set the face color flat
                    set(obj.Pobj_d.pS, 'FaceVertexCData', obj.ms.sS.C);	% Set the face color
                    set(obj.Pobj_d.pS, 'EdgeColor', 'none');        	% Set the edge color
                    obj.Pobj_d.pGC = patch(obj.AxesHandle,'Faces', obj.ms.sGC.F, 'Vertices' ,obj.ms.sGC.V);
                    set(obj.Pobj_d.pGC, 'facec', 'flat');               % Set the face color flat
                    set(obj.Pobj_d.pGC, 'FaceColor', obj.ms.sGC.C);	% Set the face color
                    set(obj.Pobj_d.pGC, 'EdgeColor', 'none');        	% Set the edge color
                    obj.Pobj_d.pGB = patch(obj.AxesHandle,'Faces', obj.ms.sGB.F, 'Vertices' ,obj.ms.sGB.V);
                    set(obj.Pobj_d.pGB, 'facec', 'flat');               % Set the face color flat
                    set(obj.Pobj_d.pGB, 'FaceVertexCData', obj.ms.sGB.C);	% Set the face color
                    set(obj.Pobj_d.pGB, 'EdgeColor', 'none');        	% Set the edge color
                    obj.Pobj_d.pOKa = patch(obj.AxesHandle,'Faces', obj.ms.sOK.F, 'Vertices' ,obj.ms.sOK.V);
                    set(obj.Pobj_d.pOKa, 'facec', 'flat');               % Set the face color flat
                    set(obj.Pobj_d.pOKa, 'FaceColor', obj.ms.sOK.C);	% Set the face color
                    set(obj.Pobj_d.pOKa, 'EdgeColor', 'none');        	% Set the edge color
                    obj.Pobj_d.pOKb = patch(obj.AxesHandle,'Faces', obj.ms.sOK.F, 'Vertices' ,obj.ms.sOK.V);
                    set(obj.Pobj_d.pOKb, 'facec', 'flat');               % Set the face color flat
                    set(obj.Pobj_d.pOKb, 'FaceColor', obj.ms.sOK.C);	% Set the face color
                    set(obj.Pobj_d.pOKb, 'EdgeColor', 'none');        	% Set the edge color
                    obj.Pobj_d.pOFa = patch(obj.AxesHandle,'Faces', obj.ms.sOF.F, 'Vertices' ,obj.ms.sOF.V);
                    set(obj.Pobj_d.pOFa, 'facec', 'flat');               % Set the face color flat
                    set(obj.Pobj_d.pOFa, 'FaceColor', obj.ms.sOF.C);	% Set the face color
                    set(obj.Pobj_d.pOFa, 'EdgeColor', 'none');        	% Set the edge color
                    obj.Pobj_d.pOFb = patch(obj.AxesHandle,'Faces', obj.ms.sOF.F, 'Vertices' ,obj.ms.sOF.V);
                    set(obj.Pobj_d.pOFb, 'facec', 'flat');               % Set the face color flat
                    set(obj.Pobj_d.pOFb, 'FaceColor', obj.ms.sOF.C);	% Set the face color
                    set(obj.Pobj_d.pOFb, 'EdgeColor', 'none');        	% Set the edge color
                    obj.Pobj_d.pIKa = patch(obj.AxesHandle,'Faces', obj.ms.sIK.F, 'Vertices' ,obj.ms.sIK.V);
                    set(obj.Pobj_d.pIKa, 'facec', 'flat');               % Set the face color flat
                    set(obj.Pobj_d.pIKa, 'FaceColor', obj.ms.sIK.C);	% Set the face color
                    set(obj.Pobj_d.pIKa, 'EdgeColor', 'none');        	% Set the edge color
                    obj.Pobj_d.pIKb = patch(obj.AxesHandle,'Faces', obj.ms.sIK.F, 'Vertices' ,obj.ms.sIK.V);
                    set(obj.Pobj_d.pIKb, 'facec', 'flat');               % Set the face color flat
                    set(obj.Pobj_d.pIKb, 'FaceColor', obj.ms.sIK.C);	% Set the face color
                    set(obj.Pobj_d.pIKb, 'EdgeColor', 'none');        	% Set the edge color
                    obj.Pobj_d.pIFa = patch(obj.AxesHandle,'Faces', obj.ms.sIF.F, 'Vertices' ,obj.ms.sIF.V);
                    set(obj.Pobj_d.pIFa, 'facec', 'flat');               % Set the face color flat
                    set(obj.Pobj_d.pIFa, 'FaceVertexCData', obj.ms.sIF.C);	% Set the face color
                    set(obj.Pobj_d.pIFa, 'EdgeColor', 'none');        	% Set the edge color
                    obj.Pobj_d.pIFb = patch(obj.AxesHandle,'Faces', obj.ms.sIF.F, 'Vertices' ,obj.ms.sIF.V);
                    set(obj.Pobj_d.pIFb, 'facec', 'flat');               % Set the face color flat
                    set(obj.Pobj_d.pIFb, 'FaceVertexCData', obj.ms.sIF.C);	% Set the face color
                    set(obj.Pobj_d.pIFb, 'EdgeColor', 'none');        	% Set the edge color
                else
                    obj.Pobj_d.pS = patch(obj.AxesHandle,'Faces', obj.ms.sS.F, 'Vertices', obj.ms.sS.V);
                    set(obj.Pobj_d.pS, 'facec', 'flat');                % Set the face color flat
                    set(obj.Pobj_d.pS, 'FaceColor', obj.ms.sS.C);       % Set the color (from file)
                    set(obj.Pobj_d.pS, 'EdgeColor', 'none');            % Set the edge color
                end
                if ee_att == 2
                    obj.Pobj_d.pG = patch(obj.AxesHandle,'Faces', obj.ms.sG.F, 'Vertices', obj.ms.sG.V);
                    set(obj.Pobj_d.pG, 'facec', 'flat');                % Set the face color flat
                    set(obj.Pobj_d.pG, 'FaceColor', obj.ms.sG.C);       % Set the color (from file)
                    set(obj.Pobj_d.pG, 'EdgeColor', 'none');            % Set the edge color
                elseif ee_att == 3
                    obj.Pobj_d.pTa = patch(obj.AxesHandle,'Faces', obj.ms.sTa.F, 'Vertices', obj.ms.sTa.V);
                    set(obj.Pobj_d.pTa, 'facec', 'flat');                % Set the face color flat
                    set(obj.Pobj_d.pTa, 'FaceColor', obj.ms.sTa.C);       % Set the color (from file)
                    set(obj.Pobj_d.pTa, 'EdgeColor', 'none');            % Set the edge color
                    obj.Pobj_d.pTb = patch(obj.AxesHandle,'Faces', obj.ms.sTb.F, 'Vertices', obj.ms.sTb.V);
                    set(obj.Pobj_d.pTb, 'facec', 'flat');                % Set the face color flat
                    set(obj.Pobj_d.pTb, 'FaceColor', obj.ms.sTb.C);       % Set the color (from file)
                    set(obj.Pobj_d.pTb, 'EdgeColor', 'none');            % Set the edge color
                end
            end
        end
        obj.Pobj_f.p0 = patch(obj.AxesHandle,'Faces', obj.ms.s0.F, 'Vertices', obj.ms.s0.V);
        set(obj.Pobj_f.p0, 'facec', 'flat');                    % Set the face color flat
        set(obj.Pobj_f.p0, 'FaceColor', obj.ms.s0.C);           % Set the color (from file)
        set(obj.Pobj_f.p0, 'EdgeColor', 'none');                % Set the edge color
        obj.Pobj_f.p1 = patch(obj.AxesHandle,'Faces', obj.ms.s1.F, 'Vertices', obj.ms.s1.V);
        set(obj.Pobj_f.p1, 'facec', 'flat');                    % Set the face color flat
        if robot_model == 2
            set(obj.Pobj_f.p1, 'FaceVertexCData', obj.ms.s1.C); % Set the color (from file)
        else
            set(obj.Pobj_f.p1, 'FaceColor', obj.ms.s1.C);       % Set the color (from file)
        end
        set(obj.Pobj_f.p1, 'EdgeColor', 'none');                % Set the edge color
        obj.Pobj_f.p2 = patch(obj.AxesHandle,'Faces', obj.ms.s2.F, 'Vertices', obj.ms.s2.V);
        set(obj.Pobj_f.p2, 'facec', 'flat');                    % Set the face color flat
        if robot_model == 2
            set(obj.Pobj_f.p2, 'FaceVertexCData', obj.ms.s2.C); % Set the color (from file)
        else
            set(obj.Pobj_f.p2, 'FaceColor', obj.ms.s2.C);       % Set the color (from file)
        end
        set(obj.Pobj_f.p2, 'EdgeColor', 'none');                % Set the edge color
        obj.Pobj_f.p3 = patch(obj.AxesHandle,'Faces', obj.ms.s3.F, 'Vertices', obj.ms.s3.V);
        set(obj.Pobj_f.p3, 'facec', 'flat');                    % Set the face color flat
        if robot_model == 2
            set(obj.Pobj_f.p3, 'FaceVertexCData', obj.ms.s3.C); % Set the color (from file)
        else
            set(obj.Pobj_f.p3, 'FaceColor', obj.ms.s3.C);       % Set the color (from file)
        end
        set(obj.Pobj_f.p3, 'EdgeColor', 'none');                % Set the edge color
        obj.Pobj_f.p4 = patch(obj.AxesHandle,'Faces', obj.ms.s4.F, 'Vertices', obj.ms.s4.V);
        set(obj.Pobj_f.p4, 'facec', 'flat');                    % Set the face color flat
        if robot_model == 2
            set(obj.Pobj_f.p4, 'FaceVertexCData', obj.ms.s4.C); % Set the color (from file)
        else
            set(obj.Pobj_f.p4, 'FaceColor', obj.ms.s4.C);       % Set the color (from file)
        end
        set(obj.Pobj_f.p4, 'EdgeColor', 'none');                % Set the edge color
        obj.Pobj_f.p5 = patch(obj.AxesHandle,'Faces', obj.ms.s5.F, 'Vertices', obj.ms.s5.V);
        set(obj.Pobj_f.p5, 'facec', 'flat');                    % Set the face color flat
        if robot_model == 2
            set(obj.Pobj_f.p5, 'FaceVertexCData', obj.ms.s5.C); % Set the color (from file)
        else
            set(obj.Pobj_f.p5, 'FaceColor', obj.ms.s5.C);       % Set the color (from file)
        end
        set(obj.Pobj_f.p5, 'EdgeColor', 'none');                % Set the edge color
        obj.Pobj_f.p6 = patch(obj.AxesHandle,'Faces', obj.ms.s6.F, 'Vertices', obj.ms.s6.V);
        set(obj.Pobj_f.p6, 'facec', 'flat');                    % Set the face color flat
        if robot_model == 2
            set(obj.Pobj_f.p6, 'FaceVertexCData', obj.ms.s6.C); % Set the color (from file)
        else
            set(obj.Pobj_f.p6, 'FaceColor', obj.ms.s6.C);       % Set the color (from file)
        end
        set(obj.Pobj_f.p6, 'EdgeColor', 'none');                % Set the edge color
        if robot_model == 1
            obj.Pobj_f.p7 = patch(obj.AxesHandle,'Faces', obj.ms.s7.F, 'Vertices', obj.ms.s7.V);
            set(obj.Pobj_f.p7, 'facec', 'flat');                % Set the face color flat
            set(obj.Pobj_f.p7, 'FaceColor', obj.ms.s7.C);       % Set the color (from file)
            set(obj.Pobj_f.p7, 'EdgeColor', 'none');            % Set the edge color
        end
        if ee_att > 0
            if robot_model == 1 || robot_model == 3
                obj.Pobj_f.pE = patch(obj.AxesHandle,'Faces', obj.ms.sE.F, 'Vertices', obj.ms.sE.V);
                set(obj.Pobj_f.pE, 'facec', 'flat');                % Set the face color flat
                set(obj.Pobj_f.pE, 'FaceColor', obj.ms.sE.C);       % Set the color (from file)
                set(obj.Pobj_f.pE, 'EdgeColor', 'none');            % Set the edge color
                if robot_model == 1
                    obj.Pobj_f.pFl = patch(obj.AxesHandle,'Faces', obj.ms.sFl.F, 'Vertices', obj.ms.sFl.V);
                    set(obj.Pobj_f.pFl, 'facec', 'flat');               % Set the face color flat
                    set(obj.Pobj_f.pFl, 'FaceColor', obj.ms.sFl.C);     % Set the color (from file)
                    set(obj.Pobj_f.pFl, 'EdgeColor', 'none');           % Set the edge color
                    obj.Pobj_f.pFr = patch(obj.AxesHandle,'Faces', obj.ms.sFr.F, 'Vertices', obj.ms.sFr.V);
                    set(obj.Pobj_f.pFr, 'facec', 'flat');               % Set the face color flat
                    set(obj.Pobj_f.pFr, 'FaceColor', obj.ms.sFr.C);     % Set the color (from file)
                    set(obj.Pobj_f.pFr, 'EdgeColor', 'none');           % Set the edge color
                end
                if robot_model == 3
                    obj.Pobj_f.pE2 = patch(obj.AxesHandle,'Faces', obj.ms.sE2.F, 'Vertices', obj.ms.sE2.V);
                    set(obj.Pobj_f.pE2, 'facec', 'flat');               % Set the face color flat
                    set(obj.Pobj_f.pE2, 'FaceColor', obj.ms.sE2.C);     % Set the color (from file)
                    set(obj.Pobj_f.pE2, 'EdgeColor', 'none');           % Set the edge color
                end
            elseif robot_model == 2
                obj.Pobj_f.pSC = patch(obj.AxesHandle,'Faces', obj.ms.sSC.F, 'Vertices' ,obj.ms.sSC.V);
                set(obj.Pobj_f.pSC, 'facec', 'flat');               % Set the face color flat
                set(obj.Pobj_f.pSC, 'FaceVertexCData', obj.ms.sSC.C);	% Set the face color
                set(obj.Pobj_f.pSC, 'EdgeColor', 'none');        	% Set the edge color
                obj.Pobj_f.pS = patch(obj.AxesHandle,'Faces', obj.ms.sS.F, 'Vertices' ,obj.ms.sS.V);
                set(obj.Pobj_f.pS, 'facec', 'flat');               % Set the face color flat
                set(obj.Pobj_f.pS, 'FaceVertexCData', obj.ms.sS.C);	% Set the face color
                set(obj.Pobj_f.pS, 'EdgeColor', 'none');        	% Set the edge color
                obj.Pobj_f.pGC = patch(obj.AxesHandle,'Faces', obj.ms.sGC.F, 'Vertices' ,obj.ms.sGC.V);
                set(obj.Pobj_f.pGC, 'facec', 'flat');               % Set the face color flat
                set(obj.Pobj_f.pGC, 'FaceColor', obj.ms.sGC.C);	% Set the face color
                set(obj.Pobj_f.pGC, 'EdgeColor', 'none');        	% Set the edge color
                obj.Pobj_f.pGB = patch(obj.AxesHandle,'Faces', obj.ms.sGB.F, 'Vertices' ,obj.ms.sGB.V);
                set(obj.Pobj_f.pGB, 'facec', 'flat');               % Set the face color flat
                set(obj.Pobj_f.pGB, 'FaceVertexCData', obj.ms.sGB.C);	% Set the face color
                set(obj.Pobj_f.pGB, 'EdgeColor', 'none');        	% Set the edge color
                obj.Pobj_f.pOKa = patch(obj.AxesHandle,'Faces', obj.ms.sOK.F, 'Vertices' ,obj.ms.sOK.V);
                set(obj.Pobj_f.pOKa, 'facec', 'flat');               % Set the face color flat
                set(obj.Pobj_f.pOKa, 'FaceColor', obj.ms.sOK.C);	% Set the face color
                set(obj.Pobj_f.pOKa, 'EdgeColor', 'none');        	% Set the edge color
                obj.Pobj_f.pOKb = patch(obj.AxesHandle,'Faces', obj.ms.sOK.F, 'Vertices' ,obj.ms.sOK.V);
                set(obj.Pobj_f.pOKb, 'facec', 'flat');               % Set the face color flat
                set(obj.Pobj_f.pOKb, 'FaceColor', obj.ms.sOK.C);	% Set the face color
                set(obj.Pobj_f.pOKb, 'EdgeColor', 'none');        	% Set the edge color
                obj.Pobj_f.pOFa = patch(obj.AxesHandle,'Faces', obj.ms.sOF.F, 'Vertices' ,obj.ms.sOF.V);
                set(obj.Pobj_f.pOFa, 'facec', 'flat');               % Set the face color flat
                set(obj.Pobj_f.pOFa, 'FaceColor', obj.ms.sOF.C);	% Set the face color
                set(obj.Pobj_f.pOFa, 'EdgeColor', 'none');        	% Set the edge color
                obj.Pobj_f.pOFb = patch(obj.AxesHandle,'Faces', obj.ms.sOF.F, 'Vertices' ,obj.ms.sOF.V);
                set(obj.Pobj_f.pOFb, 'facec', 'flat');               % Set the face color flat
                set(obj.Pobj_f.pOFb, 'FaceColor', obj.ms.sOF.C);	% Set the face color
                set(obj.Pobj_f.pOFb, 'EdgeColor', 'none');        	% Set the edge color
                obj.Pobj_f.pIKa = patch(obj.AxesHandle,'Faces', obj.ms.sIK.F, 'Vertices' ,obj.ms.sIK.V);
                set(obj.Pobj_f.pIKa, 'facec', 'flat');               % Set the face color flat
                set(obj.Pobj_f.pIKa, 'FaceColor', obj.ms.sIK.C);	% Set the face color
                set(obj.Pobj_f.pIKa, 'EdgeColor', 'none');        	% Set the edge color
                obj.Pobj_f.pIKb = patch(obj.AxesHandle,'Faces', obj.ms.sIK.F, 'Vertices' ,obj.ms.sIK.V);
                set(obj.Pobj_f.pIKb, 'facec', 'flat');               % Set the face color flat
                set(obj.Pobj_f.pIKb, 'FaceColor', obj.ms.sIK.C);	% Set the face color
                set(obj.Pobj_f.pIKb, 'EdgeColor', 'none');        	% Set the edge color
                obj.Pobj_f.pIFa = patch(obj.AxesHandle,'Faces', obj.ms.sIF.F, 'Vertices' ,obj.ms.sIF.V);
                set(obj.Pobj_f.pIFa, 'facec', 'flat');               % Set the face color flat
                set(obj.Pobj_f.pIFa, 'FaceVertexCData', obj.ms.sIF.C);	% Set the face color
                set(obj.Pobj_f.pIFa, 'EdgeColor', 'none');        	% Set the edge color
                obj.Pobj_f.pIFb = patch(obj.AxesHandle,'Faces', obj.ms.sIF.F, 'Vertices' ,obj.ms.sIF.V);
                set(obj.Pobj_f.pIFb, 'facec', 'flat');               % Set the face color flat
                set(obj.Pobj_f.pIFb, 'FaceVertexCData', obj.ms.sIF.C);	% Set the face color
                set(obj.Pobj_f.pIFb, 'EdgeColor', 'none');        	% Set the edge color
            else
                obj.Pobj_f.pS = patch(obj.AxesHandle,'Faces', obj.ms.sS.F, 'Vertices', obj.ms.sS.V);
                set(obj.Pobj_f.pS, 'facec', 'flat');                % Set the face color flat
                set(obj.Pobj_f.pS, 'FaceColor', obj.ms.sS.C);       % Set the color (from file)
                set(obj.Pobj_f.pS, 'EdgeColor', 'none');            % Set the edge color
            end
            if ee_att == 2
                obj.Pobj_f.pG = patch(obj.AxesHandle,'Faces', obj.ms.sG.F, 'Vertices', obj.ms.sG.V);
                set(obj.Pobj_f.pG, 'facec', 'flat');                % Set the face color flat
                set(obj.Pobj_f.pG, 'FaceColor', obj.ms.sG.C);       % Set the color (from file)
                set(obj.Pobj_f.pG, 'EdgeColor', 'none');            % Set the edge color
            elseif ee_att == 3
                obj.Pobj_f.pTa = patch(obj.AxesHandle,'Faces', obj.ms.sTa.F, 'Vertices', obj.ms.sTa.V);
                set(obj.Pobj_f.pTa, 'facec', 'flat');                % Set the face color flat
                set(obj.Pobj_f.pTa, 'FaceColor', obj.ms.sTa.C);       % Set the color (from file)
                set(obj.Pobj_f.pTa, 'EdgeColor', 'none');            % Set the edge color
                obj.Pobj_f.pTb = patch(obj.AxesHandle,'Faces', obj.ms.sTb.F, 'Vertices', obj.ms.sTb.V);
                set(obj.Pobj_f.pTb, 'facec', 'flat');                % Set the face color flat
                set(obj.Pobj_f.pTb, 'FaceColor', obj.ms.sTb.C);       % Set the color (from file)
                set(obj.Pobj_f.pTb, 'EdgeColor', 'none');            % Set the edge color
            end
        end
    
        if coord_frame_on == 1 && ghost_on == 1
            obj.Pobj_d.pF = patch(obj.AxesHandle,'Faces', obj.ms.sF.F, 'Vertices', obj.ms.sF.V);
            set(obj.Pobj_d.pF, 'facec', 'flat');                % Set the face color flat
            set(obj.Pobj_d.pF, 'FaceVertexCData', obj.ms.sF.C);	% Set the color (from file)
            set(obj.Pobj_d.pF, 'EdgeColor', 'none');            % Set the edge color
        end
        if coord_frame_on == 1
            obj.Pobj_f.pF = patch(obj.AxesHandle,'Faces', obj.ms.sF.F, 'Vertices', obj.ms.sF.V);
            set(obj.Pobj_f.pF, 'facec', 'flat');                % Set the face color flat
            set(obj.Pobj_f.pF, 'FaceVertexCData', obj.ms.sF.C); % Set the color (from file)
            set(obj.Pobj_f.pF, 'EdgeColor', 'none');            % Set the edge color
        end
        if coord_frame_on == 1 && ((task_mode == 1 && running_flag == 0) || (task_mode == 2 && (running_flag == 0 || trj_profile == 0)))
            obj.Pobj_r.pF = patch(obj.AxesHandle,'Faces', obj.ms.sF.F, 'Vertices', obj.ms.sF.V);
            set(obj.Pobj_r.pF, 'facec', 'none');                % Set the face color flat
            set(obj.Pobj_r.pF, 'FaceVertexCData', obj.ms.sF.C); % Set the color (from file)
            set(obj.Pobj_r.pF, 'EdgeColor', 'flat');            % Set the edge color
        end
    end
    
end


function updateView(obj, state)
    
    hold(obj.AxesHandle,'on')

    if state.running_flag == 1
        TactI_h = robotics.engines.KinematicsEngine.getTransMatrix(state.TI_0,state.kin.a_j,state.kin.alpha_j,state.kin.d_j,state.kin.theta_O_j,state.kin.j_type,state.act.q_pos);
        if (isprop(state, 'sim_mode') || isfield(state, 'sim_mode')) && state.sim_mode == 0
            TdesI_h = state.fin.Ti;
        else
            TdesI_h = robotics.engines.KinematicsEngine.getTransMatrix(state.TI_0,state.kin.a_j,state.kin.alpha_j,state.kin.d_j,state.kin.theta_O_j,state.kin.j_type,state.des.q_pos);
        end
    else
        TactI_h = state.ini.Ti;
        TdesI_h = state.fin.Ti;
    end

    if state.task_mode > 0
        if state.running_flag == 0
            if isfield(state.ini, 'Re_ref') && isfield(state.ini, 't_pos_ref') && ~isempty(state.ini.Re_ref) && ~isempty(state.ini.t_pos_ref)
                Tref_ini = [state.ini.Re_ref, state.ini.t_pos_ref; zeros(1,3), 1];
            else
                Tref_ini = [state.ini.Re, state.ini.t_pos; zeros(1,3), 1];
            end
        end
        if isfield(state.fin, 'Re_ref') && isfield(state.fin, 't_pos_ref') && ~isempty(state.fin.Re_ref) && ~isempty(state.fin.t_pos_ref)
            Tref_fin = [state.fin.Re_ref, state.fin.t_pos_ref; zeros(1,3), 1];
        else
            Tref_fin = [state.fin.Re, state.fin.t_pos; zeros(1,3), 1];
        end
    end

    if state.ee_att == 1 && state.robot_model == 2
        Tn_sC   = [eye(3),[0;0;0.0085];zeros(1,3),1];
        TsC_S   = [eye(3),[0;0;0.0375];zeros(1,3),1];
        TS_gC   = [eye(3),[0;0;0.0111];zeros(1,3),1];
        TgC_gB  = [eye(3),[0;0;0.0900];zeros(1,3),1];
        TgB_oKa = [rot_y(pi/4),[0.035;0;-0.035];zeros(1,3),1];
        TgB_oKb = [rot_z(pi)*rot_y(pi/4),[-0.035;0;-0.035];zeros(1,3),1];
        TgB_oFa = [rot_y(pi/2),[0.06;0;-0.015];zeros(1,3),1];
        TgB_oFb = [rot_z(pi)*rot_y(pi/2),[-0.06;0;-0.015];zeros(1,3),1];
        TgB_iKa = [rot_y(pi/2.25),[0.02;0;-0.025];zeros(1,3),1];
        TgB_iKb = [rot_z(pi)*rot_y(pi/2.25),[-0.02;0;-0.025];zeros(1,3),1];
        TgB_iFa = [rot_y(pi/2),[0.065;0;0.035];zeros(1,3),1];
        TgB_iFb = [rot_z(pi)*rot_y(pi/2),[-0.065;0;0.035];zeros(1,3),1];
        dz_e = 0.1874;
    end
    if state.ee_att > 0 && (state.robot_model == 4 || state.robot_model == 5)
        if state.robot_model == 5
            sensor_angle = pi/4;
        else
            sensor_angle = 0;
        end
        sensor_length = 0.0333;
        gripper_length = 0.150;
        tool_lengthA = 0.100;
        tool_lengthB = 0.081;
    end

    if state.line_on == 0 && state.robot_model ~= 0

        if state.ghost_on == 1
            nvd1 = [obj.ms.s1.V,ones(size(obj.ms.s1.V(:,1)))]*TdesI_h(:,:,2)';
            nvd2 = [obj.ms.s2.V,ones(size(obj.ms.s2.V(:,1)))]*TdesI_h(:,:,3)';
            nvd3 = [obj.ms.s3.V,ones(size(obj.ms.s3.V(:,1)))]*TdesI_h(:,:,4)';
            nvd4 = [obj.ms.s4.V,ones(size(obj.ms.s4.V(:,1)))]*TdesI_h(:,:,5)';
            nvd5 = [obj.ms.s5.V,ones(size(obj.ms.s5.V(:,1)))]*TdesI_h(:,:,6)';
            nvd6 = [obj.ms.s6.V,ones(size(obj.ms.s6.V(:,1)))]*TdesI_h(:,:,7)';
            if state.kin.n == 7
                nvd7 = [obj.ms.s7.V,ones(size(obj.ms.s7.V(:,1)))]*TdesI_h(:,:,8)';
            end
            if state.ee_att > 0
                if state.robot_model == 1 || state.robot_model == 3
                    nvdE = [obj.ms.sE.V,ones(size(obj.ms.sE.V(:,1)))]*TdesI_h(:,:,state.kin.n+2)';
                    if state.robot_model == 1
                        nvdFl = [obj.ms.sFl.V,ones(size(obj.ms.sFl.V(:,1)))]*Rot_z(pi)'*Trn_y(-state.qEDes)'*TdesI_h(:,:,state.kin.n+2)';
                        nvdFr = [obj.ms.sFr.V,ones(size(obj.ms.sFr.V(:,1)))]*Trn_y(state.qEDes)'*TdesI_h(:,:,state.kin.n+2)';
                    end
                    if state.robot_model == 3
                        nvdE2 = [obj.ms.sE2.V,ones(size(obj.ms.sE2.V(:,1)))]*Rot_x(-state.qEDes)'*Trn_z(-0.1)'*TdesI_h(:,:,state.kin.n+2)';
                    end
                elseif state.robot_model == 2
                    TdesI_sC = TdesI_h(:,:,state.kin.n+2)*Tn_sC*Trn_z(-dz_e);
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
                    nvdSC = [obj.ms.sSC.V,ones(size(obj.ms.sSC.V(:,1)))]*TdesI_sC';
                    nvdS = [obj.ms.sS.V,ones(size(obj.ms.sS.V(:,1)))]*TdesI_S';
                    nvdGC = [obj.ms.sGC.V,ones(size(obj.ms.sGC.V(:,1)))]*TdesI_gC';
                    nvdGB = [obj.ms.sGB.V,ones(size(obj.ms.sGB.V(:,1)))]*TdesI_gB';
                    nvdOKa = [obj.ms.sOK.V,ones(size(obj.ms.sOK.V(:,1)))]*TdesI_oKa';
                    nvdOKb = [obj.ms.sOK.V,ones(size(obj.ms.sOK.V(:,1)))]*TdesI_oKb';
                    nvdOFa = [obj.ms.sOF.V,ones(size(obj.ms.sOF.V(:,1)))]*TdesI_oFa';
                    nvdOFb = [obj.ms.sOF.V,ones(size(obj.ms.sOF.V(:,1)))]*TdesI_oFb';
                    nvdIKa = [obj.ms.sIK.V,ones(size(obj.ms.sIK.V(:,1)))]*TdesI_iKa';
                    nvdIKb = [obj.ms.sIK.V,ones(size(obj.ms.sIK.V(:,1)))]*TdesI_iKb';
                    nvdIFa = [obj.ms.sIF.V,ones(size(obj.ms.sIF.V(:,1)))]*TdesI_iFa';
                    nvdIFb = [obj.ms.sIF.V,ones(size(obj.ms.sIF.V(:,1)))]*TdesI_iFb';
                else
                    nvdS = [obj.ms.sS.V,ones(size(obj.ms.sS.V(:,1)))]*Rot_z(sensor_angle)'*Trn_z(sensor_length)'*Trn_z(0.110)'*TdesI_h(:,:,state.kin.n+1)';
                end
                if state.ee_att == 2
                    nvdG = [obj.ms.sG.V,ones(size(obj.ms.sG.V(:,1)))]*Rot_z(sensor_angle)'*Trn_z(gripper_length)'*Trn_z(sensor_length)'*Trn_z(0.110)'*TdesI_h(:,:,state.kin.n+1)';
                end
                if state.ee_att == 3
                    nvdTa = [obj.ms.sTa.V,ones(size(obj.ms.sTa.V(:,1)))]*Rot_z(sensor_angle)'*Trn_z(tool_lengthA)'*Trn_z(sensor_length)'*Trn_z(0.110)'*TdesI_h(:,:,state.kin.n+1)';
                    nvdTb = [obj.ms.sTb.V,ones(size(obj.ms.sTb.V(:,1)))]*Rot_z(sensor_angle)'*Trn_z(tool_lengthA+tool_lengthB)'*Trn_z(sensor_length)'*Trn_z(0.110)'*TdesI_h(:,:,state.kin.n+1)';
                end
            end
        end
        nva0 = [obj.ms.s0.V,ones(size(obj.ms.s0.V(:,1)))]*TactI_h(:,:,1)';
        nva1 = [obj.ms.s1.V,ones(size(obj.ms.s1.V(:,1)))]*TactI_h(:,:,2)';
        nva2 = [obj.ms.s2.V,ones(size(obj.ms.s2.V(:,1)))]*TactI_h(:,:,3)';
        nva3 = [obj.ms.s3.V,ones(size(obj.ms.s3.V(:,1)))]*TactI_h(:,:,4)';
        nva4 = [obj.ms.s4.V,ones(size(obj.ms.s4.V(:,1)))]*TactI_h(:,:,5)';
        nva5 = [obj.ms.s5.V,ones(size(obj.ms.s5.V(:,1)))]*TactI_h(:,:,6)';
        nva6 = [obj.ms.s6.V,ones(size(obj.ms.s6.V(:,1)))]*TactI_h(:,:,7)';
        if state.kin.n == 7
            nva7 = [obj.ms.s7.V,ones(size(obj.ms.s7.V(:,1)))]*TactI_h(:,:,8)';
        end
        if state.ee_att > 0
            if state.robot_model == 1 || state.robot_model == 3
                nvaE = [obj.ms.sE.V,ones(size(obj.ms.sE.V(:,1)))]*TactI_h(:,:,state.kin.n+2)';
                if state.robot_model == 1
                    nvaFl = [obj.ms.sFl.V,ones(size(obj.ms.sFl.V(:,1)))]*Rot_z(pi)'*Trn_y(-state.qEIni)'*TactI_h(:,:,state.kin.n+2)';
                    nvaFr = [obj.ms.sFr.V,ones(size(obj.ms.sFr.V(:,1)))]*Trn_y(state.qEIni)'*TactI_h(:,:,state.kin.n+2)';
                end
                if state.robot_model == 3
                    nvaE2 = [obj.ms.sE2.V,ones(size(obj.ms.sE2.V(:,1)))]*Rot_x(-state.qEIni)'*Trn_z(-0.1)'*TactI_h(:,:,state.kin.n+2)';
                end
            elseif state.robot_model == 2
                TactI_sC = TactI_h(:,:,state.kin.n+2)*Tn_sC*Trn_z(-dz_e);
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
                nvaSC = [obj.ms.sSC.V,ones(size(obj.ms.sSC.V(:,1)))]*TactI_sC';
                nvaS = [obj.ms.sS.V,ones(size(obj.ms.sS.V(:,1)))]*TactI_S';
                nvaGC = [obj.ms.sGC.V,ones(size(obj.ms.sGC.V(:,1)))]*TactI_gC';
                nvaGB = [obj.ms.sGB.V,ones(size(obj.ms.sGB.V(:,1)))]*TactI_gB';
                nvaOKa = [obj.ms.sOK.V,ones(size(obj.ms.sOK.V(:,1)))]*TactI_oKa';
                nvaOKb = [obj.ms.sOK.V,ones(size(obj.ms.sOK.V(:,1)))]*TactI_oKb';
                nvaOFa = [obj.ms.sOF.V,ones(size(obj.ms.sOF.V(:,1)))]*TactI_oFa';
                nvaOFb = [obj.ms.sOF.V,ones(size(obj.ms.sOF.V(:,1)))]*TactI_oFb';
                nvaIKa = [obj.ms.sIK.V,ones(size(obj.ms.sIK.V(:,1)))]*TactI_iKa';
                nvaIKb = [obj.ms.sIK.V,ones(size(obj.ms.sIK.V(:,1)))]*TactI_iKb';
                nvaIFa = [obj.ms.sIF.V,ones(size(obj.ms.sIF.V(:,1)))]*TactI_iFa';
                nvaIFb = [obj.ms.sIF.V,ones(size(obj.ms.sIF.V(:,1)))]*TactI_iFb';
            else
                nvaS = [obj.ms.sS.V,ones(size(obj.ms.sS.V(:,1)))]*Rot_z(sensor_angle)'*Trn_z(sensor_length)'*Trn_z(0.110)'*TactI_h(:,:,state.kin.n+1)';
            end
            if state.ee_att == 2
                nvaG = [obj.ms.sG.V,ones(size(obj.ms.sG.V(:,1)))]*Rot_z(sensor_angle)'*Trn_z(gripper_length)'*Trn_z(sensor_length)'*Trn_z(0.110)'*TactI_h(:,:,state.kin.n+1)';
            end
            if state.ee_att == 3
                nvaTa = [obj.ms.sTa.V,ones(size(obj.ms.sTa.V(:,1)))]*Rot_z(sensor_angle)'*Trn_z(tool_lengthA)'*Trn_z(sensor_length)'*Trn_z(0.110)'*TactI_h(:,:,state.kin.n+1)';
                nvaTb = [obj.ms.sTb.V,ones(size(obj.ms.sTb.V(:,1)))]*Rot_z(sensor_angle)'*Trn_z(tool_lengthA+tool_lengthB)'*Trn_z(sensor_length)'*Trn_z(0.110)'*TactI_h(:,:,state.kin.n+1)';
            end
        end
        if state.coord_frame_on == 1
            nvdF = [obj.ms.sF.V,ones(size(obj.ms.sF.V(:,1)))]*TdesI_h(:,:,state.kin.n+2)';
            nvaF = [obj.ms.sF.V,ones(size(obj.ms.sF.V(:,1)))]*TactI_h(:,:,state.kin.n+2)';
            if state.task_mode == 1 && state.running_flag == 0
                nvaR = [obj.ms.sF.V,ones(size(obj.ms.sF.V(:,1)))]*Tref_ini';
            elseif state.task_mode == 2 && (state.running_flag == 0 || state.trj_profile == 0)
                nvdR = [obj.ms.sF.V,ones(size(obj.ms.sF.V(:,1)))]*Tref_fin';
            end
        end
    
        if state.ghost_on == 1
            set(obj.Pobj_d.p1,'Vertices',nvd1(:,1:3),'FaceAlpha',0.25)
            set(obj.Pobj_d.p2,'Vertices',nvd2(:,1:3),'FaceAlpha',0.25)
            set(obj.Pobj_d.p3,'Vertices',nvd3(:,1:3),'FaceAlpha',0.25)
            set(obj.Pobj_d.p4,'Vertices',nvd4(:,1:3),'FaceAlpha',0.25)
            set(obj.Pobj_d.p5,'Vertices',nvd5(:,1:3),'FaceAlpha',0.25)
            set(obj.Pobj_d.p6,'Vertices',nvd6(:,1:3),'FaceAlpha',0.25)
            if state.kin.n == 7
                set(obj.Pobj_d.p7,'Vertices',nvd7(:,1:3),'FaceAlpha',0.25)
            end
            if state.ee_att > 0
                if state.robot_model == 1 || state.robot_model == 3
                    set(obj.Pobj_d.pE,'Vertices',nvdE(:,1:3),'FaceAlpha',0.25)
                    if state.robot_model == 1
                        set(obj.Pobj_d.pFl,'Vertices',nvdFl(:,1:3),'FaceAlpha',0.25)
                        set(obj.Pobj_d.pFr,'Vertices',nvdFr(:,1:3),'FaceAlpha',0.25)
                    end
                    if state.robot_model == 3
                        set(obj.Pobj_d.pE2,'Vertices',nvdE2(:,1:3),'FaceAlpha',0.25)
                    end
                elseif state.robot_model == 2
                    set(obj.Pobj_d.pSC,'Vertices',nvdSC(:,1:3),'FaceAlpha',0.25)
                    set(obj.Pobj_d.pS,'Vertices',nvdS(:,1:3),'FaceAlpha',0.25)
                    set(obj.Pobj_d.pGC,'Vertices',nvdGC(:,1:3),'FaceAlpha',0.25)
                    set(obj.Pobj_d.pGB,'Vertices',nvdGB(:,1:3),'FaceAlpha',0.25)
                    set(obj.Pobj_d.pOKa,'Vertices',nvdOKa(:,1:3),'FaceAlpha',0.25)
                    set(obj.Pobj_d.pOKb,'Vertices',nvdOKb(:,1:3),'FaceAlpha',0.25)
                    set(obj.Pobj_d.pOFa,'Vertices',nvdOFa(:,1:3),'FaceAlpha',0.25)
                    set(obj.Pobj_d.pOFb,'Vertices',nvdOFb(:,1:3),'FaceAlpha',0.25)
                    set(obj.Pobj_d.pIKa,'Vertices',nvdIKa(:,1:3),'FaceAlpha',0.25)
                    set(obj.Pobj_d.pIKb,'Vertices',nvdIKb(:,1:3),'FaceAlpha',0.25)
                    set(obj.Pobj_d.pIFa,'Vertices',nvdIFa(:,1:3),'FaceAlpha',0.25)
                    set(obj.Pobj_d.pIFb,'Vertices',nvdIFb(:,1:3),'FaceAlpha',0.25)
                else
                    set(obj.Pobj_d.pS,'Vertices',nvdS(:,1:3),'FaceAlpha',0.25)
                end
                if state.ee_att == 2
                    set(obj.Pobj_d.pG,'Vertices',nvdG(:,1:3),'FaceAlpha',0.25)
                end
                if state.ee_att == 3
                    set(obj.Pobj_d.pTa,'Vertices',nvdTa(:,1:3),'FaceAlpha',0.25)
                    set(obj.Pobj_d.pTb,'Vertices',nvdTb(:,1:3),'FaceAlpha',0.25)
                end
            end
        end
        set(obj.Pobj_f.p0,'Vertices',nva0(:,1:3))
        set(obj.Pobj_f.p1,'Vertices',nva1(:,1:3))
        set(obj.Pobj_f.p2,'Vertices',nva2(:,1:3))
        set(obj.Pobj_f.p3,'Vertices',nva3(:,1:3))
        set(obj.Pobj_f.p4,'Vertices',nva4(:,1:3))
        set(obj.Pobj_f.p5,'Vertices',nva5(:,1:3))
        set(obj.Pobj_f.p6,'Vertices',nva6(:,1:3))
        if state.kin.n == 7
            set(obj.Pobj_f.p7,'Vertices',nva7(:,1:3))
        end
        if state.ee_att > 0
            if state.robot_model == 1 || state.robot_model == 3
                set(obj.Pobj_f.pE,'Vertices',nvaE(:,1:3))
                if state.robot_model == 1
                    set(obj.Pobj_f.pFl,'Vertices',nvaFl(:,1:3))
                    set(obj.Pobj_f.pFr,'Vertices',nvaFr(:,1:3))
                end
                if state.robot_model == 3
                    set(obj.Pobj_f.pE2,'Vertices',nvaE2(:,1:3))
                end
            elseif state.robot_model == 2
                set(obj.Pobj_f.pSC,'Vertices',nvaSC(:,1:3))
                set(obj.Pobj_f.pS,'Vertices',nvaS(:,1:3))
                set(obj.Pobj_f.pGC,'Vertices',nvaGC(:,1:3))
                set(obj.Pobj_f.pGB,'Vertices',nvaGB(:,1:3))
                set(obj.Pobj_f.pOKa,'Vertices',nvaOKa(:,1:3))
                set(obj.Pobj_f.pOKb,'Vertices',nvaOKb(:,1:3))
                set(obj.Pobj_f.pOFa,'Vertices',nvaOFa(:,1:3))
                set(obj.Pobj_f.pOFb,'Vertices',nvaOFb(:,1:3))
                set(obj.Pobj_f.pIKa,'Vertices',nvaIKa(:,1:3))
                set(obj.Pobj_f.pIKb,'Vertices',nvaIKb(:,1:3))
                set(obj.Pobj_f.pIFa,'Vertices',nvaIFa(:,1:3))
                set(obj.Pobj_f.pIFb,'Vertices',nvaIFb(:,1:3))
            elseif state.robot_model == 3
                set(obj.Pobj_f.pE,'Vertices',nvaE(:,1:3))
                set(obj.Pobj_f.pE2,'Vertices',nvaE2(:,1:3))
            else
                set(obj.Pobj_f.pS,'Vertices',nvaS(:,1:3))
            end
            if state.ee_att == 2
                set(obj.Pobj_f.pG,'Vertices',nvaG(:,1:3))
            end
            if state.ee_att == 3
                set(obj.Pobj_f.pTa,'Vertices',nvaTa(:,1:3))
                set(obj.Pobj_f.pTb,'Vertices',nvaTb(:,1:3))
            end
        end
        if state.coord_frame_on == 1
            if state.ghost_on == 1 && isfield(obj.Pobj_d, 'pF') && isvalid(obj.Pobj_d.pF)
                set(obj.Pobj_d.pF,'Vertices',nvdF(:,1:3),'FaceAlpha',0.25)
            end
            if isfield(obj.Pobj_f, 'pF') && isvalid(obj.Pobj_f.pF)
                set(obj.Pobj_f.pF,'Vertices',nvaF(:,1:3))
            end
            if state.task_mode == 1 && state.running_flag == 0
                if isfield(obj.Pobj_r, 'pF') && isvalid(obj.Pobj_r.pF)
                    set(obj.Pobj_r.pF,'Vertices',nvaR(:,1:3))
                end
            elseif state.task_mode == 2 && (state.running_flag == 0 || state.trj_profile == 0)
                if isfield(obj.Pobj_r, 'pF') && isvalid(obj.Pobj_r.pF)
                    set(obj.Pobj_r.pF,'Vertices',nvdR(:,1:3),'FaceAlpha',0.25)
                end
            end
        end
    else
        ee_axes_length = 0.1;
        
        % Robot Stick Color Palette Reference:
        %   lemon      = [255, 200,   0]./255  (Staubli RX160 / RX160L)
        %   cyan_blue  = [0.3, 0.7, 0.9]       (UR3)
        %   deep_blue  = [0.2, 0.6, 0.8]       (Unitree Z1)
        %   crimson    = [0.8, 0.3, 0.2]       (Franka Emika / Default)
        %   orange     = [255, 128,   0]./255  (Custom/Industrial theme)
        %   light_blue = [110, 158, 194]./255  (Light theme accent)
        %   dark_grey  = [ 91,  95,  98]./255  (Metallic accent)
        if state.robot_model == 4 || state.robot_model == 5
            link_color = [255 200 0]./255;      % Staubli Yellow
        elseif state.robot_model == 3
            link_color = [0.2 0.6 0.8];          % Unitree Blue
        elseif state.robot_model == 2
            link_color = [0.3 0.7 0.9];          % UR3 Cyan
        else
            link_color = [0.8 0.3 0.2];          % Franka Crimson
        end
        
        DH_vec_act = zeros(3,state.kin.n+2);
        DH_vec_des = zeros(3,state.kin.n+2);
        for i = 1:state.kin.n+2
            DH_vec_act(:,i) = TactI_h(1:3,4,i);
            DH_vec_des(:,i) = TdesI_h(1:3,4,i);
        end

        can_reuse = isfield(obj.LineHandles, 'act_link') && ...
                    isvalid(obj.LineHandles.act_link) && ...
                    isfield(obj.LineHandles, 'n') && ...
                    obj.LineHandles.n == state.kin.n;
                    
        if ~can_reuse
            cla(obj.AxesHandle);
            hold(obj.AxesHandle, 'on');
            obj.LineHandles = struct();
            obj.LineHandles.n = state.kin.n;
            
            % Lightweight hardware-accelerated stick model
            obj.LineHandles.act_link = plot3(obj.AxesHandle, DH_vec_act(1,:), DH_vec_act(2,:), DH_vec_act(3,:), ...
                'LineWidth', 6, 'Color', link_color, 'Marker', 'o', 'MarkerSize', 8, 'MarkerFaceColor', link_color, 'MarkerEdgeColor', [0.1 0.1 0.1]);
            
            if state.ghost_on == 1
                obj.LineHandles.des_link = plot3(obj.AxesHandle, DH_vec_des(1,:), DH_vec_des(2,:), DH_vec_des(3,:), ...
                    'LineWidth', 3, 'Color', [0.55 0.55 0.55], 'LineStyle', '--', 'Marker', 'o', 'MarkerSize', 6, 'MarkerFaceColor', [0.75 0.75 0.75], 'MarkerEdgeColor', [0.4 0.4 0.4]);
            end
            
            if state.coord_frame_on == 1
                p_ee = TactI_h(1:3,4,end);
                px = p_ee + ee_axes_length*TactI_h(1:3,1,end);
                py = p_ee + ee_axes_length*TactI_h(1:3,2,end);
                pz = p_ee + ee_axes_length*TactI_h(1:3,3,end);
                obj.LineHandles.frame_x = plot3(obj.AxesHandle, [p_ee(1) px(1)], [p_ee(2) px(2)], [p_ee(3) px(3)], 'r-', 'LineWidth', 2.5);
                obj.LineHandles.frame_y = plot3(obj.AxesHandle, [p_ee(1) py(1)], [p_ee(2) py(2)], [p_ee(3) py(3)], 'g-', 'LineWidth', 2.5);
                obj.LineHandles.frame_z = plot3(obj.AxesHandle, [p_ee(1) pz(1)], [p_ee(2) pz(2)], [p_ee(3) pz(3)], 'b-', 'LineWidth', 2.5);

                if state.ghost_on == 1
                    p_des = TdesI_h(1:3,4,end);
                    pdx = p_des + ee_axes_length*TdesI_h(1:3,1,end);
                    pdy = p_des + ee_axes_length*TdesI_h(1:3,2,end);
                    pdz = p_des + ee_axes_length*TdesI_h(1:3,3,end);
                    obj.LineHandles.des_frame_x = plot3(obj.AxesHandle, [p_des(1) pdx(1)], [p_des(2) pdx(2)], [p_des(3) pdx(3)], 'r--', 'LineWidth', 1.5);
                    obj.LineHandles.des_frame_y = plot3(obj.AxesHandle, [p_des(1) pdy(1)], [p_des(2) pdy(2)], [p_des(3) pdy(3)], 'g--', 'LineWidth', 1.5);
                    obj.LineHandles.des_frame_z = plot3(obj.AxesHandle, [p_des(1) pdz(1)], [p_des(2) pdz(2)], [p_des(3) pdz(3)], 'b--', 'LineWidth', 1.5);
                end

                if state.task_mode > 0 && ((state.task_mode == 1 && state.running_flag == 0) || (state.task_mode == 2 && (state.running_flag == 0 || state.trj_profile == 0)))
                    if state.task_mode == 1
                        T_ref = Tref_ini;
                    else
                        T_ref = Tref_fin;
                    end
                    p_ref = T_ref(1:3,4);
                    prx = p_ref + ee_axes_length*T_ref(1:3,1);
                    pry = p_ref + ee_axes_length*T_ref(1:3,2);
                    prz = p_ref + ee_axes_length*T_ref(1:3,3);
                    obj.LineHandles.ref_frame_x = plot3(obj.AxesHandle, [p_ref(1) prx(1)], [p_ref(2) prx(2)], [p_ref(3) prx(3)], 'r:', 'LineWidth', 3);
                    obj.LineHandles.ref_frame_y = plot3(obj.AxesHandle, [p_ref(1) pry(1)], [p_ref(2) pry(2)], [p_ref(3) pry(3)], 'g:', 'LineWidth', 3);
                    obj.LineHandles.ref_frame_z = plot3(obj.AxesHandle, [p_ref(1) prz(1)], [p_ref(2) prz(2)], [p_ref(3) prz(3)], 'b:', 'LineWidth', 3);
                end
            end
        else
            set(obj.LineHandles.act_link, 'XData', DH_vec_act(1,:), 'YData', DH_vec_act(2,:), 'ZData', DH_vec_act(3,:));
            if state.ghost_on == 1 && isfield(obj.LineHandles, 'des_link') && isvalid(obj.LineHandles.des_link)
                set(obj.LineHandles.des_link, 'XData', DH_vec_des(1,:), 'YData', DH_vec_des(2,:), 'ZData', DH_vec_des(3,:));
            end
            if state.coord_frame_on == 1
                if isfield(obj.LineHandles, 'frame_x') && isvalid(obj.LineHandles.frame_x)
                    p_ee = TactI_h(1:3,4,end);
                    px = p_ee + ee_axes_length*TactI_h(1:3,1,end);
                    py = p_ee + ee_axes_length*TactI_h(1:3,2,end);
                    pz = p_ee + ee_axes_length*TactI_h(1:3,3,end);
                    set(obj.LineHandles.frame_x, 'XData', [p_ee(1) px(1)], 'YData', [p_ee(2) px(2)], 'ZData', [p_ee(3) px(3)]);
                    set(obj.LineHandles.frame_y, 'XData', [p_ee(1) py(1)], 'YData', [p_ee(2) py(2)], 'ZData', [p_ee(3) py(3)]);
                    set(obj.LineHandles.frame_z, 'XData', [p_ee(1) pz(1)], 'YData', [p_ee(2) pz(2)], 'ZData', [p_ee(3) pz(3)]);
                end
                if state.ghost_on == 1 && isfield(obj.LineHandles, 'des_frame_x') && isvalid(obj.LineHandles.des_frame_x)
                    p_des = TdesI_h(1:3,4,end);
                    pdx = p_des + ee_axes_length*TdesI_h(1:3,1,end);
                    pdy = p_des + ee_axes_length*TdesI_h(1:3,2,end);
                    pdz = p_des + ee_axes_length*TdesI_h(1:3,3,end);
                    set(obj.LineHandles.des_frame_x, 'XData', [p_des(1) pdx(1)], 'YData', [p_des(2) pdx(2)], 'ZData', [p_des(3) pdz(3)]);
                    set(obj.LineHandles.des_frame_y, 'XData', [p_des(1) pdy(1)], 'YData', [p_des(2) pdy(2)], 'ZData', [p_des(3) pdy(3)]);
                    set(obj.LineHandles.des_frame_z, 'XData', [p_des(1) pdz(1)], 'YData', [p_des(2) pdz(2)], 'ZData', [p_des(3) pdz(3)]);
                end
                if isfield(obj.LineHandles, 'ref_frame_x') && isvalid(obj.LineHandles.ref_frame_x)
                    if state.task_mode > 0 && ((state.task_mode == 1 && state.running_flag == 0) || (state.task_mode == 2 && (state.running_flag == 0 || state.trj_profile == 0)))
                        if state.task_mode == 1
                            T_ref = Tref_ini;
                        else
                            T_ref = Tref_fin;
                        end
                        p_ref = T_ref(1:3,4);
                        prx = p_ref + ee_axes_length*T_ref(1:3,1);
                        pry = p_ref + ee_axes_length*T_ref(1:3,2);
                        prz = p_ref + ee_axes_length*T_ref(1:3,3);
                        set(obj.LineHandles.ref_frame_x, 'XData', [p_ref(1) prx(1)], 'YData', [p_ref(2) prx(2)], 'ZData', [p_ref(3) prx(3)], 'Visible', 'on');
                        set(obj.LineHandles.ref_frame_y, 'XData', [p_ref(1) pry(1)], 'YData', [p_ref(2) pry(2)], 'ZData', [p_ref(3) pry(3)], 'Visible', 'on');
                        set(obj.LineHandles.ref_frame_z, 'XData', [p_ref(1) prz(1)], 'YData', [p_ref(2) prz(2)], 'ZData', [p_ref(3) prz(3)], 'Visible', 'on');
                    else
                        set(obj.LineHandles.ref_frame_x, 'Visible', 'off');
                        set(obj.LineHandles.ref_frame_y, 'Visible', 'off');
                        set(obj.LineHandles.ref_frame_z, 'Visible', 'off');
                    end
                end
            end
        end
    end

    % -------------------------------------------------------------
    % 3D Manipulability Ellipsoid Rendering (for both CAD & Stick modes)
    % -------------------------------------------------------------
    isEllipOn = (isprop(state, 'ellipsoid_on') && state.ellipsoid_on == 1) || ...
                (isfield(state, 'ellipsoid_on') && state.ellipsoid_on == 1);
    if isEllipOn && ~isempty(obj.AxesHandle) && isvalid(obj.AxesHandle)
        % Determine active joint configuration
        if state.running_flag == 1
            q_ellip = state.act.q_pos;
        elseif (isprop(state, 'sim_mode') && state.sim_mode == 0) || (isfield(state, 'sim_mode') && state.sim_mode == 0)
            q_ellip = state.ini.q_pos;
        else
            q_ellip = state.fin.q_pos;
        end
        
        if isempty(TactI_h) || size(TactI_h, 3) < state.kin.n+2
            TactI_h = robotics.engines.KinematicsEngine.getTransMatrix(state.TI_0, state.kin.a_j, state.kin.alpha_j, state.kin.d_j, state.kin.theta_O_j, state.kin.j_type, q_ellip);
        end
        p_ee = TactI_h(1:3, 4, state.kin.n+2);
        [U_v, sigma_v] = robotics.engines.KinematicsEngine.computeManipulabilityEllipsoid(state.TI_0, state.kin, q_ellip);
        [Xe, Ye, Ze, axesLines] = robotics.engines.KinematicsEngine.getEllipsoidSurfaceData(p_ee, U_v, sigma_v, 0.20, 24);
        
        % Lazy instantiation of surface and axis lines if needed
        if isempty(obj.EllipsoidSurface) || ~isvalid(obj.EllipsoidSurface)
            obj.EllipsoidSurface = surface(obj.AxesHandle, Xe, Ye, Ze, ...
                'FaceColor', [0.15 0.75 0.95], ...
                'FaceAlpha', 0.40, ...
                'EdgeColor', [0.0 0.45 0.85], ...
                'EdgeAlpha', 0.35, ...
                'FaceLighting', 'gouraud', ...
                'Tag', 'ManipulabilityEllipsoid');
        else
            set(obj.EllipsoidSurface, 'XData', Xe, 'YData', Ye, 'ZData', Ze, 'Visible', 'on');
        end
        
        axisColors = {[0.9 0.2 0.2], [0.15 0.75 0.25], [0.2 0.45 0.95]};
        if isempty(obj.EllipsoidAxisLines) || ~all(isvalid(obj.EllipsoidAxisLines))
            obj.EllipsoidAxisLines = gobjects(3, 1);
            for axIdx = 1:3
                obj.EllipsoidAxisLines(axIdx) = plot3(obj.AxesHandle, ...
                    axesLines{axIdx}(1, :), axesLines{axIdx}(2, :), axesLines{axIdx}(3, :), ...
                    'Color', axisColors{axIdx}, 'LineWidth', 2.5, 'Visible', 'on', ...
                    'Tag', 'ManipulabilityEllipsoidAxis');
            end
        else
            for axIdx = 1:3
                set(obj.EllipsoidAxisLines(axIdx), ...
                    'XData', axesLines{axIdx}(1, :), ...
                    'YData', axesLines{axIdx}(2, :), ...
                    'ZData', axesLines{axIdx}(3, :), 'Visible', 'on');
            end
        end
    else
        if ~isempty(obj.EllipsoidSurface) && isvalid(obj.EllipsoidSurface)
            set(obj.EllipsoidSurface, 'Visible', 'off');
        end
        if ~isempty(obj.EllipsoidAxisLines)
            for axIdx = 1:length(obj.EllipsoidAxisLines)
                if isvalid(obj.EllipsoidAxisLines(axIdx))
                    set(obj.EllipsoidAxisLines(axIdx), 'Visible', 'off');
                end
            end
        end
    end

    hold(obj.AxesHandle,'off')

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
    
    % function [d,d_plus,theta,theta_plus,a,alpha] = robotLinks(v,q)
    % 
    %     switch (v)
    %         case 0
    %             % Stäubli RX90
    %             theta       = [q(1);0;q(2);0;q(3);q(4);q(5);q(6)];
    %             a           = zeros(8,1);
    %             d           = zeros(8,1);
    %             alpha       = [pi/2;pi/2;pi/2;pi/2;-pi/2;pi/2;pi/2;0];
    %             d_plus    	= [0.478;0.05;0.196;0.425;-0.146;0.425;0;0.100];
    %             theta_plus  = [pi/2;pi/2;pi/2;pi;0;0;pi;0];
    %         case 1
    %             % Stäubli RX160
    %             theta       = [q(1);0;q(2);0;q(3);q(4);q(5);q(6)];
    %             a           = zeros(8,1);
    %             d           = zeros(8,1);
    %             alpha       = [pi/2;pi/2;pi/2;pi/2;-pi/2;pi/2;pi/2;0];
    %             d_plus    	= [0.55;0.15;0.257;0.825;-0.257;0.625;0;0.110];
    %             theta_plus  = [pi/2;pi/2;pi/2;pi;0;0;pi;0];
    %         case 2
    %             % Stäubli RX160L
    %             theta       = [q(1);0;q(2);0;q(3);q(4);q(5);q(6)];
    %             a           = zeros(8,1);
    %             d           = zeros(8,1);
    %             alpha       = [pi/2;pi/2;pi/2;pi/2;-pi/2;pi/2;pi/2;0];
    %             d_plus    	= [0.55;0.15;0.257;0.825;-0.257;0.925;0;0.110];
    %             theta_plus  = [pi/2;pi/2;pi/2;pi;0;0;pi;0];
    %         case 3
    %             % Stäubli RX160+sl
    %             theta       = [q(1);0;q(2);0;q(3);q(4);q(5);q(6)];
    %             a           = zeros(8,1);
    %             d           = zeros(8,1);
    %             alpha       = [pi/2;pi/2;pi/2;pi/2;-pi/2;pi/2;pi/2;0];
    %             d_plus    	= [0.55;0.15;0.257;0.825;-0.257;0.625;0;0.110+0.033];
    %             theta_plus  = [pi/2;pi/2;pi/2;pi;0;0;pi;0];
    %         case 4
    %             % Stäubli RX160+sl+gl
    %             theta       = [q(1);0;q(2);0;q(3);q(4);q(5);q(6)];
    %             a           = zeros(8,1);
    %             d           = zeros(8,1);
    %             alpha       = [pi/2;pi/2;pi/2;pi/2;-pi/2;pi/2;pi/2;0];
    %             d_plus    	= [0.55;0.15;0.257;0.825;-0.257;0.625;0;0.110+0.033+0.144];
    %             theta_plus  = [pi/2;pi/2;pi/2;pi;0;0;pi;0];
    %         otherwise
    %             theta       = zeros(6,1);
    %             a           = zeros(6,1);
    %             d           = zeros(6,1);
    %             alpha       = zeros(6,1);
    %             d_plus    	= zeros(6,1);
    %             theta_plus  = zeros(6,1);
    %     end
    % end

end
    end
end
