classdef KinematicsEngine
    % KINEMATICSENGINE Static algorithms for forward and inverse robot kinematics.
    %   Encapsulates Modified Denavit-Hartenberg (MDH) forward transformation chains,
    %   relative link transformation matrices, and inverse kinematics solvers.
    
    methods (Static)
        function qDes = inverseKinematics(robot, invGeoConfig, kin, RGoal, tGoal, q0)
            % INVERSEKINEMATICS Solves the inverse kinematics problem for a target pose.
            %   qDes = INVERSEKINEMATICS(ROBOT, INVGEOCONFIG, KIN, RGOAL, TGOAL, Q0)
            %   delegates numerical or analytical IK computation to the polymorphic
            %   robot model instance.
            %
            %   Inputs:
            %       robot        - RobotModel instance (e.g. FrankaEmika, UR3, UnitreeZ1)
            %       invGeoConfig - struct: IK solver parameters (algorithm type, gains, TI_0)
            %       kin          - struct: Kinematic parameters (DH tables, joint types, limits)
            %       RGoal        - (3x3) double: Desired end-effector orientation in SO(3)
            %       tGoal        - (3x1) double: Desired end-effector position in R^3 [m]
            %       q0           - (Nx1) double: Initial/seed joint angles [rad or m]
            %
            %   Outputs:
            %       qDes         - (Nx1) double: Solved desired joint positions [rad or m]
            %
            %   See also getTransMatrix, getRelativeTransMatrix.
            
            [qDes, ~] = robot.computeInverseKinematics(invGeoConfig, kin, RGoal, tGoal, q0);
        end
        
        function T = getMDHTransform(a, alpha, d, theta)
            % GETMDHTRANSFORM Computes a single 4x4 Modified DH transformation matrix.
            %   T = GETMDHTRANSFORM(A, ALPHA, D, THETA) returns the SE(3) transformation:
            %       T = Rot_x(alpha) * Trans_x(a) * Rot_z(theta) * Trans_z(d)
            %
            %   Inputs:
            %       a     - double scalar: Link length in meters
            %       alpha - double scalar: Link twist in radians
            %       d     - double scalar: Joint offset in meters
            %       theta - double scalar: Joint angle in radians
            %
            %   Outputs:
            %       T     - (4x4) double: Homogeneous transformation matrix in SE(3)
            
            ca = cos(alpha); sa = sin(alpha);
            ct = cos(theta); st = sin(theta);
            T = [  ct,     -st,      0,       a;
                 ca*st,  ca*ct,   -sa,    -sa*d;
                 sa*st,  sa*ct,    ca,     ca*d;
                    0,       0,      0,       1];
        end

        function TR_i = getRelativeTransMatrix(TI_0, a_j, alpha_j, d_j, theta_j, j_type, q_j)
            % GETRELATIVETRANSMATRIX Computes relative transformations for each joint.
            %   TR_i = GETRELATIVETRANSMATRIX(TI_0, A_J, ALPHA_J, D_J, THETA_J, J_TYPE, Q_J)
            %   returns a 4x4x(N+2) slice array containing relative transformations
            %   between consecutive links.
            %
            %   Inputs:
            %       TI_0    - (4x4) double: Base to inertial world transformation
            %       a_j     - (Nx1) double: MDH link lengths [m]
            %       alpha_j - (Nx1) double: MDH link twists [rad]
            %       d_j     - (Nx1) double: MDH joint offsets [m]
            %       theta_j - (Nx1) double: MDH initial joint offsets [rad]
            %       j_type  - (Nx1) double: Joint type array (1 for Revolute, 0 for Prismatic)
            %       q_j     - (Nx1) double: Current joint position vector [rad or m]
            %
            %   Outputs:
            %       TR_i    - (4x4x(N+2)) double: Array of relative transformations
            
            k = length(q_j);
            TR_i = zeros(4, 4, k+2);
            TR_i(:,:,1) = TI_0;
        
            for j = 1:k
                th = theta_j(j) + j_type(j)*q_j(j);
                dj = d_j(j) + (1-j_type(j))*q_j(j);
                TR_i(:,:,j+1) = robotics.engines.KinematicsEngine.getMDHTransform(a_j(j), alpha_j(j), dj, th);
            end
            
            TR_i(:,:,k+2) = robotics.engines.KinematicsEngine.getMDHTransform(a_j(k+1), alpha_j(k+1), d_j(k+1), theta_j(k+1));
        end

        function T_ee = forwardKinematics(TI_0, kin, q)
            % FORWARDKINEMATICS Computes the (4x4) end-effector transformation matrix in SE(3).
            %   T_ee = FORWARDKINEMATICS(TI_0, KIN, Q)
            if isfield(kin, 'a_j')
                a = kin.a_j; alpha = kin.alpha_j; d = kin.d_j; theta = kin.theta_O_j;
            else
                a = kin.a; alpha = kin.alpha; d = kin.d; theta = kin.theta;
            end
            TI_i = robotics.engines.KinematicsEngine.getTransMatrix(TI_0, a, alpha, d, theta, kin.j_type, q);
            T_ee = TI_i(:, :, end);
        end

        function TI_i = getTransMatrix(TI_0, a_j, alpha_j, d_j, theta_j, j_type, q_j)
            % GETTRANSMATRIX Computes cumulative forward kinematic transformation chain.
            %   TI_i = GETTRANSMATRIX(TI_0, A_J, ALPHA_J, D_J, THETA_J, J_TYPE, Q_J)
            %   returns a 4x4x(N+2) slice array containing transformations from the
            %   inertial base frame to each robot joint, flange, and end-effector.
            %
            %   Inputs:
            %       TI_0    - (4x4) double: Base to inertial world transformation
            %       a_j     - (Nx1) double: MDH link lengths [m]
            %       alpha_j - (Nx1) double: MDH link twists [rad]
            %       d_j     - (Nx1) double: MDH joint offsets [m]
            %       theta_j - (Nx1) double: MDH initial joint offsets [rad]
            %       j_type  - (Nx1) double: Joint type array (1 for Revolute, 0 for Prismatic)
            %       q_j     - (Nx1) double: Current joint position vector [rad or m]
            %
            %   Outputs:
            %       TI_i    - (4x4x(N+2)) double: Cumulative homogeneous transformations
            
            k = length(q_j);
            TI_i = zeros(4, 4, k+2);
            TI_i(:,:,1) = TI_0;
            
            for j = 1:k
                th = theta_j(j) + j_type(j)*q_j(j);
                dj = d_j(j) + (1-j_type(j))*q_j(j);
                
                % Fast inlined MDH relative transform
                ca = cos(alpha_j(j)); sa = sin(alpha_j(j));
                ct = cos(th);         st = sin(th);
                aj = a_j(j);
                
                % Direct SE(3) compounding: T_next = T_prev * TR
                R_prev = TI_i(1:3, 1:3, j);
                p_prev = TI_i(1:3, 4,   j);
                
                R_rel = [ ct,       -st,        0;
                          ca * st,   ca * ct,  -sa;
                          sa * st,   sa * ct,   ca ];
                p_rel = [ aj; -sa * dj; ca * dj ];
                
                TI_i(1:3, 1:3, j+1) = R_prev * R_rel;
                TI_i(1:3, 4,   j+1) = R_prev * p_rel + p_prev;
                TI_i(4,   4,   j+1) = 1;
            end
            
            % Tool/Flange transform (joint k+1)
            ca = cos(alpha_j(k+1)); sa = sin(alpha_j(k+1));
            ct = cos(theta_j(k+1)); st = sin(theta_j(k+1));
            aj = a_j(k+1);          dj = d_j(k+1);
            
            R_prev = TI_i(1:3, 1:3, k+1);
            p_prev = TI_i(1:3, 4,   k+1);
            
            R_rel = [ ct,       -st,        0;
                      ca * st,   ca * ct,  -sa;
                      sa * st,   sa * ct,   ca ];
            p_rel = [ aj; -sa * dj; ca * dj ];
            
            TI_i(1:3, 1:3, k+2) = R_prev * R_rel;
            TI_i(1:3, 4,   k+2) = R_prev * p_rel + p_prev;
            TI_i(4,   4,   k+2) = 1;
        end
        
        function J = getGeometricJacobian(TI_0, kin, q)
            % GETGEOMETRICJACOBIAN Computes the 6xN geometric Jacobian matrix.
            %   J = GETGEOMETRICJACOBIAN(TI_0, KIN, Q)
            %   Inputs:
            %       TI_0 - (4x4) double: Base transformation
            %       kin  - struct: Kinematic parameters (DH parameters and joint types)
            %       q    - (Nx1) double: Joint position vector
            %   Outputs:
            %       J    - (6xN) double: Geometric Jacobian [Jv; Jw]
            
            TI_i = robotics.engines.KinematicsEngine.getTransMatrix(...
                TI_0, kin.a_j, kin.alpha_j, kin.d_j, kin.theta_O_j, kin.j_type, q);
            
            n = length(q);
            J = zeros(6, n);
            p_e = TI_i(1:3, 4, n+2);
            
            for i = 1:n
                z_i = TI_i(1:3, 3, i+1);
                p_i = TI_i(1:3, 4, i+1);
                if kin.j_type(i) == 1
                    % Revolute joint
                    J(1:3, i) = cross(z_i, p_e - p_i);
                    J(4:6, i) = z_i;
                else
                    % Prismatic joint
                    J(1:3, i) = z_i;
                    J(4:6, i) = zeros(3, 1);
                end
            end
        end

        function e_o = computeOrientationError(R_des, R_fbk)
            % COMPUTEORIENTATIONERROR Computes the orientation error vector in SO(3).
            %   e_o = COMPUTEORIENTATIONERROR(R_DES, R_FBK)
            %   Uses the standard cross-product formulation:
            %       e_o = 0.5 * sum_{i=1}^3 (R_fbk(:, i) x R_des(:, i))
            
            e_o = 0.5 * (cross(R_fbk(:,1), R_des(:,1)) + ...
                         cross(R_fbk(:,2), R_des(:,2)) + ...
                         cross(R_fbk(:,3), R_des(:,3)));
        end

        function [w, status, colorCode] = computeManipulability(TI_0, kin, q)
            % COMPUTEMANIPULABILITY Evaluates Yoshikawa's manipulability measure w = sqrt(det(J*J')).
            %   [W, STATUS, COLORCODE] = COMPUTEMANIPULABILITY(TI_0, KIN, Q)
            J = robotics.engines.KinematicsEngine.getGeometricJacobian(TI_0, kin, q);
            n = size(J, 2);
            if n >= 6
                w = sqrt(max(0, det(J * J')));
            else
                w = sqrt(max(0, det(J' * J)));
            end
            
            if w >= 0.05
                status = 'NOMINAL';
                colorCode = [0.15 0.65 0.25]; % Green
            elseif w >= 0.015
                status = 'CAUTION';
                colorCode = [0.85 0.55 0.10]; % Amber
            else
                status = 'NEAR SINGULARITY';
                colorCode = [0.85 0.15 0.15]; % Red
            end
        end

        function [minMarginDeg, closestJoint] = computeLimitMargin(kin, q)
            % COMPUTELIMITMARGIN Computes the smallest angular distance to any joint limit.
            %   [MINMARGINDEG, CLOSESTJOINT] = COMPUTELIMITMARGIN(KIN, Q)
            q_deg = rad2deg(q(:));
            q_min_deg = rad2deg(kin.q_posLim(:, 1));
            q_max_deg = rad2deg(kin.q_posLim(:, 2));
            
            distMin = q_deg - q_min_deg;
            distMax = q_max_deg - q_deg;
            margins = min(distMin, distMax);
            
            [minMarginDeg, closestJoint] = min(margins);
        end

        function [U_v, sigma_v, U_w, sigma_w] = computeManipulabilityEllipsoid(TI_0, kin, q)
            % COMPUTEMANIPULABILITYELLIPSOID Computes principal directions and singular values
            % of translational and rotational manipulability ellipsoids via SVD.
            %   [U_V, SIGMA_V, U_W, SIGMA_W] = COMPUTEMANIPULABILITYELLIPSOID(TI_0, KIN, Q)
            J = robotics.engines.KinematicsEngine.getGeometricJacobian(TI_0, kin, q);
            
            % Translational Jacobian (top 3 rows: linear velocity)
            J_v = J(1:3, :);
            [U_v, S_v, ~] = svd(J_v);
            sigma_v = diag(S_v(1:3, 1:3));
            
            % Rotational Jacobian (bottom 3 rows: angular velocity)
            J_w = J(4:6, :);
            [U_w, S_w, ~] = svd(J_w);
            sigma_w = diag(S_w(1:3, 1:3));
        end

        function [X, Y, Z, axesLines] = getEllipsoidSurfaceData(p_ee, U, sigma, scale, n_pts)
            % GETELLIPSOIDSURFACEDATA Generates 3D coordinates for a manipulability ellipsoid.
            %   [X, Y, Z, AXESLINES] = GETELLIPSOIDSURFACEDATA(P_EE, U, SIGMA, SCALE, N_PTS)
            if nargin < 4 || isempty(scale)
                scale = 0.15; % Default visual scale factor
            end
            if nargin < 5 || isempty(n_pts)
                n_pts = 20; % 20x20 grid is smooth and performant
            end
            
            % Unit sphere
            [Xs, Ys, Zs] = sphere(n_pts);
            
            % Scale semi-axes
            radii = sigma(:) * scale;
            radii = max(radii, 1e-4); % Stability guard
            
            X_scaled = radii(1) * Xs;
            Y_scaled = radii(2) * Ys;
            Z_scaled = radii(3) * Zs;
            
            % Rotate by U and translate to center p_ee
            pts = [X_scaled(:), Y_scaled(:), Z_scaled(:)] * U';
            X = reshape(pts(:, 1) + p_ee(1), size(Xs));
            Y = reshape(pts(:, 2) + p_ee(2), size(Ys));
            Z = reshape(pts(:, 3) + p_ee(3), size(Zs));
            
            % 3 Principal Axes lines [X; Y; Z] from p_ee - v to p_ee + v
            axesLines = cell(3, 1);
            for i = 1:3
                v_axis = U(:, i) * radii(i);
                axesLines{i} = [p_ee(1:3) - v_axis, p_ee(1:3) + v_axis]; % [3 x 2]
            end
        end

        % Helper methods for Transformation Matrices
        function output = Trn_x(input)
            output = [  1 0 0 input;
                        0 1 0 0;
                        0 0 1 0;
                        0 0 0 1 ];
        end
        
        function output = Trn_z(input)
            output = [  1 0 0 0;
                        0 1 0 0;
                        0 0 1 input;
                        0 0 0 1 ];
        end
        
        function output = Rot_x(input)
            output = [	1   0           0           0;
                        0   cos(input)  -sin(input) 0;
                        0   sin(input)	cos(input)  0;
                        0   0           0           1 ];
        end
        
        function output = Rot_z(input)
            output = [  cos(input)  -sin(input) 0   0;
                        sin(input)  cos(input)  0   0;
                        0           0           1   0;
                        0           0           0   1 ];
        end
    end
end



