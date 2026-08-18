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
            
            TR_i = robotics.engines.KinematicsEngine.getRelativeTransMatrix(TI_0, a_j, alpha_j, d_j, theta_j, j_type, q_j);
            k = length(q_j);
            TI_i = zeros(4, 4, k+2);
            TI_i(:,:,1) = TR_i(:,:,1);
            for j = 1:k+1
                TI_i(:,:,j+1) = TI_i(:,:,j) * TR_i(:,:,j+1);
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



