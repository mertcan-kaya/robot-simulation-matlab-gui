classdef TestSpatialMath < matlab.unittest.TestCase
    % TESTSPATIALMATH Unit tests for spatial algebra and Lie group math utilities in +robotics/+math.

    methods (Test)

        function testSE3TransformEmbeddingAndExtraction(testCase)
            % Test SO3R3_SE3 and SE3_SO3R3 roundtrip
            R_in = [0, -1, 0; 1, 0, 0; 0, 0, 1];
            r_in = [0.15; -0.32; 0.78];

            T = robotics.math.SO3R3_SE3(R_in, r_in);
            testCase.verifySize(T, [4, 4]);
            testCase.verifyEqual(T(4,:), [0, 0, 0, 1]);

            [R_out, r_out] = robotics.math.SE3_SO3R3(T);
            testCase.verifyEqual(R_out, R_in, 'AbsTol', 1e-12);
            testCase.verifyEqual(r_out, r_in, 'AbsTol', 1e-12);
        end

        function testSkewSymmetricIdentity(testCase)
            % Test SkewSym matrix cross-product equivalence: [v]_x * w == cross(v, w)
            v = [1.2; -3.4; 5.6];
            w = [0.7; 2.1; -1.5];

            Sv = robotics.math.SkewSym(v);
            testCase.verifySize(Sv, [3, 3]);
            testCase.verifyEqual(Sv + Sv', zeros(3, 3), 'AbsTol', 1e-12); % Skew-symmetric property

            cross_mat = Sv * w;
            cross_vec = cross(v, w);
            testCase.verifyEqual(cross_mat, cross_vec, 'AbsTol', 1e-12);
        end

        function testSpatialTwistAdjoint(testCase)
            % Test SO3R3_R66_twist adjoint matrix properties
            R = [0, 0, 1; 0, 1, 0; -1, 0, 0];
            r = [0.1; 0.2; 0.3];

            X = robotics.math.SO3R3_R66_twist(R, r);
            testCase.verifySize(X, [6, 6]);
            testCase.verifyEqual(X(1:3, 1:3), R);
            testCase.verifyEqual(X(4:6, 4:6), R);
            testCase.verifyEqual(X(4:6, 1:3), zeros(3, 3));
            testCase.verifyEqual(X(1:3, 4:6), robotics.math.SkewSym(r) * R, 'AbsTol', 1e-12);
        end

        function testEulerAngleRoundtrips(testCase)
            % Test getRotMatfromEA and getEulerPosVec consistency across all 4 sequences
            angles_test = [0.3; -0.4; 0.5]; % [phi, theta, psi]

            for seq = 1:4
                % 1: ZYZ, 2: ZYX, 3: XYZ, 4: ZXZ
                R = robotics.math.getRotMatfromEA(angles_test, seq);
                testCase.verifySize(R, [3, 3]);
                testCase.verifyEqual(R * R', eye(3), 'AbsTol', 1e-10); % Orthogonality
                testCase.verifyEqual(det(R), 1.0, 'AbsTol', 1e-10);    % Proper rotation

                extracted_angles = robotics.math.getEulerPosVec(R, seq);
                testCase.verifySize(extracted_angles, [3, 1]);

                % Check that reconstructed rotation matches original
                R_reconstructed = robotics.math.getRotMatfromEA(extracted_angles, seq);
                testCase.verifyEqual(R_reconstructed, R, 'AbsTol', 1e-8);
            end
        end

        function testInertiaTensorSymmetrizationAndDotMat(testCase)
            % Test DotMat * SymVec(I) == I * r
            Ixx = 2.5; Ixy = 0.1; Ixz = -0.2;
            Iyy = 3.0; Iyz = 0.3; Izz = 1.8;

            I = robotics.math.symmetrize(Ixx, Ixy, Ixz, Iyy, Iyz, Izz);
            testCase.verifySize(I, [3, 3]);
            testCase.verifyEqual(I, I'); % Symmetric

            sym_v = robotics.math.SymVec(I);
            testCase.verifySize(sym_v, [6, 1]);

            r = [0.4; -0.5; 0.6];
            D = robotics.math.DotMat(r);
            testCase.verifySize(D, [3, 6]);

            testCase.verifyEqual(D * sym_v, I * r, 'AbsTol', 1e-12);
        end

        function testModifiedDenavitHartenbergVectors(testCase)
            % Test getRi_j, getri_j_vec, getrj_i_vec
            alpha = pi/3;
            theta = -pi/4;
            a = 0.25;
            d = 0.15;

            Ri_j = robotics.math.getRi_j(alpha, theta);
            testCase.verifySize(Ri_j, [3, 3]);
            testCase.verifyEqual(Ri_j * Ri_j', eye(3), 'AbsTol', 1e-12);

            ri_j = robotics.math.getri_j_vec(alpha, a, d);
            testCase.verifySize(ri_j, [3, 1]);
            testCase.verifyEqual(ri_j, [a; -d*sin(alpha); d*cos(alpha)], 'AbsTol', 1e-12);

            rj_i = robotics.math.getrj_i_vec(theta, a, d);
            testCase.verifySize(rj_i, [3, 1]);
            testCase.verifyEqual(rj_i, [-a*cos(theta); a*sin(theta); -d], 'AbsTol', 1e-12);
        end

    end
end
