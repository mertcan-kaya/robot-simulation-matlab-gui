classdef TestMeshCache < matlab.unittest.TestCase
    % TESTMESHCACHE Unit tests for robotics.graphics.MeshCache

    methods (TestMethodSetup)
        function clearCacheBeforeTest(testCase)
            robotics.graphics.MeshCache.clearCache();
        end
    end

    methods (Test)
        function testMeshLoadingAndCaching(testCase)
            % Test path for link0 of Franka Emika
            meshPath = 'meshes/fer/visual/link0.mat';

            % Initial state: not cached
            testCase.verifyFalse(robotics.graphics.MeshCache.isMeshCached(meshPath));
            testCase.verifyEqual(robotics.graphics.MeshCache.getCachedMeshCount(), 0);

            % Load mesh
            ms1 = robotics.graphics.MeshCache.getMesh(meshPath);

            % Verify data integrity
            testCase.verifyTrue(isfield(ms1, 'F'));
            testCase.verifyTrue(isfield(ms1, 'V'));
            testCase.verifyTrue(isfield(ms1, 'C'));
            testCase.verifyNotEmpty(ms1.V);
            testCase.verifyNotEmpty(ms1.F);

            % Verify mesh is now cached in RAM
            testCase.verifyTrue(robotics.graphics.MeshCache.isMeshCached(meshPath));
            testCase.verifyEqual(robotics.graphics.MeshCache.getCachedMeshCount(), 1);

            % Subsequent retrieval from cache returns identical structure
            ms2 = robotics.graphics.MeshCache.getMesh(meshPath);
            testCase.verifyEqual(ms1.V, ms2.V);
            testCase.verifyEqual(ms1.F, ms2.F);
        end

        function testClearCache(testCase)
            % Load two meshes
            robotics.graphics.MeshCache.getMesh('meshes/fer/visual/link0.mat');
            robotics.graphics.MeshCache.getMesh('meshes/fer/visual/link1.mat');
            testCase.verifyEqual(robotics.graphics.MeshCache.getCachedMeshCount(), 2);

            % Clear cache
            robotics.graphics.MeshCache.clearCache();
            testCase.verifyEqual(robotics.graphics.MeshCache.getCachedMeshCount(), 0);
        end
    end
end
