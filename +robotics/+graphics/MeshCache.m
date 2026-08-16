classdef MeshCache
    % MESHCACHE High-performance in-memory cache for 3D robot CAD meshes.
    % Avoids redundant disk I/O when switching robots or toggling visual modes.

    properties (Constant, Access = private)
        CacheMap = containers.Map('KeyType', 'char', 'ValueType', 'any')
    end

    methods (Static)
        function ms = getMesh(meshPath)
            % GETMESH Retrieves mesh struct from RAM cache, loading from disk on first request.
            normPath = strrep(meshPath, '/', filesep);
            normPath = strrep(normPath, '\', filesep);

            map = robotics.graphics.MeshCache.CacheMap;
            if isKey(map, normPath)
                ms = map(normPath);
            else
                if exist(normPath, 'file')
                    temp = load(normPath);
                    if isfield(temp, 'modelstruct')
                        ms = temp.modelstruct;
                    else
                        ms = temp;
                    end
                    map(normPath) = ms;
                else
                    error('Mesh file not found: %s', normPath);
                end
            end
        end

        function isCached = isMeshCached(meshPath)
            % ISMESHCACHED Checks whether the specified mesh is currently in RAM.
            normPath = strrep(meshPath, '/', filesep);
            normPath = strrep(normPath, '\', filesep);
            isCached = isKey(robotics.graphics.MeshCache.CacheMap, normPath);
        end

        function clearCache()
            % CLEARCACHE Empties the in-memory mesh cache to release RAM if desired.
            map = robotics.graphics.MeshCache.CacheMap;
            allKeys = keys(map);
            if ~isempty(allKeys)
                remove(map, allKeys);
            end
        end

        function count = getCachedMeshCount()
            % GETCACHEDMESHCOUNT Returns the number of unique meshes currently loaded in RAM.
            count = length(robotics.graphics.MeshCache.CacheMap);
        end
    end
end
