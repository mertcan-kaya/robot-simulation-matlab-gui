function v = version()
    % ROBOTICS.VERSION Returns the current version of the Robot Simulation GUI package.
    % Reads dynamically from the root VERSION file.
    pkgDir = fileparts(mfilename('fullpath')); % .../+robotics
    rootDir = fileparts(pkgDir);               % repository root
    versionFile = fullfile(rootDir, 'VERSION');
    
    if exist(versionFile, 'file')
        fid = fopen(versionFile, 'r');
        if fid ~= -1
            raw = fgetl(fid);
            fclose(fid);
            if ischar(raw) && ~isempty(strtrim(raw))
                v = strtrim(raw);
                return;
            end
        end
    end
    
    v = '1.0.0';
end
