function newVer = bumpVersion(type, createGitTag)
    % BUMPVERSION Automates Semantic Versioning for the project.
    %
    % Usage:
    %   bumpVersion('patch')        % 1.0.0 -> 1.0.1 (bug fixes)
    %   bumpVersion('minor')        % 1.0.0 -> 1.1.0 (new features)
    %   bumpVersion('major')        % 1.0.0 -> 2.0.0 (breaking changes)
    %   bumpVersion('1.2.0')        % Explicitly set version to 1.2.0
    %   bumpVersion('patch', true)  % Also create a git tag (e.g. v1.0.1)
    
    if nargin < 1
        type = 'patch';
    end
    if nargin < 2
        createGitTag = false;
    end
    
    rootDir = pwd;
    versionFile = fullfile(rootDir, 'VERSION');
    readmeFile = fullfile(rootDir, 'README.md');
    
    % Read current version
    if exist(versionFile, 'file')
        fid = fopen(versionFile, 'r');
        currentVer = strtrim(fgetl(fid));
        fclose(fid);
    else
        currentVer = '1.0.0';
    end
    
    tokens = regexp(currentVer, '^(\d+)\.(\d+)\.(\d+)', 'tokens');
    if isempty(tokens)
        error('Invalid current version in VERSION file: %s', currentVer);
    end
    
    major = str2double(tokens{1}{1});
    minor = str2double(tokens{1}{2});
    patch = str2double(tokens{1}{3});
    
    switch lower(type)
        case 'major'
            major = major + 1;
            minor = 0;
            patch = 0;
            newVer = sprintf('%d.%d.%d', major, minor, patch);
        case 'minor'
            minor = minor + 1;
            patch = 0;
            newVer = sprintf('%d.%d.%d', major, minor, patch);
        case 'patch'
            patch = patch + 1;
            newVer = sprintf('%d.%d.%d', major, minor, patch);
        case 'beta'
            betaToken = regexp(currentVer, '-beta\.(\d+)', 'tokens');
            if ~isempty(betaToken)
                bNum = str2double(betaToken{1}{1}) + 1;
            else
                bNum = 1;
            end
            newVer = sprintf('%d.%d.%d-beta.%d', major, minor, patch, bNum);
        case 'rc'
            rcToken = regexp(currentVer, '-rc\.(\d+)', 'tokens');
            if ~isempty(rcToken)
                rcNum = str2double(rcToken{1}{1}) + 1;
            else
                rcNum = 1;
            end
            newVer = sprintf('%d.%d.%d-rc.%d', major, minor, patch, rcNum);
        case 'alpha'
            alphaToken = regexp(currentVer, '-alpha\.(\d+)', 'tokens');
            if ~isempty(alphaToken)
                aNum = str2double(alphaToken{1}{1}) + 1;
            else
                aNum = 1;
            end
            newVer = sprintf('%d.%d.%d-alpha.%d', major, minor, patch, aNum);
        case 'release'
            newVer = sprintf('%d.%d.%d', major, minor, patch);
        otherwise
            % Explicit version string provided (e.g. '1.0.0-beta.1')
            if isempty(regexp(type, '^\d+\.\d+\.\d+', 'once'))
                error('Type must be ''major'', ''minor'', ''patch'', ''beta'', ''rc'', ''alpha'', ''release'', or a valid SemVer string.');
            end
            newVer = type;
    end
    
    % 1. Write to VERSION file
    fid = fopen(versionFile, 'w');
    if fid == -1
        error('Could not open VERSION file for writing.');
    end
    fprintf(fid, '%s\n', newVer);
    fclose(fid);
    fprintf('Updated VERSION file: %s -> %s\n', currentVer, newVer);
    
    % 2. Update README.md badge if it exists
    if exist(readmeFile, 'file')
        readmeText = fileread(readmeFile);
        oldBadgePattern = 'Toolbox-v[\d\.]+-green\.svg';
        newBadge = sprintf('Toolbox-v%s-green.svg', newVer);
        updatedReadme = regexprep(readmeText, oldBadgePattern, newBadge);
        
        fid = fopen(readmeFile, 'w');
        if fid ~= -1
            fwrite(fid, updatedReadme);
            fclose(fid);
            fprintf('Updated README.md version badge to v%s\n', newVer);
        end
    end
    
    % Clear persistent cached version in robotics.version
    clear robotics.version;
    
    % 3. Optional Git Tag creation
    if createGitTag
        try
            tagName = sprintf('v%s', newVer);
            [status, out] = system(sprintf('git tag -a %s -m "Release %s"', tagName, tagName));
            if status == 0
                fprintf('Created Git tag: %s\n', tagName);
            else
                fprintf('Git tag creation warning: %s\n', out);
            end
        catch ME
            fprintf('Could not create git tag: %s\n', ME.message);
        end
    end
    
    fprintf('\nVersion bump complete! Next release: v%s\n', newVer);
end
