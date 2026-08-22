% UPDATEAPPCODE Repackages MainApp.mlapp and MainApp2.mlapp with latest code from their _code.txt files.
% Run this in MATLAB whenever MainApp_code.txt or MainApp2_code.txt is modified.

disp('Closing all active figures and clearing classes...');
close all force;
clear classes;

appsToUpdate = {'MainApp2'};

for idx = 1:length(appsToUpdate)
    appName = appsToUpdate{idx};
    mlappFile = fullfile(pwd, [appName, '.mlapp']);
    codeFile  = fullfile(pwd, [appName, '_code.txt']);
    
    if ~isfile(mlappFile) || ~isfile(codeFile)
        continue;
    end
    
    fprintf('Updating %s.mlapp from %s_code.txt...\n', appName, appName);
    code = fileread(codeFile);
    
    docXml = ['<?xml version="1.0" encoding="UTF-8" standalone="no" ?>' ...
        '<w:document xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main">' ...
        '<w:body><w:p><w:pPr><w:pStyle w:val="code"/></w:pPr><w:r><w:t><![CDATA[' ...
        code ']]></w:t></w:r></w:p></w:body></w:document>'];
    
    tempDir = fullfile(pwd, ['temp_', appName, '_extract']);
    if isfolder(tempDir)
        rmdir(tempDir, 's');
    end
    mkdir(tempDir);
    
    unzip(mlappFile, tempDir);
    
    docPath = fullfile(tempDir, 'matlab', 'document.xml');
    fid = fopen(docPath, 'w', 'native', 'UTF-8');
    if fid == -1
        error('Failed to open %s for writing.', docPath);
    end
    fwrite(fid, docXml, 'char');
    fclose(fid);
    
    zip(fullfile(pwd, [appName, '_new.zip']), '*', tempDir);
    rmdir(tempDir, 's');
    movefile(fullfile(pwd, [appName, '_new.zip']), mlappFile, 'f');
    fprintf('%s.mlapp successfully updated!\n', appName);
end

disp('All available app files updated successfully.');
