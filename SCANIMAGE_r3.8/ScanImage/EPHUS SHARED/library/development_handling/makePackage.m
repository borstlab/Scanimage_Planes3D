% makePackage
%
%  This function will collect a distributable package of software, with an automated install procedure.
%
% Created 8/22/05 - Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function makePackage

[f p] = uiputfile('*.svlp', 'Specify a package file.');
if length(f) == 1 & length(p) == 1
    if f == 0 & p == 0
        return;
    end
end
if ~endsWithIgnoreCase(f, '.svlp')
    f = [f '.svlp'];
end
outputfile = fullfile(p, f);

rootdir = uigetdir(fullfile(matlabroot, 'work', 'svobodalab'), 'Choose a directory from which to construct a package.');
if length(rootdir) == 1
    if rootdir == 0
        return;
    end
end

subDirs = {'library', 'Programs', 'USERCLASSES', 'documentation'};
contents = dir(rootdir);
contents = contents(3:end);%Trim off '.' and '..'

%Check for out of the ordinary objects to be packaged.
looseFiles = 0;
extraDirectories = 0;
foundDirs = {};
for i = 1 : length(contents)
    if ~contents(i).isdir
        looseFiles = 1;
    elseif ~ismember(lower(contents(i).name), lower(subDirs)) ...
            & ~(stcmpi(contents(i).name, '.') | stcmpi(contents(i).name, '..'))
        extraDirectories = 1;
    else
        foundDirs{length(foundDirs) + 1} = contents(i).name;
    end
end

%Check what the user wants to do with stuff found in the path.
confirmMsg = '';
if length(foundDirs) < length(subDirs)
    confirmMsg = sprintf('Continue package creation?\nMissing expected directories:\n');
    for i = 1 : length(subDirs)
        if ~ismember(lower(subDirs{i}), lower(foundDirs))
            confirmMsg = sprintf('%s %s\n', confirmMsg, subDirs{i});
        end
    end
end
% yesOrNo = questdlg(confirmMsg, 'Missing expected directories.', 'Yes');
yesOrNo = questdlg('Missing expected directories. Continue packaging?', 'Missing expected directories.', 'Yes');
if ~strcmpi(yesOrNo, 'Yes')
    return;
end

collectLooseFiles = 0;
if looseFiles
    yesOrNo = questdlg('Include loose files in package?', 'Loose files found.', 'No');
    if strcmpi(yesOrNo, 'Yes')
        collectLooseFiles = 1;
    elseif strcmpi(yesOrNo, 'Cancel')
        return;
    end
end

collectExtraDirectories = 0;
if extraDirectories
    yesOrNo = questdlg('Collect extra directories in package?', 'Extra directories found.', 'No');
    if strcmpi(yesOrNo, 'Yes')
        collectExtraDirectories = 1;
    elseif strcmpi(yesOrNo, 'Cancel')
        return;
    end
end

fileList = {};
for i = 1 : length(contents)
    if ~contents(i).isdir & collectLooseFiles
        fileList{length(fileList) + 1} = contents(i).name;
        fprintf(1, 'Adding loose file ''%s'' to package...\n', fullfile(rootdir, contents(i).name));
    elseif ~ismember(lower(contents(i).name), lower(subDirs)) & collectExtraDirectories ...
            & ~(stcmpi(contents(i).name, '.') | stcmpi(contents(i).name, '..'))
        fileList{length(fileList) + 1} = contents(i).name;
        fprintf(1, 'Adding extra directory ''%s'' to package...\n', fullfile(rootdir, contents(i).name));
    else
        fileList{length(fileList) + 1} = contents(i).name;
        fprintf(1, 'Adding ''%s'' to package...\n', fullfile(rootdir, contents(i).name));
    end
end

timestamp = datestr(datevec(now), 30);
outputfile = [outputfile(1 : length(outputfile) - 5) '_' timestamp '.svlp.zip'];
fprintf(1, 'Creating zip compressed archive ''%s''...\n', outputfile);

zip(outputfile, fileList, rootdir);

fprintf(1, 'Package completed.\n\n');

return;