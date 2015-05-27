function [ targetDir ] = ephus_util_cloneSettings( varargin )
%EPHUS_UTIL_CLONESETTINGS Clones a settings folder tree
%   Copies the contents of a settings folder (recursively) to a new
%   location, modifying embedded file paths as appropriate. If any of the
%   existing settings files contain paths that point to a location within
%   the existing settings tree, these paths are modified to point to the
%   (new) location of the cloned files.
%
%   NOTE: Any file locations that are not within the settings folder tree
%   will remain unchanged (likely this includes the XSG default directory).
%
%   If no arguments are specified, folders are selected via dialogs.
%   Optionally, two arguments are accepted: 
%     1. source directory
%     2. target directory
%
%   E.g. sourceDir: C:/DATA/Ben/settings
%        targetDir: C:/DATA/Tom/settings
%
% 2010-04-07 -- Ben Suter
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if nargin == 2
    sourceDir = varargin{1};
    if ~isdir(sourceDir)
        error('The first argument must be a (source) directory.');
    end
    
    targetDir = varargin{2};
    if ~isdir(targetDir)
        error('The second argument must be a (target) directory.');
    end
else
    startPath = cd;
    sourceDir = uigetdir(startPath, 'Select source directory containing existing settings');
    if sourceDir == 0
        return;
    end
    
    targetDir = uigetdir(startPath, 'Select (empty) target directory for cloned settings');
    if targetDir == 0
        return;
    end
end

if strcmp(targetDir, sourceDir)
    error('Target and source directories must be different');
end

list = dir(fullfile(targetDir, '*'));
if numel(list) > 2
    % An empty directory listing contains two items: "." and ".."
    error('The target directory must be empty');
end

% First, copy everything over, recursively
[status, message, messageid] = copyfile(sourceDir, targetDir);
if ~status
    error('An error occurred while cloning settings: %s', message);
end

% Next, modify any .settings files
listing = dir(targetDir);
modifyFiles(targetDir, listing, sourceDir, targetDir);

end

function modifyFiles(thisDir, listing, sourceDir, targetDir)
%     extension = '.settings';
%     numExtensionChars = numel(extension);
    for i=1:numel(listing)
        entry = listing(i);
        if strcmp(entry.name, '.') || strcmp(entry.name, '..')
            continue;
        elseif entry.isdir
            subDir = [thisDir filesep entry.name];
            subListing = dir(subDir);
            modifyFiles(subDir, subListing, sourceDir, targetDir);
        elseif matchExtension(entry.name, 'settings')
            % If this is a settings file, modify it
            vars = load(fullfile(thisDir, entry.name), '-mat');
            if isfield(vars, 'settings')
                settings = vars.('settings');
                settings = changePaths(settings, sourceDir, targetDir);
                save(fullfile(thisDir, entry.name), 'settings', '-mat');
            end
        end
    end   
end

function [ isMatch ] = matchExtension(filename, extension)
% Returns true if filename ends with .extension
%
    isMatch = strncmp(fliplr(filename), fliplr(['.' extension]), numel(extension) + 1);
end


function [ settings ] = changePaths(settings, sourceDir, targetDir)
% Any settings in this hard-coded list are modified, but only if their
% value falls within the sourceDir tree.
%
    settings = changePath(settings, 'ephys', 'pulseSetDir', sourceDir, targetDir);
    settings = changePath(settings, 'ephys', 'pulseFile', sourceDir, targetDir);
    settings = changePath(settings, 'ephys', 'pulsePath', sourceDir, targetDir);
    settings = changePath(settings, 'mapper', 'mapPatternDirectory', sourceDir, targetDir);
    settings = changePath(settings, 'pulseEditor', 'directory', sourceDir, targetDir);    
    settings = changePath(settings, 'pulseJacker', 'pulsePath', sourceDir, targetDir);
    settings = changePath(settings, 'pulseJacker', 'cyclePath', sourceDir, targetDir);
    settings = changePath(settings, 'qcam', 'outputFile', sourceDir, targetDir);  
    settings = changePath(settings, 'stimulator', 'pulseSetDir', sourceDir, targetDir);
    settings = changePath(settings, 'traceViewer', 'filename', sourceDir, targetDir);
    settings = changePath(settings, 'traceViewer', 'defaultDirectory', sourceDir, targetDir);
    settings = changePath(settings, 'xsg', 'directory', sourceDir, targetDir); % unlikely to be within a settings tree
%     settings = changePath(settings, 'xsg', 'initials', sourceDir, targetDir);
    
    if isfield(settings, 'hotswitch')
        settings = changePath(settings, 'ephys', 'pulseSetDir', sourceDir, targetDir);
        states = get(settings.('hotswitch'), 'states');
        for i=1:numel(states)
            states(i).directory = strrep(states(i).directory, sourceDir, targetDir);
        end
        set(settings.('hotswitch'), 'states', states);
    end
    
    % Potentially, we could do an automated search of all fields
    % via reflection and detect cases where a
    % field value appears to be a file or directory path.  
end

function [ settings ] = changePath(settings, objName, fieldName, sourceDir, targetDir)
    if ~isfield(settings, objName)
        return;
    end
    try
        val = get(settings.(objName), fieldName);
        set(settings.(objName), fieldName, strrep(val, sourceDir, targetDir));
    catch ME
        % the field does not exist in this settings object, so ignore
    end
end




