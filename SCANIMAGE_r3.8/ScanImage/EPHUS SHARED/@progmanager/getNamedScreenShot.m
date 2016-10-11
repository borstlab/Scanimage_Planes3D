% @progmanager/getNamedScreenShot - Grab a screen shot of the indicated gui.
%
% SYNTAX
%  getNamedScreenShot(progmanager, programName)
%  getNamedScreenShot(progmanager, programName, guiName)
%   programName - The name of the program to capture.
%   guiName - The name of the gui to capture.
%             If this is unspecified, just the main gui is captured.
%
% USAGE
%
% NOTES
%  Gets the image format from 'screenshotExportFormat' in the progmanager default cache.
%  Changing the value there is one way to change the format without requiring prompting of the user.
%
%  The requested gui will be moved to the primary monitor, which is required for a screenshot. Then replaced. (Not anymore - TO120207B)
%
% CHANGES
%  TO120207B - grabFullFigureScreenshot now handles moving guis, if necessary. -- Tim O'Connor 12/2/07
%
% Created 12/2/07 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2007
function getNamedScreenShot(this, programName, varargin)
global progmanagerglobal;

if isempty(varargin)
    guiName = programName;
else
    guiName = varargin{1};
end

imageFormat = getDefaultCacheValue(this, 'screenshotExportFormat');
if isempty(imageFormat)
    imageFormat = 'png';
end

outputPath = getDefaultCacheDirectory(this, 'screenshotExportDir');
outputFile = [programName '_' guiName '.' imageFormat];

%Prompt for an output file, with a nice default choice.
[outputFile, outputDir] = uiputfile({'*.png', 'Portable Network Graphics (PNG)'; ...
                                     '*.gif', 'Graphics Interchange Format (GIF)'; ...
                                     '*.tif', 'Tagged Image File Format (TIFF)'; ...
                                     '*.tiff', 'Tagged File Format (TIFF)'; ...
                                     '*.jpg', 'Joint Photographic Experts Group (JPEG)'; ...
                                     '*.*', 'All Files (*.*)'}, ...
    'Choose a file in which to save this screenshot.', fullfile(outputPath, outputFile));
if length(outputDir) == 1
    if outputDir == 0
        return;
    end
elseif length(outputFile) == 1
    if outputFile == 0
        return;
    end
end
setDefaultCacheValue(this, 'screenshotExportDir', outputDir);

%Find out what format was chosen.
[p, f, imageFormat] = fileparts(outputFile);
if startsWith(imageFormat, '.')
    imageFormat = imageFormat(2:end);
end
setDefaultCacheValue(this, 'screenshotExportFormat', imageFormat);

fname = fullfile(outputPath, outputFile);
try
    handleVisibility = get(progmanagerglobal.programs.(programName).guinames.(guiName).fighandle, 'HandleVisibility');
    set(progmanagerglobal.programs.(programName).guinames.(guiName).fighandle, 'HandleVisibility', 'On');

    %TO120207B
    % %Prep the figure for capture.
    % %Make sure the gui is in the foreground.
    % set(progmanagerglobal.programs.(programName).guinames.(guiName).fighandle, 'Visible', 'Off');
    % set(progmanagerglobal.programs.(programName).guinames.(guiName).fighandle, 'Visible', 'On');
    % %Make sure the gui is on the screen.
    % movegui(progmanagerglobal.programs.(programName).guinames.(guiName).fighandle);
    % %TO120207A - On some systems, there's a lag before the graphics are properly drawn.
    % drawnow;
    
    %Grab and save the screen shot.
    grabFullFigureScreenshot(progmanagerglobal.programs.(programName).guinames.(guiName).fighandle, fname, imageFormat);

    set(progmanagerglobal.programs.(programName).guinames.(guiName).fighandle, 'HandleVisibility', handleVisibility);

    fprintf(1, ' Saved %s:%s to %s\n', programName, guiName, fname);
catch
    fprintf(2, 'Failed to capture/save %s:%s to %s\n%s\n', programName, guiName, fname, getLastErrorStack);
end

return;