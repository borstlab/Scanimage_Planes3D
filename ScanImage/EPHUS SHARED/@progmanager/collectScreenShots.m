% @progmanager/collectScreenshots - Grab screen shots of all visible programs.
%
% SYNTAX
%  collectScreenshots(progmanager)
%
% USAGE
%
% NOTES
%  Gets the image format from 'screenshotExportFormat' in the progmanager default cache.
%  Changing the value there is one way to change the format without requiring prompting of the user.
%
%  All visible GUIs will be moved to the primary monitor, which is required for a screenshot. (Not anymore - TO120207B)
%
% CHANGES
%  TO120207B - grabFullFigureScreenshot now handles moving guis, if necessary. -- Tim O'Connor 12/2/07
%
% Created 8/30/07 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2007
function collectScreenshots(this)
global progmanagerglobal;

outputDir = uigetdir(getDefaultCacheDirectory(this, 'screenshotExportDir'), 'Choose a directory in which to dump screenshots.');
if outputDir == 0
    return;
end
setDefaultCacheValue(this, 'screenshotExportDir', outputDir);
imageFormat = getDefaultCacheValue(this, 'screenshotExportFormat');
if isempty(imageFormat)
    imageFormat = 'png';
end
setDefaultCacheValue(this, 'screenshotExportFormat', imageFormat);

promptedForOverwrite = 0;
overwrite = 0;

fprintf(1, 'Exporting screenshots...\n');
programs = fieldnames(progmanagerglobal.programs);
for i = 1 : length(programs)
    guis = fieldnames(progmanagerglobal.programs.(programs{i}).guinames);
    for j = 1 : length(guis)
        try
            fname = fullfile(outputDir, [programs{i} '_' guis{j} '.' imageFormat]);
            %Test for file overwriting.
            if exist(fname) == 2
                if ~promptedForOverwrite
                    overwriteResponse = questdlg(sprintf('''%s'' exists.\nOverwrite?', fname), 'Overwrite screenshot(s)?', 'Overwrite All', 'Overwrite One', 'Skip', 'Overwrite All');
                    if strcmpi(overwriteResponse, 'Overwrite All')
                        overwrite = 1;
                        promptedForOverwrite = 1;
                    elseif strcmpi(overwriteResponse, 'Overwrite One')
                        overwrite = 1;
                        promptedForOverwrite = 0;
                    else
                        overwrite = 0;
                    end
                end
                if ~overwrite
                    continue;
                end
            end
            
            handleVisibility = get(progmanagerglobal.programs.(programs{i}).guinames.(guis{j}).fighandle, 'HandleVisibility');
            set(progmanagerglobal.programs.(programs{i}).guinames.(guis{j}).fighandle, 'HandleVisibility', 'On');

            % %Prep the figure for capture.
            % if strcmpi(get(progmanagerglobal.programs.(programs{i}).guinames.(guis{j}).fighandle, 'Visible'), 'On')
            %     %Make sure the gui is in the foreground.
            %     set(progmanagerglobal.programs.(programs{i}).guinames.(guis{j}).fighandle, 'Visible', 'Off');
            %     set(progmanagerglobal.programs.(programs{i}).guinames.(guis{j}).fighandle, 'Visible', 'On');
            %     %Make sure the gui is on the screen.
            %     movegui(progmanagerglobal.programs.(programs{i}).guinames.(guis{j}).fighandle);  
            %     %TO120207A - On some systems, there's a lag before the graphics are properly drawn.
            %     drawnow;
            %     pause(0.25);
            % end
    
            %Grab and save the screen shot.
            grabFullFigureScreenshot(progmanagerglobal.programs.(programs{i}).guinames.(guis{j}).fighandle, fname, imageFormat);
            
            set(progmanagerglobal.programs.(programs{i}).guinames.(guis{j}).fighandle, 'HandleVisibility', handleVisibility);
            
            fprintf(1, ' Saved %s:%s to %s\n', programs{i}, guis{j}, fname);
        catch
            fprintf(2, 'Failed to capture/save %s:%s to %s - %s\n', programs{i}, guis{j}, fname, lasterr);
        end
    end
end

return;