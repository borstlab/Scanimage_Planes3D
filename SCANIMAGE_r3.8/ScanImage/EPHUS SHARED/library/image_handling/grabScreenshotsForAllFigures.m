% grabScreenshotsForAllFigures - Grab a screen shot of every visible figure.
%
% SYNTAX
%  grabScreenshotsForAllFigures
%  grabScreenshotsForAllFigures(fmt)
%   fmt - A Matlab support image file format (default is 'png').
%
% USAGE
%
% NOTES
%  The GUI must be onscreen and visible in order to be captured. (Not anymore - TO120207B).
%
%  May grab hidden GUIs.
%
% CHANGES
%  TO12207B - Automatically move the gui to the main window and make it visible, if necessary. Restore to original state when done. -- Tim O'Connor 12/2/07
%
% Created 8/30/07 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2007
function grabScreenshotsForAllFigures(varargin)

try
    defaultDir = getDefaultCacheDirectory(progmanager, 'grabScreenshotsForAllFiguresDir');
catch
    defaultDir = pwd;
end
outputDir = uigetdir(defaultDir, 'Choose a directory in which to dump screenshots.');
if outputDir == 0
    return;
end
try
    setDefaultCacheValue(progmanager, 'grabScreenshotsForAllFiguresDir', outputDir);
catch
    %Ignore.
end

if isempty(varargin)
    fmt = 'png';
else
    fmt = varargin{1};
end

promptedForOverwrite = 0;
overwrite = 0;

kids = allchild(0);
for i = 1 : length(kids)
    if strcmpi(get(kids(i), 'Type'), 'figure')
        handleVisibility = get(kids(i), 'HandleVisibility');
        set(kids(i), 'HandleVisibility', 'On');
        if strcmpi(get(kids(i), 'Visible'), 'On')
            fname = fullfile(outputDir, [get(kids(i), 'Name') '.' fmt]);
            try
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
                
                %TO120207B
                % %Prep the figure for capture.
                % if strcmpi(get(kids(i), 'Visible'), 'On')
                %     %Make sure the gui is in the foreground.
                %     set(kids(i), 'Visible', 'Off');
                %     set(kids(i), 'Visible', 'On');
                %     %Make sure the gui is on the screen.
                %     movegui(kids(i));
                % end
                
                %Grab and save the screen shot.
                grabFullFigureScreenshot(kids(i), fname, fmt);
                
                fprintf(1, ' Saved %s to %s\n', get(kids(i), 'Name'), fname);
            catch
                fprintf(2, 'Failed to capture/save %s to %s - %s\n', get(kids(i), 'Name'), fname, lasterr);
            end
        end
        set(kids(i), 'HandleVisibility', handleVisibility);
    end
end

return;