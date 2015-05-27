% grabFullFigureScreenshot - Grab a full screen shot of a visible and on-screen figure and save as a file.
%
% SYNTAX
%  grabFullFigureScreenshot(f, fname, fmt)
%   f - The figure to grab.
%   fname - The output file.
%   fmt - A Matlab supported image file format.
%
% USAGE
%
% NOTES
%  The GUI must be onscreen and visible in order to be captured.
%
% CHANGES
%  TO12207B - Automatically move the gui to the main window and make it visible, if necessary. Restore to original state when done. -- Tim O'Connor 12/2/07
%
% Created 8/30/07 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2007
function grabFullFigureScreenshot(f, fname, fmt)

%Check out the validity of the figure handle.
if ~ishandle(f)
    error('Capturing a screenshot requires a valid figure handle.');
end
if ~strcmpi(get(f, 'Type'), 'figure')
    error('Capturing a screenshot requires a handle to a figure, other GUI objects are not acceptable: ''%s''', get(f, 'Type'));
end

%TO112207B
originalPosition = get(f, 'Position');
originalVisibility = get(f, 'Visible');
originalHandleVisibility = get(f, 'HandleVisibility');
set(f, 'HandleVisibility', 'On');
%Prep the figure for capture.
%Make sure the gui is in the foreground.
set(f, 'Visible', 'Off');
set(f, 'Visible', 'On');
%Make sure the gui is on the screen.
movegui(f);
%Render to framebuffer.
drawnow expose;%TO042309A - Not using expose can cause C-spawned events to fire out of order.
pause(0.25);%On some systems, there's a lag before the graphics are properly drawn.
drawnow expose;%TO042309A - Not using expose can cause C-spawned events to fire out of order.

%Work out the idiotic dimensioning crap, or else it won't grab the whole window.
units = get(f, 'Units');
set(f, 'Units', 'pixels');
outerPos = get(f, 'OuterPosition');%The not-so-documented 'OuterPosition' property makes things nice.
pos = zeros(4, 1);
pos(3:4) = outerPos(3:4) - [5, 3];%There's still a quirk, hence the empirically determined subtraction.
set(f, 'Units', units);

%Some axes seem to get lost if it's not the current figure.
cf = get(0, 'CurrentFigure');
set(0, 'CurrentFigure', f);

%Grab the screenshot, making sure to include the entire figure.
screenshot = getframe(f, pos);

if ~endsWithIgnoreCase(fname, ['.' fmt])
    fname = [fname '.' fmt];
end

%Put the real current figure back.
set(0, 'CurrentFigure', cf);

%Save the screenshot.
if isempty(screenshot.colormap)
    imwrite(screenshot.cdata, fname, fmt);
else
    imwrite(screenshot.cdata, screenshot.colormap, fname, fmt);
end

%TO112207B - Restore original state.
set(f, 'Position', originalPosition);
set(f, 'Visible', originalVisibility);
set(f, 'HandleVisibility', originalHandleVisibility);
drawnow expose;%TO042309A - Not using expose can cause C-spawned events to fire out of order.

return;