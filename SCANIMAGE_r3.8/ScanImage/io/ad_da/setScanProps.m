function setScanProps(h)
%% function setScanProps(h)
%   Final handler for changes to scan properties which require scan output pattern to be recomputed
%% NOTES
%   This was originally specifically for changes to zoom factor or rotation
%   Now, other changes can occur from Main Controls 
%
%   Except for changes during FOCUS itself, this function simply sets the 'updatedZoomOrRot' flag 
%
%% CHANGES
%   VI012509A: Only disable/enable specified control handle if it's supplied -- Vijay Iyer 1/25/09
%   VI082511A: Eliminate varargin option here

global state gh

vis=get(gh.mainControls.focusButton, 'Visible');
if strcmp(vis, 'off')	% Not focusing...cant change these paraemters.
	return
end
%updateCurrentROI; 
if nargin >= 1 && ishandle(h) %VI012509A
    set(h,'Enable','off');
end
state.internal.updatedZoomOrRot=1;
val=get(gh.mainControls.focusButton, 'String');
if strcmp(val, 'ABORT') % focusing now....
    stopAndRestartFocus;
else
    state.internal.updatedZoomOrRot=1;
end
enable='on';

% if nargin > 1
%     while length(varargin) >= 2
%         eval([varargin{1} '=varargin{2};'])
%         varargin=varargin(2:end);
%     end
% end
if nargin >= 1 && ishandle(h)  %VI012509A
    set(h,'Enable',enable);
end
