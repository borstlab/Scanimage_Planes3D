function hideGUI(varargin)

global gh;

assert(nargin > 0);

if ~ishandle(varargin{1})
    GUI = varargin{1};
	[topName, s, f]=structNameParts(GUI);
	eval(['global ' topName]);
	eval(['set(' GUI ', ''Visible'', ''off'')']);
	
	% take care of the 'close' callback, if it exists
	% NOTE: to implement a callback, create a file named 
	% '<GUINAME>_hideGUICallback.m in the './ScanImage/guis/gui_callbacks/' 
	% folder, where <GUINAME> is a valid ScanImage GUI tag.
	periods = strfind(GUI,'.');
    if isempty(periods)
        guiName = GUI;
    else
        guiName = GUI((periods(end-1)+1:periods(end)-1));
    end
    if exist([guiName '_hideGUICallback'],'file')
		eval([guiName '_hideGUICallback']);
    end
    
    guiHandle = gh.(guiName).figure1;
else %Direct callback, first argument is source handle
    guiHandle = varargin{1};
end

set(guiHandle,'Visible','off');