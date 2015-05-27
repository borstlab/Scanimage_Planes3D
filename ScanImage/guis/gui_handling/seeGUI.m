function seeGUI(GUI)
% makes a gui visible.  expects string with name of gui
	[topName, s, f]=structNameParts(GUI);
	eval(['global ' topName]);
	eval(['set(' GUI ', ''Visible'', ''on'')']);
	
	% take care of the 'open' callback, if it exists
	% NOTE: to implement a callback, create a file named 
	% '<GUINAME>_seeGUICallback.m in the './ScanImage/guis/gui_callbacks/' 
	% folder, where <GUINAME> is a valid ScanImage GUI tag.
	periods = strfind(GUI,'.');
	guiName = GUI((periods(end-1)+1:periods(end)-1));
	if exist([guiName '_seeGUICallback'])
		eval([guiName '_seeGUICallback']);
	end