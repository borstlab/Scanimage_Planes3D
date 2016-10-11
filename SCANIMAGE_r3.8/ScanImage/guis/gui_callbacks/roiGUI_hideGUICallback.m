function roiGUI_hideGUICallback()
%CYCLEGUI_HIDEGUICALLBACK Called when hideGUI() is used to close the ncycleGUI
	global gh;
	
	set(gh.mainControls.tbToggleROI,'Value',false);
	set(gh.mainControls.tbToggleROI,'String','ROI >>');
end

