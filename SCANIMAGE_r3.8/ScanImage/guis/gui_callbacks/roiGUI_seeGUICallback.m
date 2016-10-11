function roiGUI_seeGUICallback()
%CYCLEGUI_SEEGUICALLBACK Executes when seeGUI() is used to display the cycleGUI
	global gh;
	
	set(gh.mainControls.tbToggleROI,'Value',true);
    set(gh.mainControls.tbToggleROI,'String','ROI <<');
end

