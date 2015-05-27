function cycleGUI_hideGUICallback()
%CYCLEGUI_HIDEGUICALLBACK Called when hideGUI() is used to close the ncycleGUI
	global gh;
	
	set(gh.mainControls.tbCycleControls,'Value',false);
	set(gh.mainControls.tbCycleControls,'String','CYC>>');
	
	hideGUI('gh.metaStackGUI.figure1');
end

