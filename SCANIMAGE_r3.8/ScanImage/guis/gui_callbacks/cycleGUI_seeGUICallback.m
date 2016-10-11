function cycleGUI_seeGUICallback()
%CYCLEGUI_SEEGUICALLBACK Executes when seeGUI() is used to display the cycleGUI
	global gh;
	
	set(gh.mainControls.tbCycleControls,'Value',true);
end

