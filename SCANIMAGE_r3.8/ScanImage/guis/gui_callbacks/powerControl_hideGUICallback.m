function powerControl_hideGUICallback()
%CYCLEGUI_HIDEGUICALLBACK Called when hideGUI() is used to close the powerControls GUI
	global gh;
	
	hideGUI('gh.powerBox.figure1');
end