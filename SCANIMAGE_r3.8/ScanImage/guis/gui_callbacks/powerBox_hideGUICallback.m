function powerBox_hideGUICallback()
%CYCLEGUI_HIDEGUICALLBACK Called when hideGUI() is used to close the powerBox GUI
	global gh;
	
	set(gh.powerControl.tbShowPowerBox,'Value',false);
	set(gh.powerControl.tbShowPowerBox,'String','>>');
end
