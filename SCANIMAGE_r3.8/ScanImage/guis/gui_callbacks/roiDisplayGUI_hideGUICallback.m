function cycleGUI_hideGUICallback()
%ROIDISPAYGUI_HIDEGUICALLBACK Called when hideGUI() is used to close the RDF.
	global gh;
	
	set(gh.roiGUI.tbROIDisplay,'Value',false);
end

