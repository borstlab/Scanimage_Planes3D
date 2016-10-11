function openROIManager
global  gh
if isfield(gh,'roiManagerGUI')
    seeGUI('gh.roiManagerGUI.figure1');
else
    startROIManager;
    seeGUI('gh.roiManagerGUI.figure1');
end

