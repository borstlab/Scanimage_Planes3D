function saveROICycle
global state
fn=fullfile(state.roiCycle.roiCyclePath,state.roiCycle.roiCycleName);
if exist(fn)==2
    currentROICycle=state.roiCycle.currentROICycle;
    save(fn,'currentROICycle','-mat');
    state.roiCycle.roiCycleSaved=1;
    scanimagepath=fileparts(which('scanimage'));
    roiCyclePath=state.roiCycle.roiCyclePath;
    save(fullfile(scanimagepath,['lastroiCyclePath.mat']),'roiCyclePath', '-mat');
else
    saveROICycleAs;
end
