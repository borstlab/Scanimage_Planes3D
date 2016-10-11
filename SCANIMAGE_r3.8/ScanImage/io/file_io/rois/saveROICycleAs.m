function saveROICycleAs
global state
if isdir(state.roiCycle.roiCyclePath)
    cd(state.roiCycle.roiCyclePath);
end
[fname,pname]=uiputfile('*.rcf','Select File Name and Path...');
if isnumeric(fname)
    return
else
    state.roiCycle.roiCyclePath=pname;
    state.roiCycle.roiCycleName=[fname '.rcf'];
    updateGUIByGlobal('state.roiCycle.roiCycleName');
    fn=fullfile(state.roiCycle.roiCyclePath,state.roiCycle.roiCycleName);
    currentROICycle=state.roiCycle.currentROICycle;
    save(fn,'currentROICycle','-mat');
    state.roiCycle.roiCycleSaved=1;
    scanimagepath=fileparts(which('scanimage'));
    roiCyclePath=state.roiCycle.roiCyclePath;
    save(fullfile(scanimagepath,['lastroiCyclePath.mat']),'roiCyclePath', '-mat');
end