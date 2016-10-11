function openROICycle
global state gh

if isdir(state.roiCycle.roiCyclePath)
    cd(state.roiCycle.roiCyclePath);
else    
    scanimagepath=fileparts(which('scanimage'));
    filename=fullfile(scanimagepath,['lastroiCyclePath.mat']);
    if exist(filename)==2
        pathToROIFile=load(filename,'-mat');
        pathToROIFile=getfield(pathToROIFile,char(fieldnames(pathToROIFile)));
        if isdir(pathToROIFile)
            cd(pathToROIFile);
        end
    end
end
[fname,pname]=uigetfile('*.rcf','Select File Name and Path...');
if isnumeric(fname)
    return
else
    state.roiCycle.roiCyclePath=pname;
    state.roiCycle.roiCycleName=fname;
    updateGUIByGlobal('state.roiCycle.roiCycleName');
    fn=fullfile(state.roiCycle.roiCyclePath,state.roiCycle.roiCycleName);
    currentROICycle=state.roiCycle.currentROICycle;
    currentROICycle=load(fn,'-mat');
    state.roiCycle.currentROICycle=getfield(currentROICycle,char(fieldnames(currentROICycle)));
    state.roiCycle.roiCyclePosition=1;
    updateGUIByGlobal('state.roiCycle.roiCyclePosition');
    roiCycleGUI('roiCyclePosition_Callback',gh.roiCycleGUI.roiCyclePosition);
    roiCyclePath=pname;
    scanimagepath=fileparts(which('scanimage'));
    save(fullfile(scanimagepath,['lastroiCyclePath.mat']),'roiCyclePath', '-mat');
end
