function setSavePath
global state

startPath = state.files.savePath;

if isempty(startPath)
    startPath = state.files.rootSavePath;
    
    %%%VI110210A
    if isempty(startPath)
        startPath = most.idioms.startPath(); %VI111110A
    end
end

p = uigetdir(startPath, 'Select Save Path');
if p
    state.files.savePath=p;    

    updateFullFileName(0);
    %cd(p); %VI092410A: REMOVED - scim_savePath() utility created as alternative.
    disp(['*** SAVE PATH = ' state.files.savePath ' ***']); 
end






