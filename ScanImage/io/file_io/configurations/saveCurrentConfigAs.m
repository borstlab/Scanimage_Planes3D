function varargout =saveCurrentConfigAs(curFileDefault)
%% function varargout =saveCurrentConfigAs(curFileDefault)
%   Save current configuration to user specified CFG file
%% SYNTAX
%   saveCurrentConfigAs()
%
%% CHANGES
%   VI020209A: No longer use 'cd' to change the default path option -- Vijay Iyer 2/2/09
%   VI110210A: Use windows home-drive, rather than Matlab current directory, as the default directory -- Vijay Iyer 11/2/10
%   VI111110A Vijay Iyer 11/11/10 -- Defer to most.idioms.startPath() logic to implement VI110210A
%% *******************************************
global state

%%%VI020209A
% if ~isempty(state.configPath)
%     cd(state.configPath)
% end
%%%%%%%%%%%%

%%%VI020209A%%%%%%%%%%%%%
if ~isempty(state.configPath)
    startPath = state.configPath;
else
    startPath = most.idioms.startPath(); %VI111110A %VI110210A    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%
   
[fname, pname]=uiputfile('*.cfg', 'Choose Configuration name...',startPath); %VI020209A

if ~isnumeric(fname)
    setStatusString('Saving config...');
	
    [~,f,e] = fileparts(fname);
    if strcmpi(e,'.cfg')
        state.configName = f;
    else
        state.configName = [f e];
    end 
    state.configPath=pname;

    %%%VI020209A%%%%%%%%%
    if ~state.cycle.cycleOn
        state.internal.configName = state.configName;
        state.internal.configPath = state.configPath;
    end
    %%%%%%%%%%%%%%%%%%%%%%

    updateGUIByGlobal('state.configName');
    saveCurrentConfig;
    setStatusString('');
    state.internal.configurationNeedsSaving=0;
else
    setStatusString('Cannot open file');
end
