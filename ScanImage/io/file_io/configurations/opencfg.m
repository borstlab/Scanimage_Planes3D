function opencfg(configNameFull,suppressUpdateFastConfigButtons)
%% function opencfg(configNameFull,suppressUpdateFastConfigButtons)
%
%% NOTES
%   Replaces previous loadStandardModeConfig() and opencfg() functions (see MOLD files) -- Vijay Iyer 1/5/10
%
%% CREDITS
% Created 1/5/11, by Vijay Iyer
% Based on earlier loadStandardModeConfig() function
%% ************************************************************

global state gh

%turnOffMenus; %VI092810B
turnOffExecuteButtons;

%Allow config file to be specified as an input argument
if nargin && ~isempty(configNameFull)    
    assert(exist(configNameFull,'file') > 0,'The specified file ''%s'' was specified, but not found to exist',configNameFull);
    [state.configPath, state.configName] = fileparts(configNameFull);
end

%Allow update of fast config highlighting after 
if nargin < 2 || isempty(suppressUpdateFastConfigButtons)
    suppressUpdateFastConfigButtons = false;
end

configSelected=1;

if isnumeric(state.configName) || isempty(state.configName)
    configSelected=0;
else
    [flag, fname, pname]=initGUIs(fullfile(state.configPath,[state.configName '.cfg']));
    if flag==0
        configSelected=0;
    end
end
    
if configSelected
    setStatusString('Config loaded');
    state.configName=fname;
    state.configPath=pname;
else
    setStatusString('Using default config'); %VI110708A
    disp('opencfg: No configuration selected.  Using ''default'' values (i.e. those from current INI file).'); %VI110708A
    state.configName='Default';
    state.configPath='';
end
updateGUIByGlobal('state.configName');

% if strcmp(state.userFcns.saveTarget,'CFG')
	updateUserFunctionState('loadStateVars','CFG'); %VI100110A
	updateUserFunctionState('loadStateVars','OVERRIDECFG');
% end

applyChannelSettings; %This calls applyConfigurationSettings() 

if state.init.eom.pockelsOn   
    state.init.eom.changed(:) = 1;
end

setStatusString('');

set(gh.userFcnGUI.UserFcnSelected,'Value',1,'String', state.userFcnGUI.UserFcnSelected); %VI120109A

verifyEomConfig;
resetConfigurationNeedsSaving(); %Reset flag var

if ~suppressUpdateFastConfigButtons
    updateFastConfigButtons();
end

%%%VI021009A%%%%%%%%%
%turnOnMenus; %VI092810B
turnOnExecuteButtons;
%%%%%%%%%%%%%%%%%%%%%
