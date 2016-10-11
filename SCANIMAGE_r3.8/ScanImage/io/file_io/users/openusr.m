function out=openusr(fileName, startup)
%% function out=openusr(fileName)
%   Function parses .usr file to update selected Scanimage state variables with values saved by a user(overriding the .ini file settings)
%% SYNTAX
%   out = openusr(fileName)
%   out = openuser(fileName, startup)
%       fileName: Name of USR file to parse/process
%       startup: Optional boolean flag indicating TRUE when called during startup (i.e. open USR file for first time), FALSE otherwise. FALSE is assumed.
%       out: Flag equals 0 when there's no error, 1 if an error occurs. 
%% NOTES
%   The 'out' flag doesn't work, but isn't really needed -- Vijay Iyer 10/31/08
%       
%% MODIFICATIONS
%   VI082608A: Cycle through all 6 fastConfig, now that there are 6 instead of 3. Thanks to Jesper Sjostrom for this bug find. -- Vijay Iyer 8/26/08
%   VI103108A: Calibrate Pockels Cell here now, having determined what beams this user actually employs
%   VI021009A: Defer to updateFastConfig() to handle fastConfig settings loaded with the USR file -- Vijay Iyer 2/10/09
%   VI011310A: No longer need to call updateXXXCheckMark(). The AutoOverwrite and KeepAllSlicesInMemory functions are deprecated. The AutoSave callback should be invoked anyway, as it is an INI-file callback. -- Vijay Iyer 1/13/10
%   VI092710A: Use new updateFastConfigTable(), instead of updateFastConfig(); this pertains to the new UserConfiguration dialog -- Vijay Iyer 9/27/10
%   VI010511A: Replace deprecated applyModeCycleAndConfigSettings() with simple opencfg() -- Vijay Iyer 1/5/11
%   VI012511A: Protect against (corrupted) USR files with XXXVisible set as empty -- Vijay Iyer 1/25/11
%   VI022311A: BUGFIX -- Double crosshair was appearing on startup; need to make sure that updateImageBox() call occurs /after/ resetImageProperties -- Vijay Iyer 2/23/11
%
%% *******************************************************

out=1;
[fid, message]=fopen(fileName);
if fid<0
    beep;
    disp(['openusr: Error opening ' fileName ': ' message]);
    out=1;
    return
end
[fileName,permission, machineormat] = fopen(fid);
fclose(fid);

disp(['*** CURRENT USER SETTINGS FILE = ' fileName ' ***']);

initGUIs(fileName);

[path,name,~] = fileparts(fileName);

global state
state.userSettingsName=name;
state.userSettingsPath=path;
saveUserSettingsPath;

% load all USR-bound SI3 properties
state.hSI.loadUSRProperties(fileName);

%%%VI010611A: Removed%%%
% state.configName='';
% state.configPath='';
%%%%%%%%%%%%%%%%%%%%%%%%

%VI103008A: Calibrate beams now, only doing those actually used by this user
if nargin < 2 
    startup = false;
end
calibrateBeams(startup);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%applyModeCycleAndConfigSettings; %VI010511A
opencfg(); %VI010511A

%%%VI011310A:Removed%%%%%%%%%%%
% %Update uimenu checkmark-able options based on USR file
% updateAutoSaveCheckMark;		% BSMOD
% updateKeepAllSlicesCheckMark; % BSMOD
% updateAutoOverwriteCheckMark;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%DEQ2010119-removed because this was overriding channel-specific maps
%setColorMapFromMenu(state.internal.colormapSelected);

global gh	% BSMOD added 1/30/1 with lines below


%arrayfun(@(num)updateFastConfig(num),1:state.files.numFastConfigs); %VI092710A %VI021009A
updateFastConfigTable(); %VI092710A

% clear any existing cache entries and cache any fast-config files
removeCachedConfiguration();
for i = 1:state.files.numFastConfigs
	fastConfigFileName = eval(['state.files.fastConfig' num2str(i)]);
	if ~isempty(fastConfigFileName)
		cacheConfiguration(fastConfigFileName);
	end
end

updateGUIByGlobal('state.userSettingsName');

% if strcmp(state.userFcns.saveTarget,'USR')
	updateUserFunctionState('loadStateVars','USR'); %VI100110A
	updateUserFunctionState('loadStateVars','OVERRIDEUSR');
% end
updateUserFunctionState('loadStateVars','USRONLY');

%%%VI021009A: Removed %%%%%%
% for number=1:6 %VI082608A
%     fname=getfield(state.files,['fastConfig' num2str(number)]);
%     if isempty(fname)
%         fname=num2str(number);
%     else
%         [path,fname]=fileparts(fname);
%     end
%     h=getfield(gh.mainControls,['fastConfig' num2str(number)]);
%     label=get(h,'Label');
%     ind=findstr(label,' ');
%     label(1:ind(end))=[];
%     label=[fname '   ' label];
%     set(h,'Label',label);
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

wins=fieldnames(gh);

for winCount=1:length(wins)
    winName=wins{winCount};
    if isfield(state.internal, [winName 'Bottom']) & isfield(state.internal, [winName 'Left'])
        pos=get(getfield(getfield(gh, winName), 'figure1'), 'Position');
        if ~isempty(getfield(state.internal, [winName 'Left'])) %TPMOD
            pos(1)=getfield(state.internal, [winName 'Left']);
            pos(2)=getfield(state.internal, [winName 'Bottom']);
            set(getfield(getfield(gh, winName), 'figure1'), 'Position', pos);
        end
        if isfield(state.internal, [winName 'Visible']) && ~isempty(state.internal.(sprintf('%sVisible',winName))) %VI012511A
             set(getfield(getfield(gh, winName), 'figure1'), 'Visible', getfield(state.internal, [winName 'Visible']));
         end
    end
end

resetImageProperties(true);
%TPMOD....
% if isfield(state.internal, 'roifigurePositionX') & isfield(state.internal, 'roifigureVisible') 
%     roipos=[state.internal.roifigurePositionX state.internal.roifigurePositionY state.internal.roifigureWidth state.internal.roifigureHeight];
%     set(state.internal.roifigure,'Position',roipos,'Visible',state.internal.roifigureVisible);
% end
%userFcnGUI('UserFcnPath_Callback',gh.userFcnGUI.UserFcnPath); %VI120109A: Removed
powerControl('usePowerArray_Callback',gh.powerControl.usePowerArray);
powerTransitions('useBinaryTransitions_Callback',gh.powerTransitions.useBinaryTransitions);
updateShutterDelay;
%updateImageBox(); %VI112911A: Removed %VI022311A
