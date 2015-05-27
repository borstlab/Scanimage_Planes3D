function out=loadConfigurationFile(cfgFile)
% Allows user to select a configuration from disk and loads it
%
%% SYNTAX
%   out=loadConfigurationFile(cfgFile)
%       cfgFile: A filename, or a full filename path, to configuration file to load. If a simple filename, then the last configuration directory will be presumed. If empty/omitted, user will be prompted to select file.
%
%% CHANGES
%   Tim O'Connor 2/19/04 TO21904a - Make messages more understandable.
%   TO3204b - Pick up path from standard.ini, for convenience.
%   VI012709A: Use resetConfigurationNeedsSaving() -- Vijay Iyer 1/27/09
%   VI021009A: Refactor some common code to loadStandardModeConfig -- Vijay Iyer 2/10/09
%   VI021009B: Update fast config toggle buttons following manual configuration load -- Vijay Iyer 2/10/09
%   VI021009C: Don't use 'cd' to direct initial configruration path choice -- Vijay Iyer 2/10/09
%   VI021009D: Set status string to final state of operation, rather than restoring original state -- Vijay Iyer 2/10/09
%   VI050510A: Allow filename, either partial or full, to be specified as argument, bypassing graphical selection -- Vijay Iyer 5/5/10
%   VI010511A: Set state.configName/Path directly, instead of standardMode indirection -- Vijay Iyer 1/5/11
%   VI082511A: Remove call to applyConfigurationSettings(), which was redundant with opencfg() -- Vijay Iyer 8/25/11
%
%% CREDITS
%   Author: Bernardo Sabatini
%% ***********************************'

out=0;

global state

status=state.internal.statusString;
setStatusString('Loading Configuration...');
if state.internal.configurationNeedsSaving==1

    if ~isempty(state.configName)
        button = questdlg(['Do you want to save changes to ''' state.configName '''?'],'Save changes?','Yes','No','Cancel','Yes');
    else
        %TO21904 - Don't just print a set of empty quotes.
        button = questdlg(['Do you want to save changes to the current configuration?'],'Save changes?','Yes','No','Cancel','Yes');
    end

    if strcmp(button, 'Cancel')
        disp(['*** LOAD CYCLE CANCELLED ***']);
        setStatusString('Cancelled');
        return
    elseif strcmp(button, 'Yes')
        if ~isempty(state.configName)
            disp(['*** SAVING CURRENT CONFIGURATION = ' state.configPath '\' state.configName ' ***']);
            flag=saveCurrentConfig;
            if ~flag
                disp(['loadConfigurationFile: Error returned by saveCurrentCycle.  Cycle may not have been saved.']);
                setStatusString('Error saving file');
                return
            end
        else
            %TO21904a - Need to choose a name.
            saveCurrentConfigAs;
        end
        %state.internal.configurationNeedsSaving=0; %VI012709A
        resetConfigurationNeedsSaving(); %VI012709A
    end
end

%%%VI021009C %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if ~isempty(state.configPath) && isdir(state.configPath)
    startPath = state.configPath;
else
    startPath = cd;
end
if ~strcmpi(startPath(end),filesep) %Ensure startPath ends with a slash
    startPath = [startPath filesep];
end

if nargin < 1 || isempty(cfgFile) %VI050510A
    [fname, pname] = uigetfile([startPath '*.cfg'], 'Choose configuration to load');
    
    if isnumeric(fname)
        return;
    end
    
    [~,~,ext] = fileparts(fname);
    if isempty(ext) || ~strcmpi(ext,'.cfg')
        fprintf(2,'WARNING: Invalid file extension provided. Cannot open CFG file.\n');
        return
    end
    
%%%%%VI050510A%%%%%%%%%%%%%%%%%   
else
    assert(ischar(cfgFile)&&isvector(cfgFile),'Argument to loadConfigurationFile() must be string-valued, specifying configuration (CFG) filename/path to load');
    [pname,f,e] = fileparts(cfgFile);
    if isempty(pname)
        pname = startPath;
    end
    fname = [f e];
    if ~strcmpi(e,'.cfg') || ~exist(fullfile(pname,fname),'file')
        error('Invalid or non-existant configuration (CFG) file specified.');
    end
end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%VI021009C: Redone%%%%%%%%%%%%%%%%
% if ~isempty(state.configPath) & isdir(state.configPath)
%     try
%         cd(state.configPath)
%     end
% end
% %TO3204b - Use a prespecified path from standard.ini, if possible.
% if ~isempty(state.standardMode.configPath)
%     %Make sure it's terminated with a '\' character.
%     if state.standardMode.configPath(end) ~= '\'
%         state.standardMode.configPath = [state.standardMode.configPath '\'];
%     end
%     [fname, pname] = uigetfile([state.standardMode.configPath '*.cfg'], 'Choose configuration to load');
% else
%     [fname, pname] = uigetfile('*.cfg', 'Choose configuration to load');
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if ~isnumeric(fname)
    
    %%%VI010611A: Removed%%%%%%    
    %     periods=findstr(fname, '.');
    %     if any(periods)
    %         fname=fname(1:periods(1)-1);
    %     else
    %         disp('loadConfigurationFile: Error: found file name without extension');
    %         setStatusString('Can''t open file');
    %         return
    %     end
    %
    %     state.configName=fname; %VI010511A
    %     state.configPath=pname; %VI010511A
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%VI021009A: Refactored%%%%%%
    %     turnOffMenus;
    %     turnOffExecuteButtons;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%VI021009A: Removed%%%%%%%%%%%%
    %     try
    %         if state.init.pockelsOn
    %             for j = 1 : state.init.eom.numberOfBeams
    %                 h=findobj('Type','Rectangle','Tag', sprintf('PowerBox%s', num2str(j)));
    %                 if ~isempty(h)
    %                     delete(h);
    %                 end
    %             end
    %         end
    %     catch
    %         warning(lasterr)
    %     end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    opencfg(fullfile(pname,fname)); %VI010611A
        
    
    %%%VI021009A: Refactored%%%%%%
    %     turnOnMenus;
    %     turnOnExecuteButtons;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    setStatusString('Config Loaded'); %VI021009D
else
    setStatusString(''); %VI021009D
end

%    closeConfigurationGUI; %VI092508A
%Note, this call is probably not necesary--the call to loadStandardmodeConfig calls applyConfigurationSettings(). -- Vijay Iyer 9/27/08
%applyConfigurationSettings; %VI082511A %VI092508A
