function updateFastConfigButtons(fastConfigNum,suppressAutoStart)
%% function updateFastConfigButtons(fastConfigNum)
%  Function to update display of fast configuration toggle buttons
%
%% SYNTAX
%   updateFastConfigButtons()
%   updateFastConfigButtons(fastConfigNum)
%   updateFastConfigButtons(fastConfigNum,suppressAutoStart)
%       fastConfigNum: <One of [1:6]> Identifies fast configuration that has just been set
%       suppressAutoStart: <OPTIONAL - LOGICAL - Default=false> Specifies, if true, to suprress autoStart behavior 
%
%   If no arguments are supplied, then current configuration is checked against all fast configs, and all are updated accordingly.
%
%% CREDITS
%   Created 10/14/10, by Vijay Iyer
%% *****************************************

global state

state.files.fastConfigSelection = '';

if nargin < 2 || isempty(suppressAutoStart)
    suppressAutoStart = false;
end

fastConfigOffs = 1:state.files.numFastConfigs;

if nargin == 0 || isempty(fastConfigNum) %No fastConfigNum supplied
    
    fullConfigName = fullfile(state.configPath, [state.configName '.cfg']);    
    
    for i=1:state.files.numFastConfigs        
        if strcmpi(state.files.(sprintf('fastConfig%d',i)), fullConfigName)
            fastConfigOffs = toggleOnSmart(i,fastConfigOffs,suppressAutoStart);
        end        
    end    
    
else    
    fastConfigOffs = toggleOnSmart(fastConfigNum, fastConfigOffs, suppressAutoStart);
end

toggleOff(fastConfigOffs);

return;

function fastConfigOffs = toggleOnSmart(fastConfigNum, fastConfigOffs, suppressAutoStart)
global gh state

if suppressAutoStart || ~state.files.fastConfigAutoStartArray(fastConfigNum)
    fastConfigOffs(fastConfigNum) = [];    
    
    hButton = gh.mainControls.(sprintf('tbFastConfig%d',fastConfigNum));
    set(hButton,'Value',1);
    
    state.files.fastConfigSelection = fastConfigNum;
end

return;


function toggleOff(fastConfigNums)
global gh

for i=1:length(fastConfigNums) 
    
    hButton = gh.mainControls.(sprintf('tbFastConfig%d',fastConfigNums(i)));    
    set(hButton,'Value',0);
    %set(hButton,'ForegroundColor',[.25 .25 .25],'Value',0);    
end

return;
        
        
        
        %
        %         if ismember(fullConfigName
        %
        %
        %
        %
        %             %Update fast config filename
        %             updateFastConfigDisplayRaw(i,'configName',state.files.(['fastConfig' num2str(i)]));
        %
        %             %Initialize AutoStart and AutoStartType arrays, if needed
        %             if isempty(state.files.fastConfigAutoStartArray)
        %                 state.files.fastConfigAutoStartArray = zeros(1,state.files.numFastConfigs);
        %             end
        %
        %             if isempty(state.files.fastConfigAutoStartTypeArray)
        %                 state.files.fastConfigAutoStartTypeArray = repmat({'GRAB'},1,state.files.numFastConfigs);
        %             end
        %
        %             %Update autoStart array
        %             updateFastConfigDisplayRaw(i,'autoStart',state.files.fastConfigAutoStartArray(i));
        %
        %             %Update autoStartyType array
        %             updateFastConfigDisplayRaw(i,'autoStartType',state.files.fastConfigAutoStartTypeArray{i});
        %         end
        %     else
        %         error(nargchk(3,3,nargin,'struct')); %If any arguments are supplied, all three must be supplied.
        %
        %         updateFastConfigDisplayRaw(fastConfigNum,dataVar,dataVal);
        %     end
        %
        %
        %
        %     function updateFastConfigDisplayRaw(fastConfigNum,dataVar,dataVal)
        %
        %         if ~isempty(dataVal)
        %             hTable = gh.fastConfigurationGUI.tblFastConfig;
        %             tableData = get(hTable,'Data');
        %
        %             switch dataVar
        %
        %                 case 'configName'
        %                     [~,fname] = fileparts(dataVal);
        %                     tableData{fastConfigNum, 2} = fname;
        %
        %                     %Update tooltip on MainControls
        %                     if ~isempty(fname)
        %                         autoStart =state.files.fastConfigAutoStartArray(fastConfigNum);
        %                         tipString = sprintf('%s (F%d)',fname,fastConfigNum);
        %                         if autoStart
        %                             tipString = sprintf('%s\nAUTO-START!\n(Right-click or Shift+F%d to suppress auto-start)',tipString,fastConfigNum);
        %                         end
        %                         set(gh.mainControls.(['tbFastConfig' num2str(fastConfigNum)]),'TooltipString',tipString);
        %                     end
        %
        %                 case 'autoStart'
        %                     tableData{fastConfigNum, 3} = logical(dataVal);
        %
        %                     %Update tooltip on MainControls
        %                     if ~isempty(state.files.(['fastConfig' num2str(fastConfigNum)]))
        %                         hButton = gh.mainControls.(['tbFastConfig' num2str(fastConfigNum)]);
        %
        %                         if dataVal && ~get(hButton,'Value') %autoStart ON, and toggled OFF
        %                             set(hButton,'BackgroundColor',[.4 .8 .4]);
        %                         else
        %                             set(gh.mainControls.(['tbFastConfig' num2str(fastConfigNum)]),'BackgroundColor',get(0,'defaultUIControlBackgroundColor'));
        %                         end
        %                     end
        %
        %                 case 'autoStartType'
        %                     assert(ismember(dataVal,{'FOCUS', 'GRAB', 'LOOP', 'GRAB STACK'}));
        %                     tableData{fastConfigNum, 4} = dataVal;
        %
        %                 otherwise
        %                     assert(false);
        %             end
        %
        %             set(hTable,'Data',tableData);
        %         end
        %
        %     end
        %
        % end