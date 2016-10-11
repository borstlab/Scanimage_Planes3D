function loadFastConfig(handleOrConfigNum,suppressAutoStart)
%% function loadFastConfig(handleOrConfigNum,suppressAutoStart)
%  Function for loading a ScanImage fast configuration 
%
%% SYNTAX
%   loadFastConfig(hObject)
%   loadFastConfig(configNum,forceValue)
%       handleOrConfigNum: Either 1) handle to fastConfig toggle button or 2) fast config number
%       suppressAutoStart: <OPTIONAL - LOGICAL - Default=false> Specifies, if true, to suprress 

%       forceValue: Logical value which forces the toggle button to the specified state. Must be supplied if configNum is supplied.
%% NOTES
%   Function is used in two cases: 1) directly as a callback when the button is pressed, and 2) programatically, where the configuration is identified by number and forced to a value
%
%   This function needs to be rearchitected somewhat -- so that the two modes of operation (button pressed vs. called as function) are made more distinct OR are separate functions
%   
%% CREDITS
%   VI102010A: Catch case where fast config filename is no longer valid. -- Vijay Iyer 10/20/10
%   VI102810A: Only load configuration if it's needed -- Vijay Iyer 10/28/10
%   VI010511A: loadStandardModeConfig() is now opencfg() -- Vijay Iyer 1/5/11
%
%% CREDITS
%   Created 10/14/10, by Vijay Iyer   
%% *****************************

global state gh

guiCallback = ~ismember(handleOrConfigNum, 1:state.files.numFastConfigs); % Function is result of direct toggle (callback)

if guiCallback
    configNumStr = deblank(get(handleOrConfigNum,'Tag'));
    configNum = str2double(configNumStr(end));
    hButton = handleOrConfigNum;
    
    %Disallow direct toggle off
    if ~get(hButton,'Value')
        assert(configNum == state.files.fastConfigSelection);
        updateFastConfigButtons(configNum,true);
        return;
    end       
    
    %Auto-start always honored on direct push of button
    suppressAutoStart = false;
else   
    configNum = handleOrConfigNum;
    hButton = gh.mainControls.(['tbFastConfig' num2str(configNum)]);
  
    if nargin < 2 || isempty(suppressAutoStart)
        suppressAutoStart = false;
	end
end

%% Determine if config change is requested -- return if not
lastFastConfig = state.files.fastConfigSelection;
if ~isempty(lastFastConfig) && lastFastConfig == configNum; 
    return;
end

%If auto-start is to apply, make toggle button seem/feel like a pushbutton
autoStart = ~suppressAutoStart && state.files.fastConfigAutoStartArray(configNum); %Will auto-start the configuration
if autoStart
    set(hButton,'Value',0); 
    drawnow update;
end

%Cache the previous configuration
currentConfig = fullfile(state.configPath,[state.configName '.cfg']);
    
%Determine if fast configuration selected exists
fieldname = sprintf('fastConfig%d',configNum);
if ~isfield(state.files,fieldname) || isempty(state.files.(fieldname))
    setStatusString(['Fast Config ' num2str(configNum) ' Not Set']);
    updateFastConfigButtons(); %Make sure toggle buttons reflect current state
    return
else
    needLoadConfig = ~strcmpi(currentConfig,state.files.(fieldname)); %VI102810A
    
    if needLoadConfig %VI102810a
        setStatusString('Switching config...');
    end
end


%Load configuration, if needed
if needLoadConfig %VI102810A
    try
        %Update toggle button  /before/ loading config, to give feeling of immediate response
        updateFastConfigButtons(configNum,suppressAutoStart);
        drawnow expose;
        
        fieldname = sprintf('fastConfig%d',configNum);
        
        try
            loadCachedConfiguration(state.files.(fieldname));
			
            %%%VI102010A%%%%%%%
        catch ME
            if ~(exist(state.files.(fieldname),'file') == 2)
                setStatusString(['Fast Config ' num2str(configNum) ' Invalid']);
                state.files.(fieldname) = '';
                updateFastConfigTable(configNum,'configName');
                updateFastConfigButtons();
            else
                ME.rethrow();
            end
        end
        %%%%%%%%%%%%%%%%%%%%
        
    catch ME
        updateFastConfigButtons(); %Make sure toggle buttons reflect current state
        ME.rethrow();
    end
end

figure(gh.mainControls.figure1);

%Handle auto-start        
state.files.fastConfigAutoStartCachedConfig = '';
if autoStart
        
    try        
        state.files.fastConfigAutoStartCachedConfig = currentConfig;        
        
        %Start specified type of acquisition             
        switch state.files.fastConfigAutoStartTypeArray{configNum}
            case 'FOCUS'
                executeFocusCallback(gh.mainControls.focusButton);
            case 'GRAB'
                executeGrabOneCallback(gh.mainControls.grabOneButton);
            case 'LOOP'
                executeStartLoopCallback(gh.mainControls.startLoopButton);
            case 'GRAB STACK'
                executeGrabOneStackCallback();
            otherwise
                assert(false);
        end
        
    catch ME;
        state.files.fastConfigAutoStartCachedConfig = '';
        ME.rethrow();
    end
end

return;

