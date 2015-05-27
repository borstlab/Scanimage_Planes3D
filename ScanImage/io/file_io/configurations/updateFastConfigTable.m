function updateFastConfigTable(fastConfigNum,dataVar)
%% function updateFastConfigTable(fastConfigNum,dataVar)
%  Function to update display of fast configuration data, either of specified config variable, or all combinations
%
%% SYNTAX
%   updateFastConfigTable()
%   updateFastConfigTable(fastConfigNum,dataVar)
%       fastConfigNum: <One of [1:6]> Identifies fast configuration to be set
%       dataVar: <One of {'configName' 'autoStart' 'autoStartType'}> Identifies variable to be set pertaining to identified fast configuration
%
%   If no arguments are supplied, then all fast configuration UI controls are updated according to the current state variable values
%
%% NOTES
%   Function exists to contain logic shared between UI callbacks and openusr()
%
%   Function also serves to initialize the fast config array vars
%
%   Arguably, the updateFastConfigButtonDisplay() logic should be in updateFastConfigButtons(), which could be called from here in some way -- Vijay Iyer 10/14/10
%
%% CREDITS
%   Created 9/27/10, by Vijay Iyer
%% *****************************************

global gh state

    if nargin == 0 %No arguments supplied
        for i=1:state.files.numFastConfigs     
            
            %Initialize AutoStart and AutoStartType arrays, if needed
            if isempty(state.files.fastConfigAutoStartArray)
                state.files.fastConfigAutoStartArray = zeros(1,state.files.numFastConfigs);
            end
            
            if isempty(state.files.fastConfigAutoStartTypeArray)
                state.files.fastConfigAutoStartTypeArray = repmat({'GRAB'},1,state.files.numFastConfigs);
            end           
            
            %Update fast config filename
            updateFastConfigDisplayRaw(i,'configName');
                         
            %Update autoStart array
            updateFastConfigDisplayRaw(i,'autoStart');                        
            
            %Update autoStartyType array 
            updateFastConfigDisplayRaw(i,'autoStartType');
        end        
    else
        error(nargchk(2,2,nargin,'struct')); %If any arguments are supplied, all must be supplied.        
        
        updateFastConfigDisplayRaw(fastConfigNum,dataVar);                                        
    end
        


    function updateFastConfigDisplayRaw(fastConfigNum,dataVar)
        
        hTable = gh.fastConfigurationGUI.tblFastConfig;
        tableData = get(hTable,'Data');
        
        switch dataVar
            
            case 'configName'
                fname =  extractFastConfigFilename(fastConfigNum);
                tableData{fastConfigNum, 3} = fname;
                
                %Update toggle button display
                updateFastConfigButtonDisplay(fastConfigNum,fname);
                
            case 'autoStart'              
                tableData{fastConfigNum, 4} = logical(state.files.fastConfigAutoStartArray(fastConfigNum));
                
                %Update toggle button display
                updateFastConfigButtonDisplay(fastConfigNum);
                    
            case 'autoStartType'
                dataVal = state.files.fastConfigAutoStartTypeArray{fastConfigNum};
                assert(ismember(dataVal,{'FOCUS', 'GRAB', 'LOOP', 'GRAB STACK'}));
                tableData{fastConfigNum, 5} = dataVal;
                
            otherwise
                assert(false);
        end
        
        set(hTable,'Data',tableData);
                
    end

    function updateFastConfigButtonDisplay(fastConfigNum,fname)    
        
        if nargin < 2
            fname =  extractFastConfigFilename(fastConfigNum);
        end

        %Update button color
        hButton = gh.mainControls.(sprintf('tbFastConfig%d',fastConfigNum));
        defaultColor = get(0,'defaultUIControlBackgroundColor');
        if isempty(fname)
            set(hButton,'BackgroundColor',defaultColor); 
        else        
            if logical(state.files.fastConfigAutoStartArray(fastConfigNum))
                set(hButton,'BackgroundColor',[.4 .8 .4]); %Auto-Start ON
            else
                set(hButton,'BackgroundColor',defaultColor); %Auto-Start OFF
            end
        end
        
        %Update tooltip string        
        autoStart =state.files.fastConfigAutoStartArray(fastConfigNum);
        
        if state.init.fastConfigHotKeysNeedCtl
            fKeyString = 'Ctrl+F';
        else
            fKeyString = 'F';
        end
        
        if isempty(fname) %VI102010A
            tipString = 'Unassigned';
        else
            tipString = sprintf('%s (%s%d)',fname,fKeyString,fastConfigNum);
            if autoStart
                tipString = sprintf('%s\nAUTO-START!\n(Right-click or Shift+%s%d to suppress auto-start)',tipString,fKeyString,fastConfigNum);
            end
        end
        set(gh.mainControls.(['tbFastConfig' num2str(fastConfigNum)]),'TooltipString',tipString);        
    end
    
    function fname = extractFastConfigFilename(fastConfigNum)
        fastConfigVal = state.files.(sprintf('fastConfig%d',fastConfigNum));
        if isempty(fastConfigVal)
            fname = '';
        else
            [~,fname] = fileparts(fastConfigVal);
        end
    end


end

