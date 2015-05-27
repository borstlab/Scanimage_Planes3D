function cacheStruct = parseInitializationFile(file,mode)
%PARSEINITIALIZATIONFILE Parses a INI/CFG file (read in cell array format)
%and does one of two things depending on 'mode': either loads data to state
%vars and updates GUI elements, or loads the data to the global
%configuration cache.

global state;

if nargin < 2 || isempty(mode)
	mode = 'initguis';
else
	mode = lower(mode);
end

if ~iscell(	file)
	error('parseInitializationFile: Input must be a cell array (like output from textread)')
end

state.internal.loading = 1;

currentStructure=[];
variableList={};

if strcmp(mode,'cachecfg')
	cacheStruct = struct();
end

try
    lineCounter=0;
    while lineCounter<length(file)				% step through each line of the file
        lineCounter=lineCounter+1;
        tokens=tokenize(file{lineCounter});		% turn each line into a cell array of tokens (words)
        if length(tokens)>0	        		% are there words on this line?
            if strcmp(tokens{1}, '%')       % if comment line, skip it
            elseif strcmp(tokens{1}, 'structure')		% are we starting a new structure?
                if length(currentStructure)>0
                    currentStructure=[currentStructure '.' tokens{2}];
                else
                    currentStructure=tokens{2};
                end
                [topName, structName, fieldName]=structNameParts(currentStructure);	

                if strcmp(mode,'initguis')
                    eval(['global ' topName ';']);		% get a global reference to the correct top level variable
                    if ~exist(topName,'var')
                        eval([topName '=[];']);
                    end
                    if length(fieldName)>0
                        if ~eval(['isfield(' structName ',''' fieldName ''');'])
                            eval([currentStructure '=[];']);
                        end
                    end
                elseif strcmp(mode,'cachecfg')
                    if ~isfield(cacheStruct,topName)
                        cacheStruct.(topName) = [];
                    end
                    if length(fieldName)>0
                        eval(['cacheStruct.' currentStructure '=[];']);
                    end
                end

            elseif strcmp(tokens{1}, 'endstructure') 		% are we ending a structure?
                periods=findstr(currentStructure, '.');		% then trim currentStructure depending on whether it
                if any(periods)								% has any subfields
                    currentStructure=currentStructure(1:periods(length(periods))-1);
                else
                    currentStructure=[];
                end

            else											% it must be a fieldname[=val] [param, value]* line
                fieldName=tokens{1};						% get fieldName
                startingValue=[];
                equ=findstr(fieldName, '=');				% is there a initialization value?
                if any(equ)
                    startingValue=fieldName(equ(1)+1:end);	% get initialization value
                    fieldName=fieldName(1:equ(1)-1);		% get fieldname without init value
                    val=str2num(startingValue);
                    if length(val)==0 | ~isnumeric(val)
                        if length(startingValue)>0
                            %%%VI060710A: Removed%%%%%
                            %                         %Note this should no longer be needed -- since all non-numeric-scalar values are now stored as strings
                            %                         if startingValue(1)~='''' | startingValue(end)~=''''
                            %                             startingValue=['''' startingValue ''''];
                            %                         end
                            %%%%%%%%%%%%%%%%%%%%%%%%%%
                        else
                            startingValue='0';
                        end
                    end
                end

                if length(currentStructure)==0						% must be a global variable and not the field of a global
                    fullVariableName=fieldName;

                    if strcmp(mode,'initguis')
                        eval(['global ' fullVariableName]);				% get access to the global
                        if ~exist(fullVariableName,'var')				% if global does not exist...
                            eval([fullVariableName '=' startingValue ';']);		% create it.
                        elseif length(startingValue)>0					% if global exists and there is an init value ...
                            eval([fullVariableName '=' startingValue ';']) 	% initialize global.
                        end
                    elseif strcmp(mode,'cachecfg')
                        eval(['cacheStruct.' fullVariableName ' = ' startingValue ';']);
                    end
                else												% we are dealing with the field of a global
                    fullVariableName=[currentStructure '.' fieldName];
                    if length(startingValue)>0	%there is an init value

                        %Determine if this is an Array/ArrayString pairing (NOTE: This is no longer meant to be utilized -- Vijay Iyer 2/10/09
                        patLoc = findstr(fullVariableName,'ArrayString');
                        if strcmp(mode,'initguis')
                            if  ~isempty(patLoc) && patLoc == length(fullVariableName) - 10 % && ~strcmpi(startingValue,'0')
                                evalVarName = fullVariableName(1:end-6);
                                arrayVar = true;
                            else
                                evalVarName = fullVariableName;
                                arrayVar = false;
                            end
                        elseif strcmp(mode,'cachecfg')
                            evalVarName = fullVariableName;
                            arrayVar = false;
                        end

                        if ~isempty(findstr(startingValue,'&'))
                            if strcmp(mode,'initguis')
                                eval([evalVarName '= ndArrayFromStr(' startingValue ');']);
                            elseif strcmp(mode,'cachecfg')
                                eval(['cacheStruct.' evalVarName ' = ndArrayFromStr(' startingValue ');']);
                            end
                        elseif ~isempty(findstr(startingValue,'$')) % look for a dollar sign, which we treat as an escape character for complex strings (i.e. quotes inside the string)
                            if strcmp(mode,'initguis')
                                eval([evalVarName '=' startingValue(1) startingValue(3:end) ';']);
                            elseif strcmp(mode,'cachecfg')
                                eval(['cacheStruct.' evalVarName ' = ' startingValue(1) startingValue(3:end) ';']);
                            end
                        elseif arrayVar %NOTE: This should no longer be utilized -- Vijay Iyer 2/10/09
                            if ~strcmpi(startingValue,'0')
                                eval([evalVarName '= str2num(' startingValue ');']);
                            else
                                eval([evalVarName '=[];']);
                            end
                        else
                            try
                                evalVal = eval(startingValue);
                            catch ME
                                ME.rethrow();
                            end

                            if ischar(evalVal) && ~isempty(str2num(evalVal)) %String represeents a non scalar/empty numeric value
                                if strcmp(mode,'initguis')
                                    eval([evalVarName '= str2num(' startingValue ');']);
                                elseif strcmp(mode,'cachecfg')
                                    eval(['cacheStruct.' evalVarName ' = str2num(' startingValue ');']);
                                end
                            elseif ischar(evalVal) && ~isempty(evalVal) && evalVal(1) == '{' %string cell array
                                evalVal = evalVal(2:(end-1)); %Remove cell brackets

                                if isempty(strtrim(evalVal))
                                    cellString = '{}';
                                else
                                    semicolons = findstr(';',evalVal);
                                    rowVector = ~isempty(semicolons);
                                    if rowVector
                                        evalVal(semicolons) = [];
                                    end

                                    cellVals = textscan(evalVal,'%s','Delimiter','|');
                                    cellVals = cellVals{1};
                                    cellVals = cellfun(@(x)strrep(x,'''',''''''),cellVals,'UniformOutput',false);

                                    %Construct string encoding of cell array to evaluate
                                    if rowVector
                                        cellString = ['{' sprintf('''%s''; ',cellVals{:}) '}'];
                                    else
                                        cellString = ['{' sprintf('''%s'' ',cellVals{:}) '}'];
                                    end
                                end

                                if strcmp(mode,'initguis')
                                    %Evaluate cell array & load to state var
                                    eval([evalVarName '= ' cellString ';']);
                                elseif strcmp(mode,'cachecfg')
                                    eval(['cacheStruct.' evalVarName ' = ' cellString ';']);
                                end
                            else %String represents a scalar/empty numeric value, or a non-numeric value (e.g. a string, or a string cell array)
                                if strcmp(mode,'initguis')
                                    eval([evalVarName '= ' startingValue ';']);
                                elseif strcmp(mode,'cachecfg')
                                    eval(['cacheStruct.' evalVarName ' = ' startingValue ';']);
                                end
                            end
                        end
                    elseif ~eval(['isfield(' currentStructure ',''' fieldName ''');']) 	% if not, if field does not exist ...
                        if strcmp(mode,'initguis')
                            eval([fullVariableName '=[];'])					% initialize it
                        elseif strcmp(mode,'cachecfg')
                            eval(['cacheStruct.' fullVariableName '=[]']);
                        end
                    end

                end

                if strcmp(mode,'initguis')
                    variableList=[variableList, {fullVariableName}];
                    validGUI=0;
                    if length(tokens)>1
                        tokenCounter=2;
                        while tokenCounter<length(tokens)							% loop through [param, value]* 
                            param=tokens{tokenCounter};
                            if strcmp(param, '...')					% continuation marker
                                lineCounter=lineCounter+1;				% advance to next line in file
                                tokens=tokenize(file{lineCounter});		% turn each line into a cell array of tokens (words)
                                tokenCounter=1;
                                param=tokens{tokenCounter};
                            end
                            value=tokens{tokenCounter+1};
                            if strcmp(param, '%')                       % found comment field. Skip line
                                break;
                            else                                        % not a comment line
                                if strcmp(param, 'Gui')						% special case for associating a GUI to a Global
                                    if ~existGlobal(value)
                                        disp(['initGUIs: GUI ' value ' for ' fullVariableName ' does not exist.  Skipping userdata...']);
                                    else
                                        validGUI=1;
                                        addGUIOfGlobal(fullVariableName, value);
                                        setUserDataByGUIName({value}, 'Global', fullVariableName);	
                                    end
                                elseif strcmp(param, 'Config')				% special case for labelling a global as part of a configuration
                                    setGlobalConfigStatus(fullVariableName, value);

                                    % if the 'CFG' bit is set, add this to our list of CFG vars
                                    if ~isnan(str2double(value)) && bitand(str2double(value),1) % MAGICNUMBER: by convention, '1' indicates a CFG var
                                        state.configVars = [state.configVars fullVariableName];
                                    end
                                else										% put everything else in UserData
                                    if validGUI==1
                                        vNum=str2num(value);
                                        if isnumeric(vNum) & length(vNum)==1	% can it be a number?
                                            value=vNum;							% yes, then make it a number
                                        end
                                        setUserDataByGlobal(fullVariableName, param, value);	% put in UserData
                                    end
                                end
                            end
                            tokenCounter=tokenCounter+2;
                        end
                    end                
                    updateGUIByGlobal(fullVariableName);				% update all the GUIs that deal with the global variable
                end
            end
        end
    end



    % Now execute all the callbacks that were collected during the processing of the
    % *.ini.  This ensures that everything is correct after the fields in the GUIs
    % have been changed by the initialization.
    if strcmp(mode,'initguis')
        doneCallBacks=';;;';
        for i=1:length(variableList)
            entry=variableList{i};
            GUIList=getGuiOfGlobal(entry);
            if length(GUIList)>0
                for count=1:length(GUIList)
                    GUI=GUIList{count};
                    if length(GUI)>0
                        [topGUI, sGui, fGui]=structNameParts(GUI);
                        eval(['global ' topGUI]);
                        funcName='';
                        eval(['funcName=getUserDataField(' GUI ', ''Callback'');']);
                        if length(funcName)>0
                            if length(findstr(doneCallBacks, [';' funcName ';']))==0
                                doneCallBacks=[doneCallBacks funcName ';'];
                                %							disp(['DoGUICallback(' GUI ');']);		% for debugging
                                eval(['doGUICallback(' GUI ');']);
                            end
                        end
                    end
                end
            end
        end
    end
catch ME
   ME.rethrow();
   state.internal.loading = 0;
end

state.internal.loading = 0;

