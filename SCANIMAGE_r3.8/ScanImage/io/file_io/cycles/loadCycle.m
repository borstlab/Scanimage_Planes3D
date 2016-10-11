function [ output_args ] = loadCYC( input_args )
%LOADCYC Loads a saved cycle file (*.cyc)
    global state gh
    
	try
		%Prompt user to select file
		startPath = state.hSI.getLastPath('cycleLastPath');
		[fname, pname]=uigetfile({'*.cyc'},'Choose CYC File...',startPath);
		if isnumeric(fname)
			return
		else
			[~,filenameNoExtension,~] = fileparts(fname);

			if ~strcmp(pname,startPath)
				state.hSI.setLastPath('cycleLastPath',pname);
			end
		end
	catch ME
		ME.throwAsCaller();
	end
	
	% handle a 'cancel' click
	if isnumeric(fname) && fname == 0 && isnumeric(pname) && pname == 0
		return;
	end
	
    try 
        [fID, message] = fopen(fullfile(pname,fname));
    catch ME
        error('Unable to open file.');
    end
    
    if fID < 0
        error('Unable to open file: %s.',message);
	end

	% initialize some needed regular expressions:
    globalExp = '^(\D\w*)\t(.+)\t$';
    cycleExp = '^(\d+)\t((.+)\t(.+)\t)+';
    keyValExp = '([^\t]+)\t([^\t]+)\t';
    
	% ensure the uitable is in the expected state
	set(gh.cycleGUI.tblCycle,'ColumnName',state.cycle.cycleTableColumnsPretty');
	
    % initialize the table Data cell array (the data that gets written to the uitable's 'Data' field)
	tableDataInit = repmat(state.cycle.cycleTableColumnDefaults,2,1);
    state.cycle.cycleConfigPaths = repmat({''},2,1);
    
	tableData = tableDataInit;
    
	state.cycle.cycleTableColumnsUserAdded = {};
	
    currentLine = fgetl(fID);
    while ischar(currentLine)
        tokens = regexp(currentLine,globalExp,'tokens','once');
        if ~isempty(tokens) % we have a global param...
            if regexp(tokens{2},'^\d+$') % scalar numeric
                state.cycle.(tokens{1}) = str2double(tokens{2});
            else % string
                state.cycle.(tokens{1}) = tokens{2};
            end
            
            updateGUIByGlobal(sprintf('state.cycle.%s',tokens{1}));
        else % we have a cycleable param...
            % DEQ20110121: there has to be a more elegant way of doing this than three chained regexps...
            tokens = regexp(currentLine,cycleExp,'tokens','once');
            if ~isempty(tokens)
                index = str2double(tokens{1});
				
				tableData(index,:) = state.cycle.cycleTableColumnDefaults(:);
                state.cycle.cycleConfigPaths{index} = '';
				
                keyValLine = tokens{2};
                matches = regexp(keyValLine,keyValExp,'match');
                for match = matches
                    keyVal = regexp(match{:},'(.+)\t(.+)\t','tokens');
					key = keyVal{1}{1};
					val = keyVal{1}{2};	
                    
					if ~strcmp(key,'configPath')
						if strcmp(key(1:5),'state')
							% this must be a user-added parameter, so add it to our internal list of columns:
							state.cycle.cycleTableColumnsUserAdded = [state.cycle.cycleTableColumnsUserAdded {key}];

							% add it to the cycle table struct
							keyNoDots = strrep(key,'.','DOT');
							state.cycle.cycleTableStruct(index).(keyNoDots) = val;

							% update the uitable's column names (with just the variable name, no 'state.XXX.')
							varName = regexp(key,'^state\.\w+\.(\w+)$','tokens','once');
							if ~isempty(varName)
								colNames = get(gh.cycleGUI.tblCycle,'ColumnName');
								colNames = [colNames; {varName{:}}];
								set(gh.cycleGUI.tblCycle,'ColumnName',colNames);
							end
							colIndex = length(state.cycle.cycleTableColumns) + length(state.cycle.cycleTableColumnsUserAdded);
						else
							% determine if we have a numeric, a string, or a logical
							numericVal = str2double(val);
							if ~isnan(numericVal)
								% make sure we don't load a numeric to a cell that should hold a string ('motorStepPos')
								colIndex = find(ismember(state.cycle.cycleTableColumns,key));
								if ~ischar(tableData{index,colIndex(1)})
									val = numericVal;
								end
							elseif strcmp(val,'true')
								val = true;
							elseif strcmp(val,'false')
								val = false;								
							end
							state.cycle.cycleTableStruct(index).(key) = val;
							colIndex = find(ismember(state.cycle.cycleTableColumns,key));
						end

						% write the table data
						tableData{index,colIndex} = val;
					else
						state.cycle.cycleConfigPaths{index} = val;
					end
                end
            end
        end
        currentLine = fgetl(fID);
    end

    fclose(fID);
    
    % update the GUI uitable to reflect the just-loaded data
    set(gh.cycleGUI.tblCycle,'Data',tableData);
	
	% cache any referenced config files
	for i = 1:size(state.cycle.cycleTableStruct,1)
		if i <= length(state.cycle.cycleConfigPaths) && ~isempty(state.cycle.cycleConfigPaths{i}) && ~isempty(state.cycle.cycleTableStruct(i).configName)
			fileName = fullfile(state.cycle.cycleConfigPaths{i}, [state.cycle.cycleTableStruct(i).configName '.cfg']);
			cacheConfiguration(fileName);
		end
	end
end

