function loadCachedConfiguration(key)
%LOADCACHEDCONFIGURATION Loads a cached configuration.
	global state;

	if nargin < 1 || isempty(key)
		error('Please specify a valid key by which the cached-config is stored.');
	end

	if ~isKey(state.configCache,key)
		error('The specified config file has not been cached.');
	end
	
	cacheStruct = state.configCache(key);
	
	%setStatusString('Loading cached config...');
	try
        state.internal.loading = 1;
		traverseCacheStruct(cacheStruct);
        state.internal.loading = 0;
        
        if state.internal.channelChanged
            applyChannelSettings(); %this will call applyConfigurationSetings() as welll
        elseif state.internal.configurationChanged
            applyConfigurationSettings;
        end

	catch ME
        state.internal.loading = 0;
        resetConfigName();
		error('An error occurred while loading a cached configuration: %s', most.idioms.reportError(ME));
    end
    
%     try
%         applyChannelSettings();
%     catch ME
%         resetConfigName();
%         error('An error occurred while applying cached configuration settings: %s',most.idioms.reportError(ME));
%     end

	
	setStatusString('Config loaded');
	
	% restore the config name
	if isfield(cacheStruct.state,'configName') && isfield(cacheStruct.state,'configPath')
		state.configName = cacheStruct.state.configName;
		state.configPath = cacheStruct.state.configPath;
	else
		[pathName,fileName,~] = fileparts(key);
		state.configName = fileName;
		state.configPath = pathName;
	end
	updateGUIByGlobal('state.configName');
end

function resetConfigName()
state.configName = '';
state.configPath = '';
updateGUIByGlobal('state.configName');
end

function traverseCacheStruct(s,currentStructure)
	%TRAVERSECACHESTRUCT Recursively traverses all nodes of a nested strucure, loading all cached config data.
	
	if nargin < 2 || isempty(currentStructure)
		currentStructure = '';
	end
	
	% set up persistent variables to maintain state between recursive calls
	persistent openStructure;
	if isempty(openStructure) || isempty(currentStructure)
		openStructure = false;
	end
	persistent endStructure;
	if isempty(endStructure) || isempty(currentStructure)
		endStructure = false;
	end
	
	fields = fieldnames(s);
	
	try
		for i = 1:length(fields)
			field = fields{i};
			
			% if we've finished with a sub-structure, update the 'currentStructure' string
			if endStructure
				indices = strfind(currentStructure,'.');
				if ~isempty(indices)
					currentStructure = currentStructure(1:indices(end)-1);
				end
				endStructure = false;

				openStructure = openStructure(1:end-1);
			end

			if isstruct(s.(field))
				% if we see a sub-structure, update the 'currentStructure' string and recurse one level deeper

				if isempty(currentStructure)
					currentStructure = field;
				else
					currentStructure = [currentStructure '.' field];
					openStructure = [openStructure true];
				end

				traverseCacheStruct(s.(field),currentStructure);
			elseif ~isempty(s.(field))
				% otherwise, assign the value

				[topName,structName,fieldName] = structNameParts(currentStructure);
				if ~exist(topName,'var')
					eval(['global ' topName ';']);
				end

				val = s.(field);

				% add enclosing quotes to any strings
				if ~isempty(val) && ischar(val)
					val = ['''' val ''''];
				end

				% cell arrays require some extra work...convert it to a string that eval() can use.
				if iscell(val)
					cellString = '{';
					for cellVal = val
						cellString = [cellString '''' strrep(cellVal{1},'''','''''') ''' '];
					end
					cellString = [cellString '}'];
					val = cellString;
				end

				% handle numeric values
				if isnumeric(val)
					if length(val) > 1
						val = mat2str(val);
					else
						val = num2str(val);
					end
				end

				% handle any empty values
				if isempty(val)
					if ischar(eval([currentStructure '.' field]))
						val = '''''';
					else
						val = '[]';
					end
                end

                currVal = eval([currentStructure '.' field]);
                eval([currentStructure '.' field '=' val ';']);
                newVal = eval([currentStructure '.' field]);
                
                if ~isequal(currVal,newVal)
                    updateGUIByGlobal([currentStructure '.' field],'Callback',1);
                end                                
			end
		end
	catch ME
		error(['An error occurred while loading a cached configuration: ' ME.message]);
	end
	% we've iterated through all fields, mark the structure to be 'closed'
	if openStructure(end)
		endStructure = true;
	end
end
