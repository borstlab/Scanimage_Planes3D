function key = cacheConfiguration(fileNameOrVarNames)
    % CACHECONFIGURATION Caches a configuration to memory, either from a file, or from a list of state-vars.
    %
    % fileNameOrVarNames: a string containing a full path to the file to be
    % cached, or a cell array of strings containing the names of state-vars to
    % be cached.  NOTE: if a filename is given, the configuration will be cached
	% using the filename as the key, otherwise, if a list of varNames is given, 
	% the configuration will be cached using 'CYCLE_CACHE' as the key.
	%
	% key: a string containing the key under which the cached config is stored.
    %%

    global state;
    
	if nargin < 1 || isempty(fileNameOrVarNames)
		% if no args, cache the current configuration
		fileNameOrVarNames = [state.configVars 'state.configName' 'state.configPath'];
	end
	
    if iscell(fileNameOrVarNames)
        % cache the given list of state-vars
        varNames = fileNameOrVarNames;
		key = 'CYCLE_CACHE';
		
		if state.configCache.isKey(key)
			cacheStruct = state.configCache(key);
		else
			cacheStruct = struct('state','');
		end
        
		for i = 1:length(varNames)
            varName = varNames{i};
            
			% varName looks like 'a.b.c'...build an eval-able string like 'cacheStruct.a.b.c'...
			remainder = varName;
			varString = '';
			isValid = true;
			while length(remainder) > 1
				[token,remainder] = strtok(remainder,'.');
				if isempty(varString) 
					varString = token;
				elseif isfield(eval(varString),token)
					varString = [varString '.' token];
				else
					isValid = false;
					break;
				end
			end
			
			if ~isValid
				continue;
			end
			
			% store the current state-var to the struct
            eval(['cacheStruct.' varString ' = ' varName ';']);
		end
		
        % cache this configuration
		state.configCache(key) = cacheStruct;

    elseif ischar(fileNameOrVarNames)
        % cache the given filename
        fileName = fileNameOrVarNames;
		key = fileName;
        
        % open file and read in by line ignoring comments
        fID=fopen(fileName, 'r');
        if fID==-1
            disp(['cacheConfiguration: Error: Unable to open file ' fileName ]);
            return
        else
            out=1;
            [fullName, per, mf]=fopen(fID);
            fclose(fID);
            [pname, fname, ext]=fileparts(fullName);
        end
        file = textread(fullName,'%s', 'commentstyle', 'matlab', 'delimiter', '\n');

        % store the cache structure to the global map	
        try
            state.configCache(key) = parseInitializationFile(file,'cacheCFG');
        catch ME
            error(['An error occurred while caching the config file: ' ME.message]);
        end 
    end
end
