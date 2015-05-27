function removeCachedConfiguration(fileName)
%REMOVECACHEDCONFIGURATION Clears all cached configuration data for a given config file.
	
	global state;
	
	if nargin < 1 || isempty(fileName)
		state.configCache.remove(state.configCache.keys);
	elseif state.configCache.isKey(fileName)
		state.configCache.remove(fileName);
	end
end

