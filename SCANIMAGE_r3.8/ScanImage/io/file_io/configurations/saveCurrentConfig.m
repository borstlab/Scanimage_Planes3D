function out=saveCurrentConfig()
global state

out=0;
if isempty(state.configPath) | ~isdir(state.configPath)
    saveCurrentConfigAs;
    return
end
setStatusString('Saving Config...');
[fid, message]=fopen(fullfile(state.configPath, [state.configName '.cfg']), 'wt');
if fid==-1
    disp(['saveCurrentConfig: Error cannot open output file ' fullfile(state.configPath, [state.configName '.cfg']) ]);
    return
end
createConfigFileFast(1, fid, 1);

% if this is a fast-config, update the cached config structure
if state.configCache.isKey(fullfile(state.configPath, [state.configName '.cfg']))
	cacheConfiguration(fullfile(state.configPath, [state.configName '.cfg']));
end

fclose(fid);
out=1;
%cd(state.configPath); %VI110210A
resetConfigurationNeedsSaving();
setStatusString('Saved Configuration'); %VI020609A

