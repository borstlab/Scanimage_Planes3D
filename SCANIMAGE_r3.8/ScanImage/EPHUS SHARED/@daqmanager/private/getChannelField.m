%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  Retrieve a field from a named channel.
%%
%%  value = getChannelField(OBJ, channelName, fieldName)
%%
%%  Created - Tim O'Connor 11/11/03
%%
%%  Changed:
%%           1/27/04 Tim O'Connor TO12704a: Use "pointers". See daqmanager.m for details.
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function value = getChannelField(dm, channelName, fieldName)
global gdm;

if isempty(channelName)
    error('channelName must not be empty.');
end

index = getChannelIndex(dm, channelName);
if ~index
    errmsg = sprintf('No channel found with name: %s', channelName);
    error(errmsg);
end

value = getField(gdm(dm.ptr).channels(index), fieldName);

return;