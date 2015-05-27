%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  Change a field in a named channel.
%%
%%  OBJ = setChannelField(OBJ, channelName, fieldName, value)
%%
%%  Created - Tim O'Connor 11/11/03
%%
%%  Changed:
%%           1/27/04 Tim O'Connor TO12704a: Use "pointers". See daqmanager.m for details.
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function dm = setChannelField(dm, channelName, fieldName, value)
global gdm;

if isempty(channelName)
    error('channelName must not be empty.');
end

index = getChannelIndex(dm, channelName);
if ~index
    errmsg = sprintf('No channel found with name: %s', channelName);
    error(errmsg);
end

gdm(dm.ptr).channels(index) = setField(gdm(dm.ptr).channels(index), fieldName, value);

return;