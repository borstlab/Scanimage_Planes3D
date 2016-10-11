%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  Retrieve AO properties from the table of AO properties for this channel.
%%
%%  PROPERTY = takeAOProperty(OBJ, 'channelName', 'PROPERTY_NAME')
%%
%%  Created - Tim O'Connor 11/11/03
%%
%%  Changed:
%%           1/27/04 Tim O'Connor TO12704a: Use "pointers". See daqmanager.m for details.
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function property = takeAOProperty(dm, name, key)
global gdm;

property = [];

chIndex = getChannelIndex(dm, name);
if ~chIndex
    errmsg = sprintf('No channel found with name: %s.', name);
    error(errmsg);
end

aoProps = gdm(dm.ptr).channels(chIndex).aoProps;
rowIndex = getRowIndex(aoProps, key);
if rowIndex > -1
    property = aoProps{rowIndex, 2};
else    
    errMsg = sprintf('AOProperty ''%s'' not found for channel ''%s''.', key, name);
    error(errMsg);
end

return;    