%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  Retrieve AI properties from the table of AI properties for this channel.
%%
%%  PROPERTY = takeAIProperty(OBJ, 'channelName', 'PROPERTY_NAME')
%%
%%  Created - Tim O'Connor 11/29/04
%%
%%  Changed:
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function property = takeAIProperty(dm, name, key)
global gdm;

property = [];

chIndex = getChannelIndex(dm, name);
if ~chIndex
    errmsg = sprintf('No channel found with name: %s.', name);
    error(errmsg);
end

aiProps = gdm(dm.ptr).channels(chIndex).aiProps;
rowIndex = getRowIndex(aiProps, key);
if rowIndex > -1
    property = aiProps{rowIndex, 2};
else    
    errMsg = sprintf('AIProperty ''%s'' not found for channel ''%s''.', key, name);
    error(errMsg);
end

return;    