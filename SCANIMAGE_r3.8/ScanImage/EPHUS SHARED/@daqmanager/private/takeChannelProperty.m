%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  Retrieve channel properties from the table of channel properties for this channel.
%%
%%  PROPERTY = takeAOProperty(OBJ, 'channelName', 'PROPERTY_NAME')
%%
%%  Created - Tim O'Connor 11/11/03
%%
%%  Changed:
%%           1/27/04 Tim O'Connor TO12704a: Use "pointers". See daqmanager.m for details.
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function property = takeChannelProperty(dm, name, key)
global gdm;

%Check the args.
if mod(length(varargin), 2) ~= 0
    error('Wrong number of arguments.');
end

chIndex = getChannelIndex(dm, name);
if ~chIndex
    errmsg = sprintf('No channel found with name: %s.', name);
    error(errmsg);
end

chanProps = gdm(dm.ptr).channels(chIndex).chanProps;
rowIndex = getRowIndex(chanProps, key);
if rowIndex > -1
   property = chanProps(rowIndex, 2);
end

return;    