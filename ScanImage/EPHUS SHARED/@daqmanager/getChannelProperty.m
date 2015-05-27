%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  Call-through to set properties on channel objects.
%%
%%  PROPERTY_VALUE = setChannelProperty(OBJ, channelName, 'PROPERTY_NAME')
%%
%%  Works like the standard 'get' function, except it takes a channelName as well as an object
%%  as the first arguments.
%%
%%  Created - Tim O'Connor 11/6/03
%%
%%  Changed:
%%           1/27/04 Tim O'Connor TO12704a: Use "pointers". See daqmanager.m for details.
%%           1/27/04 Tim O'Connor T012704b: Fixed copy/paste error.
%%
%%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2003
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function property = setChannelProperty(dm, channelName, propertyName)

property = {};
index = getChannelIndex(dm, channelName);

if index < 1
    error(sprintf('No channel found with identifier: %s', channelName));
end

%T012704b: This had getChannelProperty as the call, which must've been
% an error from copying and pasting.
property = putChannelProperty(dm, channelName, propertyName);

return;