%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  Call-through to set properties on channel objects.
%%
%%  PROPERTIES = setChannelProperty(OBJ, channelName)
%%
%%  OBJ = setChannelProperty(OBJ, channelName, 'PROPERTY_NAME', 'PROPERTY_VALUE', ...)
%%
%%  Works like the standard 'set' function, except it takes a channelName as well as an object
%%  as the first arguments.
%%
%%  Created - Tim O'Connor 11/6/03
%%
%%  Changed:
%%      TO12904d: Tim O'Connor 1/29/04 - Use "pointers". See daqmanager.m for details. Add support for input channels.
%%
%%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2003
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function val = setChannelProperty(dm, channelName, varargin)

%Check the args.
if mod(length(varargin), 2) ~= 0
    error('Wrong number of arguments.');
end

index = getChannelIndex(dm, identifier);

if index < 1
    error(sprintf('No channel found with identifier: %s', channelName));
end

%Watch out for things that don't exist on the board, are on the 'ignore list',
%or are read only.
for i = 1 : 2 : length(varargin) - 1
    if getRowIndex(varargin{i}) < 1
        errMsg(sprintf('Property ''%s'' is either not a valid property or is not allowed to be altered.', varargin{i}));
        error(errMsg);
    end
end

if ~isnumeric(identifier)
    val = putChannelProperty(dm, identifier, varargin);
end

return;