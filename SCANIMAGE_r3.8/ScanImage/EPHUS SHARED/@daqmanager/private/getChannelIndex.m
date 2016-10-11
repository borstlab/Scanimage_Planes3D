%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  Finds the index in the object's channel array for the named channel.
%%  Returns 0 if no corresponding channel was found.
%%
%%  channelIndex = getChannelIndex(OBJ, 'channelName')
%%  channelIndex = getChannelIndex(OBJ, boardId, channelId, ioFlag)
%%  channelIndex = getChannelIndex(OBJ, boardId, channelId, ioFlag)
%%
%%  Created - Tim O'Connor 11/11/03
%%
%%  Changed:
%%           1/27/04 Tim O'Connor TO12704a: Use "pointers". See daqmanager.m for details.
%%           1/20/05 Tim O'Connor TO012005d: Allow i/o agnostic lookups.
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function index = getChannelIndex(dm, varargin)
global gdm;

index = 0;

if isempty(gdm(dm.ptr).channels)
    return;
end

if nargin == 2
    %Look for a channel by name.

    %Check the arguments.
    if ~strcmpi(class(varargin{1}), 'char')
        error('Invalid argument, ''channelName'' must be a string.');
    end
    if isempty(varargin{1})
        error('channelName must not be empty.');
    end

    %Search for the name.
    for i = 1 : length(gdm(dm.ptr).channels)
        if strcmpi(gdm(dm.ptr).channels(i).name, varargin{1}) %Case insensitive.
            index = i;
            return
        end
    end
elseif nargin == 3
    %TO012005d
    %Look for a channel by boardId-channelId pair.
    for i = 1 : length(gdm(dm.ptr).channels)
        if (gdm(dm.ptr).channels(i).boardId == varargin{1}) & (gdm(dm.ptr).channels(i).channelId == varargin{2})
            index = i;
            return
        end
    end
elseif nargin == 4
    %Look for a channel by boardId-channelId pair.
    for i = 1 : length(gdm(dm.ptr).channels)
        if (gdm(dm.ptr).channels(i).boardId == varargin{1}) & (gdm(dm.ptr).channels(i).channelId == varargin{2}) & ...
                (gdm(dm.ptr).channels(i).ioFlag == varargin{3})
            index = i;

            return
        end
    end
else
    error('Wrong number of arguments.');
end

return;