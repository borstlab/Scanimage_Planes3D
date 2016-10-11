%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  Unbind a name from a channelId-boardId pair.
%%
%%  Channel names CAN NOT be overwritten. The name must first be dename,
%%  before it can be bound to another channelId-boardId pair.
%%
%%  channelName = denameInputChannel(OBJ, boardId, channelId, ioFlag)
%%  channelName = denameInputChannel(OBJ, name)
%%
%%  Created - Tim O'Connor 11/27/04
%%
%%  Changed:
%%      TO111204a - Implemented a listener. - Tim O'Connor 11/12/04
%%      TO030105a - Allow multiple channel lifecycle listeners. See TO011204a. - Tim O'Connor 3/1/05
%%      TO112205C - Implement lifecycle listeners using the @callbackManager. -- Tim O'Connor 11/22/05
%%
%%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2003
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function channelName = denameChannel(dm, varargin)
global gdm;

index = 0;
%Check arguments and find a channel to dename.
if nargin == 2
    index = getChannelIndex(dm, varargin{1});
elseif nargin == 4
    %TO12704d - Require the ioFlag.
    index = getChannelIndex(dm, varargin{1}, varargin{2}, varargin{3});
else
    error('Wrong number of arguments.');
end

%Found a channel to dename.
if index
    %Return the denamed channel.
    channelName = gdm(dm.ptr).channels(index).name;
    
    %Disable the channel before denaming it.
    if gdm(dm.ptr).channels(index).state ~= 0
       disableChannel(dm, channelName);
    end
    
    %Remove it from the channels structure.
    channels = gdm(dm.ptr).channels(1 : index - 1);
    if index < length(gdm(dm.ptr).channels)
        channels(index : length(gdm(dm.ptr).channels) - 1) = gdm(dm.ptr).channels(index + 1 : length(gdm(dm.ptr).channels));
    end
    
    %Put the shortened array into place.
    gdm(dm.ptr).channels = channels;
else
    %None found, generate an error.
    if nargin == 2
        errmsg = sprintf('Failed to dename channel. No channel found with name: %s', varargin{1});
    else
        errmsg = sprintf('Failed to dename channel. No channel found with boardId-channelId pair: %s-%s', num2str(varargin{1}), num2str(varargin{2}));
    end
    
    error(errmsg);
end

%TO112205C
fireEvent(gdm(dm.ptr).cbm, 'channelRemoval');
removeEvent(gdm(dm.ptr).cbm, [channelName 'Start']);
removeEvent(gdm(dm.ptr).cbm, [channelName 'Stop']);
removeEvent(gdm(dm.ptr).cbm, [channelName 'Trigger']);

%TO112205C
% %TO111204a - Implemented a listener. - Tim O'Connor 11/12/04
% %TO030105a - The structure of the listener storage has been changed, to allow multiples. - Tim O'Connor 3/1/05
% for i = 1 : size(gdm(dm.ptr).channelRemovalListeners, 1)
%     try
%         listener = gdm(dm.ptr).channelRemovalListeners{i, 2};
%         if strcmpi(class(listener), 'function_handle')
%             feval(listener);
%         elseif strcmpi(class(listener), 'char')
%             eval(listener);
%         elseif strcmpi(class(listener), 'cell')
%             feval(listener{:});
%         else
%             warning('Failed to notify channel removal listener, unrecognized callback type: %s', class(listener));
%         end
%     catch
%         warning('Failed to notify channel removal listener: %s', lasterr);
%     end
% end

return;