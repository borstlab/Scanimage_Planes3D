%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  Bind a name to a channelId-boardId pair.
%%
%%  Channel names CAN NOT be overwritten. The name must first be denamed,
%%  before it can be bound to another channelId-boardId pair.
%%
%%  [OBJ channel] = nameChannel(OBJ, boardId, channelId, ioFlag, 'name')
%%
%%  Created - Tim O'Connor 11/27/03
%%
%%  MODIFICATIONS:
%           11/11/04 Tim O'Connor TO111104a: Track when put/get calls for data, for debugging purposes.
%           TO111204a - Implemented a listener. - Tim O'Connor 11/12/04
%           TO012005b - Fix direct indexing into AO/AI array. -- Tim O'Connor 1/20/05
%           TO030105a - Allow multiple channel lifecycle listeners. See TO011204a. - Tim O'Connor 3/1/05
%           TO040605A - Copy BufferingConfig property. -- Tim O'Connor 4/6/05
%           TO041305A - The Matlab 6.5 default for BufferingConfig is unusable. -- Tim O'Connor 4/13/05
%           TO112205C - Implement lifecycle listeners using the @callbackManager. -- Tim O'Connor 11/22/05
%           TO011706C - Augmented the ignorelist to include 'BufferingConfig', 'ChannelSkew', 'TriggerCondition'. -- Tim O'Connor 1/17/06
%           TO012706E - Augmented the ignorelist. -- Tim O'Connor 1/27/06
%           TO022706D - Optimization. Try remembering which properties have been modified, to reduce the time taken to actually set the board properties. -- Tim O'Connor 2/27/06
%           TO022806A - Initialization of the property modification flags needs to be a vector, not a square matrix (the `1` was left out by mistake). -- Tim O'Connor 2/28/06
%           VI021808A - Remove 'ClockSource' from the Ignore List -- Vijay Iyer 02/18/08
%           VI043008A - Remove 'BufferingConfig' and 'BufferingMode' from the ignorelist, and eliminate the special BufferingConfig handling for Matlab 6.5 -- Vijay Iyer 04/30/08
%
%%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2003
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function varargout = nameChannel(dm, boardId, channelId, ioFlag, name)
global gdm;

%Make sure it doesn't already exist.
if getChannelIndex(dm, name)
    errmsg = sprintf('The channelName %s is already in use.', name);
    error(errmsg);
end

index = getChannelIndex(dm, boardId, channelId, ioFlag);

if index
    errmsg = sprintf('Channel %s on board %s is already in use as %s.', num2str(channelId), num2str(boardId), gdm(dm.ptr).channels(index).name);
    error(errmsg);
end

len = length(gdm(dm.ptr).channels) + 1;

%These are properties which should not be copied automatically.
% ignorelist = {'BufferingConfig', 'Channel', 'Running'};
ignorelist = {'Channel', 'ChannelSkew', 'ChannelSkewMode', 'EventLog', ...
    'InitialTriggerTime', 'ManualTriggerHwOn', 'MaxSamplesQueued', 'Running', 'Sending', 'TriggerChannel', 'TriggerCondition', ...
    'TriggersExecuted'};%TO040605A, TO011706C, TO012706E, VI021808A, VI043008A

if ioFlag == 0
    gdm(dm.ptr).channels(len).aoProps = properties2cellarray(getAO(dm, boardId), ignorelist);%TO012005b
    gdm(dm.ptr).channels(len).aoPropsModificationFlags = ones(size(gdm(dm.ptr).channels(len).aoProps, 1), 1);%TO022706D %TO022806A
elseif ioFlag == 1
    gdm(dm.ptr).channels(len).aiProps = properties2cellarray(getAI(dm, boardId), ignorelist);%TO012005b
    gdm(dm.ptr).channels(len).aiPropsModificationFlags = ones(size(gdm(dm.ptr).channels(len).aiProps, 1), 1);%TO022706D %TO022806A
end

%%%%%%%%%%Commented out (VI043008A)
%The Matlab 6.5 default for BufferingConfig is unusable. -- TO041305A Tim O'Connor 4/13/05
% if ioFlag == 0
%     for i = 1 : size(gdm(dm.ptr).channels(len).aoProps, 1)
%         if strcmpi(gdm(dm.ptr).channels(len).aoProps{i, 1}, 'BufferingConfig')
%             gdm(dm.ptr).channels(len).aoProps{i, 2} = [];
%             break;
%         end
%     end
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Fill in the channel structure.
gdm(dm.ptr).channels(len).boardId = boardId;
gdm(dm.ptr).channels(len).channelId = channelId;
gdm(dm.ptr).channels(len).name = name;
gdm(dm.ptr).channels(len).lastData = 0;
gdm(dm.ptr).channels(len).state = 0;
gdm(dm.ptr).channels(len).chanProps = {};
gdm(dm.ptr).channels(len).ioFlag = ioFlag;
%TO111104a - Track when put/get calls for data, for debugging purposes. -- Tim O'Connor 11/11/04
gdm(dm.ptr).channels(len).lastPutEventString = 'NONE';
gdm(dm.ptr).channels(len).lastGetEventString = 'NONE';
gdm(dm.ptr).channels(len).data = [];

varargout{1} = dm;

%Return the new channel's structure.
varargout{2} = gdm(dm.ptr).channels(len);

%TO112205C
fireEvent(gdm(dm.ptr).cbm, 'channelCreation');
addEvent(gdm(dm.ptr).cbm, [name 'Start']);
addEvent(gdm(dm.ptr).cbm, [name 'Stop']);
addEvent(gdm(dm.ptr).cbm, [name 'Trigger']);

%TO112205C
% %TO111204a - Implemented a listener. - Tim O'Connor 11/12/04
% %TO030105a - The structure of the listener storage has been changed, to allow multiples. - Tim O'Connor 3/1/05
% for i = 1 : size(gdm(dm.ptr).channelCreationListeners, 1)
%     try
%         listener = gdm(dm.ptr).channelCreationListeners{i, 2};
%         if strcmpi(class(listener), 'function_handle')
%             feval(listener);
%         elseif strcmpi(class(listener), 'char')
%             eval(listener);
%         elseif strcmpi(class(listener), 'cell')
%             feval(listener{:});
%         else
%             warning('Failed to notify channel creation listener, unrecognized callback type: %s', class(listener));
%         end
%     catch
%         if strcmpi(class(listener), 'function_handle')
%             listenerString = ['@' func2str(listener)];
%         elseif strcmpi(class(listener), 'char')
%             listenerString = listener;
%         elseif strcmpi(class(listener), 'cell')
%             listenerString = ['{@' func2str(listener{1}) ', ...}'];
%         end
%         warning('Failed to notify channel creation listener: %s - %s', listenerString, lasterr);
%     end
% end

return;