% @daqmanager/stopFcn_Callback - A daqtoolbox callback event handler.
%
% SYNTAX
%  startFcn_Callback(obj, eventdata, dm)
%   obj - The daqobject that initiated the event.
%   eventdata - The eventdata supplied by the daqtoolbox.
%   dm - The @daqmanager instance for the event initiating daqobject.
%
% USAGE
%
% NOTES
%  Moved from a subfunction in startChannel to a private function in the flass.
%
% CHANGES
%  TO080606A: See @daqmanager/startChannel for changes prior to refactoring. -- Tim O'Connor 8/6/06
%
% Created 8/6/06 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2006
function stopFcn_Callback(obj, eventdata, dm, startID)
global gdm;
% fprintf(1, '@daqmanager/startChannel/stopFcn_Callback: %s\n%s', get(obj, 'Tag'), getStackTraceString);
%TO030606A - The stopFcn must not be executed on a channel restart while adding new channels. -- Tim O'Connor 3/6/06
if gdm(dm.ptr).allowMultistart & gdm(dm.ptr).restartingChannelForChannelAddition
    return;
end

info = daqhwinfo(obj);
boardIndex = getNumericSuffix(info.ID);
if strcmpi(info.SubsystemType, 'AnalogInput')
    if gdm(dm.ptr).aiStartIDs(boardIndex) ~= startID
% fprintf(1, '@daqmanager/startChannel/stopFcn_Callback: Denying stop event for AI-%s:%s - %s\n', get(obj, 'Tag'), num2str(gdm(dm.ptr).aiStartIDs(boardIndex)), num2str(startID));
        return;
    else
% fprintf(1, '@daqmanager/startChannel/stopFcn_Callback: Allowing stop event for AI-%s:%s\n', get(obj, 'Tag'), num2str(startID));
        gdm(dm.ptr).aiStartIDs(boardIndex) = rand;
    end
elseif strcmpi(info.SubsystemType, 'AnalogOutput')
    if gdm(dm.ptr).aoStartIDs(boardIndex) ~= startID
% fprintf(1, '@daqmanager/startChannel/stopFcn_Callback: Denying stop event for AO-%s:%s - %s\n', get(obj, 'Tag'), num2str(gdm(dm.ptr).aoStartIDs(boardIndex)), num2str(startID));
        return;
    else
% fprintf(1, '@daqmanager/startChannel/stopFcn_Callback: Allowing stop event for AO-%s:%s\n', get(obj, 'Tag'), num2str(startID));
        gdm(dm.ptr).aoStartIDs(boardIndex) = rand;
    end
else
    warning('Unsupported subsystem type for daq object in @daqmanager/startChannel/stopFcn_Callback.');
end

if ~isempty(obj.Channel)
    %TO010506F - Index across all channels when notifying of stops (there may be listeners bound to each. -- Tim O'Connor 1/5/06
    %TO030706E: Can not use a for loop to index across channels, use a while instead. -- Tim O'Connor 3/7/06
    channelCounter = 0;
    channelCount = length(obj.Channel);
    channelCounterMax = 2 * channelCount;
    completedChannels = {};
    
    %TO031006F: Only bother with this if a trigger has been executed, otherwise it's a "meaningless" stop event. -- Tim O'Connor 3/10/06
    while length(obj.Channel) > 0 & channelCounter < channelCounterMax & (get(obj, 'TriggersExecuted') > 0 & channelCounter < channelCount)
        channelCounter = channelCounter + 1;
        %TO030706E: Let the while loop handle termination, `i` is no long a valid subscript. - Tim O'Connor 3/7/06
        %%TO011906D - Look out for when channels are deleted inside the loop. -- Tim O'Connor 1/19/06
        %if i > length(obj.Channel)
        %    break;
        %end
        channelIndex = find(~ismember(obj.Channel.ChannelName, completedChannels));
        if length(channelIndex) > 1
            %Just take the first channel for which no stop function has been called, the others will get taken care of in subsequent passes.
            channelIndex = channelIndex(1);
        elseif isempty(channelIndex)
            %There are no channels left on the board for which the stop function has not been called.
            break;
        end
        % fprintf(1, '@daqmanager/startChannel/stopFcn_Callback: Calling StopFcn for ''%s''\n', obj.Channel(channelIndex).ChannelName);
        %Label it as done, before executing the stop function, in case that function removes the channel and thus the name gets lost.
        % fprintf(1, '%s - daqmanager/startChannel/stopFcn_Callback -\n Notifying: ''%s''\n', datestr(now), obj.Channel(channelIndex).ChannelName);
        completedChannels{end + 1} = obj.Channel(channelIndex).ChannelName;
% fprintf(1, '@daqmanager/startChannel/stopFcn_Callback: firing stopEvent for %s\n', [obj.Channel(channelIndex).ChannelName 'Stop']);
% strcts = getCallbackStructs(gdm(dm.ptr).cbm, [obj.Channel(channelIndex).ChannelName 'Stop'])
% strcts(2)
% strcts(2).callbackSpec
% func2str(strcts(2).callbackSpec{1})
        fireEvent(gdm(dm.ptr).cbm, [obj.Channel(channelIndex).ChannelName 'Stop'], obj.Channel(channelIndex).ChannelName, obj, eventdata);
    end
end

%TO123105B - This may be a non-problem, and the warning only causes undue worry/confusion among users. -- Tim O'Connor 1/3/06 (Implemented 12/31/05)
% if counter >=20
%     warning('@daqmanager/startChannel/stopFcn_Callback while loop executed an unusually large number of times. The loop was forcibly terminated for the board containing channel ''%s''', obj.Channel(1).ChannelName);
%     callbacks = getCallbacksAsStrings(gdm(dm.ptr).cbm, [obj.Channel(1).ChannelName 'Stop'])
%     dm
% end

return;