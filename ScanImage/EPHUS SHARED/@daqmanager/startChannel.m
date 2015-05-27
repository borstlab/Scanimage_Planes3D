%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Begin sending/recieving data on this channel.
%
%  startChannel(OBJ, 'name', ...)
%
%%  NOTES
%    Start/Stop listeners are called before any actual work (daq preparation or starting) is done by this function. - TO012005a
%
%%  CHANGES
%      TPMOD_1: Modified by Tom Pologruto 1/8/04:  Bug when looping in
%      addData subfunction, there was some error handling padding.  moved
%      the call to subs. into data to before the padding calls.
%
%      TO11904a: Tim O'Connor 1/19/04 - Improved clarity of warning message.
%      TO12704a: Tim O'Connor 1/27/04 - Use "pointers". See daqmanager.m for details.
%      TO12704d: Tim O'Connor 1/27/04 - Add support for input channels.
%      TO033104b: Tim O'Connor 3/31/04 - Add ability to display data sent to the hardware buffer.
%      TO072104a: Tim O'Connor 7/21/04 - Print the actual hardware channel # instead of the index.
%      TO072104b: Tim O'Connor 7/21/04 - Factored out the stack trace construction.
%      TO092804a: Tim O'Connor 9/28/04 - This had been requesting getAI using index, not the channel name.
%      TO092804b: Tim O'Connor 9/28/04 - Can't reference fields in an empty struct?
%      TO100404a: Tim O'Connor 10/4/04 - Use `length(aoCell) + 1` instead of aos(i), because if an index gets skipped, it leaves an "invalid object" in its place.
%      TO012005a: Tim O'Connor 1/20/05 - Implemented a pair of listeners for monitoring channel stops/starts. See @AOMUX.
%      TO012705d: Tim O'Connor 1/27/05 - Use `length(aiCell) + 1` instead of ais(i), because if an index gets skipped, it leaves an "invalid object" in its place.
%      TO021605a: Tim O'Connor 2/16/05 - Pulled TeX character escapes out of getStackTraceString, and moved them into texSafe.
%      TO022305a: Tim O'Connor 2/23/05 - Corrected indexing, in the event of a skip.
%      TO041305A: Tim O'Connor 4/13/05 - The Matlab 6.5 default for BufferingConfig is unusable.
%      TO112205C: Tim O'Connor 11/22/05 - Allow per-channel event listeners. Implement all state/lifecycle listeners using the @CALLBACKMANAGER.
%      TO112205A: Tim O'Connor 11/22/05 - Creating the channels in order seems to prevent the TO042104a bug.
%      TO120105D: Tim O'Connor 12/1/05 -  Fire the start events before adding the data, since that will often put data on the channel.
%      TO123105B: Tim O'Connor 12/31/05 (1/3/06) - Don't issue a warning from stopFcn_Callback if the loop runs many times. Maybe remove the loop altogether?
%      TO010406A: Tim O'Connor 1/4/06 - Take into account the change of board IDs from numbers to strings between Traditional NI-DAQ and DAQmx.
%      TO010506E: Tim O'Connor 1/5/06 - Force stack traces into error messages, when appropriate.
%      TO010506F: Tim O'Connor 1/5/06 - Be sure to notify all listeners of a stop event.
%      TO010606E: Tim O'Connor 1/6/06 - Optimization(s).
%      TO011706D: Tim O'Connor 1/17/06 - Take into account NIDAQmx labelling scheme.
%      TO011906D: Tim O'Connor 1/19/06 - Look out for when channels are deleted inside the loop.
%      TO012706F: Tim O'Connor 1/27/06 - Implement the allowMultistart functionality (allowing channels on the same board to be started at separate times, restarting where necessary).
%      TO014106A: Tim O'Connor 1/31/06 - Must convert into a numeric array in two steps, it can't be done in one.
%      TO013106B: Tim O'Connor 1/31/06 - Make sure the same channel isn't added multiple times.
%      TO013106C: Tim O'Connor 1/31/06 - Handle differences between cell arrays and char arrays when retrieving the channel name field.
%      TO013106D: Tim O'Connor 1/31/06 - Only restart those boards which have been stopped.
%      TO020106B: Tim O'Connor 2/1/06 - Fixed missing '{' and '}' pair.
%      TO022706D: Tim O'Connor 2/27/06 - Optimization(s). Rewrote `inList` to take advantage of `ismember`.
%                                        Use flags to determine which properties actually need to be modified on the board.
%     TO030606A: Tim O'Connor 3/6/06 - The stopFcn must not be executed on a channel restart while adding new channels.
%     TO031006F: Tim O'Connor 3/10/06 - Only bother with propogating stop events if a trigger has been executed, otherwise it's a "meaningless" stop event.
%     TO031106G: Tim O'Connor 3/10/06 - Only restart boards that have been stopped by this call, that requires checking against the initial varargin list.
%     TO032006B: Tim O'Connor 3/21/06 - Enforce a default -10 to 10 V range.
%     TO032406F: Tim O'Connor 3/24/06 - Try to mitigate effects of superfluous calls of the StopFcn callback by using randomized startIDs.
%     TO070606B: Tim O'Connor 7/7/06 - Enforce a default -10 to 10 V range on the output as well as the input.
%     TO080406E: Tim O'Connor 8/4/06 - Factored out createChannels, setProperties, and addData into private methods.
%     TO080606A: Tim O'Connor 8/6/06 - Continued TO080406E after being interrupted by a massive power failure.
%     VI073108A: Vijay Iyer 7/31/08 - For 2008b compatibility, do not uncoditionally 
%     VI081208A: Vijay Iyer 8/12/08 - Replaced errant 'aos' with 'ais'
%
%% CREDITS
%  Created - Tim O'Connor 11/11/03
%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2003
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function startChannel(dm, varargin)
global gdm;
% varargin{:}
% getStackTraceString
% fprintf(1, '-------------------------------------------------------\n\n');
% tic

if length(varargin) == 1 & strcmpi(class(varargin{1}), 'cell')
    varargin = varargin{1};
end
% t1 = toc, tic
%Check for errors before getting into the meat of the function.
%This may or may not be most efficient. It's certainly more efficient in the error cases.
for i = 1 : length(varargin)
    index = getChannelIndex(dm, varargin{i});
    
    if ~index
        errMsg = sprintf('No channel found with name %s.', varargin{i});
        error(errMsg);
    elseif gdm(dm.ptr).channels(index).state == 0
        errMsg = sprintf('Can not start channel ''%s'' because it is disabled.', varargin{i});
        error(errMsg);
    elseif gdm(dm.ptr).channels(index).state == 2
        %TO010506E - Make sure a stack trace is displayed. -- Tim O'Connor 1/5/06
        errMsg = sprintf('Can not start channel ''%s'' because it is already started.\n%s', varargin{i}, getStackTraceString);
        error(errMsg);
    end
end
% t2 = toc, tic

% t4 = toc, tic
%TO112205C
% %TO012005a - Implemented a pair of listeners for monitoring channel stops/starts. See @AOMUX. - Tim O'Connor 1/20/05
% if ~isempty(gdm(dm.ptr).channelStartListener)
%     for i = 1 : length(varargin)
%         index = find(strcmpi({gdm(dm.ptr).channelStartListener{:, 1}}, varargin{i}));
%         if isempty(index)
%             continue;
%         end
% 
%         if strcmpi(class(gdm(dm.ptr).channelStartListener{index, 2}), 'function_handle')
%             feval(gdm(dm.ptr).channelStartListener{index, 2}, varargin{i});
%         elseif strcmpi(class(gdm(dm.ptr).channelStartListener{index, 2}), 'char')
%             eval(gdm(dm.ptr).channelStartListener{index, 2});
%         elseif strcmpi(class(gdm(dm.ptr).channelStartListener{index, 2}), 'cell')
%             feval(gdm(dm.ptr).channelStartListener{index, 2}{:}, varargin{i});
%         else
%             error('Failed to notify channel start listener, unrecognized callback type: %s', class(gdm(dm.ptr).channelStartListener{index, 2}));
%         end
%     end
% end
% t5 = toc, tic
% % if strcmpi(class(varargin), 'cell')
% %     channelNameList = '';
% %     for c = 1 : length(varargin)
% %         channelNameList = [channelNameList varargin{c} ', '];
% %     end
% %     fprintf(1, '@daqmanager/startChannel - creating channels for %s\n', channelNameList(1:end-2));
% % else
% %     fprintf(1, '@daqmanager/startChannel - creating channels for %s\n', varargin);
% % end
createChannels(dm, varargin);
% % fprintf(1, '@daqmanager/startChannel - channels created.\n');

%TO012706F: If allowMultistart is on, the new channels have been created and the old have been stopped.
%           The data for the old channels must get reput. The properties are okay, but a check should be done
%           to debug conflicts, if requested.
%TO013106C: Handle differences between cell arrays and char arrays when retrieving the channel name field.
%TO031106G: Only restart boards that have been stopped by this call, that requires checking against the initial varargin list. -- Tim O'Connor 3/10/06
channelNames = {};
for i = 1 : length(gdm(dm.ptr).aos)
    if ~isempty(gdm(dm.ptr).aos{i})
        %TO013106D: Only restart those boards which have been stopped.
        if ~strcmpi(get(gdm(dm.ptr).aos{i}, 'Running'), 'On')
            if ~isempty(gdm(dm.ptr).aos{i}.Channel) && ~isempty(intersect(varargin, gdm(dm.ptr).aos{i}.Channel.ChannelName)) %TO031106G %VI073108A
                if length(gdm(dm.ptr).aos{i}.Channel) == 1
                    channelNames = cat(1, channelNames, {gdm(dm.ptr).aos{i}.Channel.ChannelName});
                elseif length(gdm(dm.ptr).aos{i}.Channel) > 1
                    channelNames = cat(1, channelNames, gdm(dm.ptr).aos{i}.Channel.ChannelName);
                end
            end
        end
    end
end
for i = 1 : length(gdm(dm.ptr).ais)
    if ~isempty(gdm(dm.ptr).ais{i})
        %TO013106D: Only restart those boards which have been stopped.
        if ~strcmpi(get(gdm(dm.ptr).ais{i}, 'Running'), 'On')
            if ~isempty(gdm(dm.ptr).ais{i}.Channel) && ~isempty(intersect(varargin, gdm(dm.ptr).ais{i}.Channel.ChannelName)) %TO031106G %VI073108A %VI081208A
                if length(gdm(dm.ptr).ais{i}.Channel) == 1
                    channelNames = cat(1, channelNames, {gdm(dm.ptr).ais{i}.Channel.ChannelName});%TO020106B
                elseif length(gdm(dm.ptr).ais{i}.Channel) > 1
                    channelNames = cat(1, channelNames, gdm(dm.ptr).ais{i}.Channel.ChannelName);
                end
            end
        end
    end
end

%TO012706F - Moved this check to after all channels have been gathered.
%TO010606E - Only do this if the option is turned on, for speed.
%Check for conflicts in the board properties.
if gdm(dm.ptr).debugMessages
    errs = generateConflictWarnings(dm, channelNames);
% t3 = toc, tic
    if ~isempty(errs)
        errMsg = '';

        for i = 1 : length(errs)
            if ~isempty(errs{i})
                errMsg = sprintf('%s %s', errMsg, errs{i});
            end
        end

        error('Possible data acquisition board conflicts found:\n%s', errMsg);
    end
end

% t6 = toc, tic
% % if strcmpi(class(varargin), 'cell')
% %     channelNameList = '';
% %     for c = 1 : length(varargin)
% %         channelNameList = [channelNameList varargin{c} ', '];
% %     end
% %     fprintf(1, '@daqmanager/startChannel - setting properties for %s\n', channelNameList(1:end-2));
% % else
% %     fprintf(1, '@daqmanager/startChannel - setting properties for %s\n', varargin);
% % end
setProperties(dm, varargin);%TO012706F - Just set the properties for the newly started channels.
% % fprintf(1, '@daqmanager/startChannel - properties set.\n');
% t7 = toc, tic

%TO120105D - Fire the start events before adding the data, since that will often put data on the channel.
%TO012706F - This has to be done for the restarted channels as well.
for i = 1 : length(channelNames)
% fprintf(1, '@daqmanager/startChannel: Firing start event for ''%s''...\n%s', channelNames{i}, getStackTraceString);
    if gdm(dm.ptr).channels(getChannelIndex(dm, channelNames{i})).ioFlag == 0
        fireEvent(gdm(dm.ptr).cbm, [channelNames{i} 'Start'], channelNames{i}, getAO(dm, channelNames{i}), []);
    elseif gdm(dm.ptr).channels(getChannelIndex(dm, channelNames{i})).ioFlag == 1
        fireEvent(gdm(dm.ptr).cbm, [channelNames{i} 'Start'], channelNames{i}, getAI(dm, channelNames{i}), []);
    else
        warning('Unsupported/Unimplemented ioFlag encountered while starting channels: %s', num2str(gdm(dm.ptr).channels(getChannelIndex(dm, channelNames{i})).ioFlag));
    end
end

% % if strcmpi(class(varargin), 'cell')
% %     channelNameList = '';
% %     for c = 1 : length(channelNames)
% %         channelNameList = [channelNameList channelNames{c} ', '];
% %     end
% %     fprintf(1, '@daqmanager/startChannel - adding data for %s\n', channelNameList(1:end-2));
% % else
% %     fprintf(1, '@daqmanager/startChannel - adding data for %s\n', channelNames);
% % end
%Specific to analog outputs.
addData(dm, channelNames);%TO012706F
% % fprintf(1, '@daqmanager/startChannel - data added.\n');
% t8 = toc, tic
%Specific to analog inputs.

% % if strcmpi(class(varargin), 'cell')
% %     channelNameList = '';
% %     for c = 1 : length(channelNames)
% %         channelNameList = [channelNameList channelNames{c} ', '];
% %     end
% %     fprintf(1, '@daqmanager/startChannel - clearing buffers for %s\n', channelNameList(1:end-2));
% % else
% %     fprintf(1, '@daqmanager/startChannel - clearing buffers for %s\n', channelNames);
% % end
clearBuffers(dm, channelNames);%TO012706F
% % fprintf(1, '@daqmanager/startChannel - buffers cleared.\n');
% t9 = toc, tic
count = startAOs(dm, channelNames);%TO012706F
% t10 = toc, tic
count = count + startAIs(dm, channelNames);%TO012706F
% t11 = toc, tic
%Mark the channels as 'Started'.
for i = 1 : length(varargin)
    gdm(dm.ptr).channels(getChannelIndex(dm, varargin{i})).state = 2;
end
% t12 = toc, tic
return;

%-------------------------------------------------------
% TO080406E - Factored out into a private method.
% function createChannels(dm, varargin)


%-------------------------------------------------------
% TO080606A - Factored out into a private method.
% function setProperties(dm, varargin)


%-------------------------------------------------------
% TO080606A - Factored out into a private method.
% function addData(dm, varargin)

%-------------------------------------------------------
% TO080606A - Factored out into a private method.
% function clearBuffers(dm, varargin)

%-------------------------------------------------------
% TO080606A - Factored out into a private method.
% function startAOs(dm, varargin)

%-------------------------------------------------------
% TO080606A - Factored out into a private method.
% function startAIs(dm, varargin)

%-------------------------------------------------------
% TO080606A - Factored out into a private method.
%TO112205C - Implement Start/Stop/Trigger listeners using the @callbackManager. -- Tim O'Connor 11/22/05


%-------------------------------------------------------
% TO080606A - Factored out into a private method.
%TO112205C - Implement Start/Stop/Trigger listeners using the @callbackManager. -- Tim O'Connor 11/22/05
%TO032406F - Use startIDs to drop excess calls. -- Tim O'Connor 3/24/06
% function stopFcn_Callback(obj, eventdata, dm, startID)

%-------------------------------------------------------
% TO080606A - Factored out into a private method.
%TO112205C - Implement Start/Stop/Trigger listeners using the @callbackManager. -- Tim O'Connor 11/22/05
% function triggerFcn_Callback(obj, eventdata, dm)

%-------------------------------------------------------
% TO080606A - Factored out into a private method.
% function isInList = inList(obj, list)

%-------------------------------------------------------
% TO080606A - Factored out into a private method.
%Added the ability to display data sent to the hardware buffer. -- Tim O'Connor 3/31/04: TO033104b
% function plotData(dm, ao, data)

%---------------------------------------------
% TO080606A - Factored out into a private method.
% function captureDaq(f, daq)

%---------------------------------------------
% TO080606A - Factored out into a private method.
% function captureValue(f, val)