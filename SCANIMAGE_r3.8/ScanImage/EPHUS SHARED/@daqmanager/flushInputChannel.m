%  daqmanager/flushInputChannel(dm, channelName) - Calls any waiting samplesAcquireFcn for a given channel, to flush all data.
%
% USAGE
%  flushChannel(dm, channelName)
%  flushChannel(dm, channelName, samples)
%    dm - A @daqmanager instance
%    channelName - A valid input channel or a cell array of input channel names.
%    samples - The expected number of samples to be flushed.
%
% NOTES
%
% CHANGES
%  TO061605B: Issue an error if the device is still running. -- Tim O'Connor 6/16/05
%  TO062405E: This should solve the problems with flushing data and queued SamplesAcquiredFcn callbacks. -- Tim O'Connor 6/24/05
%  TO101105C: Flush the data while the device is running, otherwise stopping via the daqmanager deletes the data when it removes the channels. -- Tim O'Connor 10/11/05
%  TO121305A: Optimize a bit, don't request things inside a loop when it's not necessary more than once. -- Tim O'Connor 12/13/05
%  TO121505H: Block until the 'SamplesPerTrigger' property has been reached. -- Tim O'Connor 12/15/05
%  TO123005G: Allow interrupting callbacks (if they exist) a chance to execute. This may actually help in flushing the specified buffer, since it facilitates data flow. -- Tim O'Connor 12/30/05
%  TO010606A: Massive rewrite, to round-robin across channels, thus (possibly) allowing data to come in without wasting computer cycles. See Sun's UltraSPARC T1 pipeline architecture. -- Tim O'Connor 1/6/06
%  TO011706B: Watch out for empty properties from the AI objects. -- Tim O'Connor 1/17/06
%  TO011906A: The number of channels on the board can (and often will) change as the channels get iterated. -- Tim O'Connor 1/19/06
%  TO011906B: Watch out for the case when the channels have been removed. -- Tim O'Connor 1/19/06
%  TO012606A: Reset indices (use indices2 now) variable for a second round of pruning. -- Tim O'Connor 1/26/06
%  TO020806C: There was some redundancy as well as some ordering stupidity in gathering board properties. -- Tim O'Connor 2/8/06
%  TO020806D: Update the channel count when no channels are found. -- Tim O'Connor 2/8/06
%  TO021406B: Do not append to the channel count array, just overwrite onto it. -- Tim O'Connor 2/14/06
%  TO021406C: Removed erroneous sampleRate factor. -- Tim O'Connor 2/14/06
%  TO021406D: If samplesAvailable is greater than 0, then we need to flush the data. So, change any(samplesAcquired < expectedSamples) to any(samplesAcquired <= expectedSamples). -- Tim O'Connor 2/14/06
%  TO021406E: If it seems to be taking a very long time, introduce a time delay. This becomes an issue on faster CPUs. -- Tim O'Connor 2/14/06
%  TO021406G: Check for trigger executions. -- Tim O'Connor 2/14/06
%  TO021406H: Batch the continue commands. -- Tim O'Connor 2/14/06
%  TO021706A: Change the loop condition to dedistribute the `any` statement, so it now applies to the totality of conditionals across the boards. -- Tim O'Connor 2/16/06
%  TO033106D: Remove all calls to flushInputChannel, the semaphores should handle all of this cleanly now. -- Tim O'Connor 3/31/06
%
% Created 6/2/05 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function flushInputChannel(this, channelName)
global gdm;

warning('DEPRECATED - TO033106D: Remove all calls to flushInputChannel, the semaphores should handle all of this cleanly now. -- Tim O''Connor 3/31/06\n%s', getStackTraceString);
return;

% DEBUG = 'NO_CHANNELS_SPECIFIED';
% if strcmpi(class(channelName), 'cell')
%     DEBUG = channelName{1};
%     for i = 2 : length(channelName)
%         DEBUG = [DEBUG ', ' channelName{i}];
%     end
% elseif ~isempty(channelName)
%     DEBUG = channelName;
% end
% fprintf(1, '%s - @daqmanager/flushInputChannel: ''%s''\n', datestr(now), DEBUG);

if isempty(channelName)
    return;
end

% fprintf(1, '%s - @daqmanager/flushInputChannel\n', datestr(now));
if ~strcmpi(class(channelName), 'cell')
    channelName = {channelName};
end

%Marshall the AI objects.
boardIDs = [];
if strcmpi(class(channelName), 'cell')
    for i = 1 : length(channelName)
        index = getChannelIndex(this, channelName{i});
        boardIDs(length(boardIDs) + 1) = gdm(this.ptr).channels(index).boardId;
    end
    %Prune the list so that only requested AIs (and no duplicates) are dealt with.
    ai = {gdm(this.ptr).ais{unique(boardIDs)}};
else
    ai = getAI(this, channelName);
end

%Compute the expected samplesAcquired value for each object.
expectedSamples = zeros(1, length(ai));
samplesAvailable = expectedSamples;
indices = [];

for i = 1 : length(ai)
    %TO011906B - Watch out for the case when the channels have been removed. -- Tim O'Connor 1/19/06
    if isempty(ai{i}.Channel)
% fprintf(1, '@daqmanager/flushInputChannel: NO_CHANNELS for ''%s''\n', get(ai{i}, 'Name'));
        continue;
    end

    %TO020806C - There was some redundancy here as well as some ordering issues. -- Tim O'Connor 2/8/06
    samplesPerTriggerTemp = get(ai{i}, 'SamplesPerTrigger');
    sampleRateTemp = get(ai{i}, 'SampleRate');
    samplesAcquiredTemp = get(ai{i}, 'SamplesAcquired');
    samplesAvailableTemp = get(ai{i}, 'SamplesAvailable');
    triggersExecutedTemp = get(ai{i}, 'TriggersExecuted');%TO021406G

    %TO011706B - Check for empty values (I'm not currently sure what induces this condition, but it does occur). -- Tim O'Connor 1/17/06
    if isempty(samplesPerTriggerTemp) | isempty(sampleRateTemp) | isempty(samplesAcquiredTemp) | isempty(samplesAvailableTemp) | isempty(triggersExecutedTemp)
% fprintf(1, '@daqmanager/flushInputChannel: NO_BOARD_PROPS for ''%s''\n', get(ai{i}, 'Name'));
        continue;
    end
    
    %TO020806C - There was some redundancy here as well as some ordering issues. -- Tim O'Connor 2/8/06
    samplesPerTrigger(i) = samplesPerTriggerTemp;
    sampleRate(i) = sampleRateTemp;
    expectedSamples(i) = samplesPerTrigger(i);%TO021406C
    samplesAcquired(i) = samplesAcquiredTemp;
    samplesAvailable(i) = samplesAvailableTemp;
    triggersExecuted(i) = triggersExecutedTemp;%TO021406G
    callbacks{i} = get(ai{i}, 'SamplesAcquiredFcn');

    if isempty(callbacks{i})
        channelList = ai(i).Channel(1).ChannelName;
        for j = 2 : length(ai(i).Channel)
            channelList = [channelList ', ' ai(i).Channel(j).ChannelName];
        end
        fprintf(2, '@daqmanager/flushInputChannel - Warning: Can not flush data for a channel on a board with no ''SamplesAcquiredFcn'' desginated. Ignoring ''%s''\n', channelList);
    elseif ~strcmpi(class(callbacks{i}), 'char') & ~strcmpi(class(callbacks{i}), 'function_handle') & ~strcmpi(class(callbacks{i}), 'cell')
        channelList = ai(i).Channel(1).ChannelName;
        for j = 2 : length(ai(i).Channel)
            channelList = [channelList ', ' ai(i).Channel(j).ChannelName];
        end
        fprintf(2, '@daqmanager/flushInputChannel - Warning: Can not flush data due to an invalid ''SamplesAcquiredFcn''. Ignoring ''%s''\n', channelList);
    else
        indices(length(indices) + 1) = i;
    end
end
if isempty(indices)
% fprintf(1, '@daqmanager/flushInputChannel: no boards selected ''%s''\n', get(ai{i}, 'Name'));
    return;
end
%Prune the lists, based on invalid callbacks.
expectedSamples = expectedSamples(indices);
samplesAcquired = samplesAcquired(indices);
samplesAvailable = samplesAvailable(indices);
triggersExecuted = triggersExecuted(indices);%TO021406G
callbacks = {callbacks{indices}};
ai = {ai{indices}};

indices2 = [];%TO012606A - This needs to be reset to prune again, this time based on channel existence. It had been reusing indices without resetting it.
channels = [];
for i = 1 : length(ai)
    if ~isempty(ai{i}.Channel)
        channels(length(channels) + 1) = length(ai{i}.Channel);
        indices2(length(indices2) + 1) = i;%TO012606A
    end
end
ai = {ai{indices2}};%TO012606A

if all(samplesAcquired == 0)
    triggers = [];
    for i = 1 : length(ai)
        triggers(i) = get(ai{i}, 'TriggersExecuted');
    end

    if all(triggers == 0)
% fprintf(1, '%s - @daqmanager/flushInputChannel: NOT_TRIGGERED\n', datestr(now));
        return;
    end
end

% fprintf(1, '%s - @daqmanager/flushInputChannel - boards: %s\n', datestr(now), mat2str(indices));%TO012606A - Which indices should get displayed here?
%Now iterate over the objects, attempting to flush each as much as possible.
loopMax = 150 * length(ai);%20 is an arbitrary choice and may need to be empirically tuned. Changed to 150 on 2/14/06
loopPauseIndex = 75 * length(ai);%TO021406E
loopCounter = 0;
eventdata.Type = 'SamplesAcquired';
eventdata.Data.RelSample = NaN;
continueLoop = 0;%TO021406H
noChannelCounter = 0;
noTriggerCounter = 0;
noSamplesCounter = 0;

% fprintf(1, '%s - @daqmanager/flushInputChannel: while condition = %s\n', datestr(now), ...
%     num2str(loopCounter < loopMax & any(samplesAvailable > 0) & any(samplesAcquired <= expectedSamples) & any(channels ~= 0)));
% fprintf(1, '@daqmanager/flushInputChannel: %1.0f, %1.0f, %1.0f, %1.0f\n', loopCounter < loopMax, any(triggersExecuted & samplesAvailable), any(triggersExecuted & (samplesAcquired <= expectedSamples)), any(channels ~= 0));
%TO021406D: If samplesAvailable is greater than 0, then we need to flush the data. So, change any(samplesAcquired < expectedSamples) to any(samplesAcquired <= expectedSamples). -- Tim O'Connor 2/14/06
%TO021406G: The samples are only counted if a trigger has been executed to go with them. -- Tim O'Connor 2/14/06
%TO021706A: Moved the any statement(s), so it applies across boards on the totality of conditions. -- Tim O'Connor 2/16/06
while loopCounter < loopMax & any(triggersExecuted & samplesAvailable & (samplesAcquired <= expectedSamples) & (channels ~= 0))
    loopCounter = loopCounter + 1;%TO020806D - Put this at the top, since when it's at the bottom it is defeated by the continue statements. -- Tim O'Connor 2/8/06
    aiIndex = mod(loopCounter, length(ai)) + 1;
    
    %TO021406E: If it seems to be taking a very long time, introduce a time delay. -- Tim O'Connor 2/14/06
    if loopCounter > loopPauseIndex
        pause(0.003);
        drawnow;
    end
    
    continueLoop = 0;%TO021406H

% if ~isempty(ai{aiIndex}.Channel)
%     DEBUG = ai{aiIndex}.Channel(1).ChannelName;
%     for j = 2 : length(ai{aiIndex}.Channel)
%         DEBUG = [DEBUG ', ' ai{aiIndex}.Channel(j).ChannelName];
%     end
% end
% fprintf(1, '%s - @daqmanager/flushInputChannel: ''%s''\n', datestr(now), DEBUG);

    %TO021406G: Check for trigger executions. -- Tim O'Connor 2/14/06
    if get(ai{aiIndex}, 'TriggersExecuted') < 1
%         fprintf(1, '%s - @daqmanager/flushInputChannel: NO_TRIGGER\n', datestr(now));
        triggersExecuted(aiIndex) = 0;
        continueLoop = 1;%TO021406H
        noTriggerCounter = noTriggerCounter + 1;
    end

    if isempty(ai{aiIndex}.Channel)
%         fprintf(1, '%s - @daqmanager/flushInputChannel: NO_CHANNELS\n', datestr(now));
        %TO020806D
        channels(aiIndex) = 0;
        for i = 1 : length(ai)
           if i ~= aiIndex
               if ~isempty(ai{i}.Channel)
                   channels(i) = length(ai{i}.Channel);
              end
          end
        end
        continueLoop = 1;%TO021406H
        noChannelCounter = noChannelCounter + 1;
    end
    
    %TO021406H
    if continueLoop
        continueLoop = 0;
        continue;
    end
    
    if get(ai{aiIndex}, 'SamplesAvailable') == 0
        samplesAcquired(aiIndex) = get(ai{aiIndex}, 'SamplesAcquired');
        samplesAvailable(aiIndex) = get(ai{aiIndex}, 'SamplesAvailable');
% fprintf(1, '%s - @daqmanager/flushInputChannel: NO_SAMPLES\n', datestr(now));
        noSamplesCounter = noSamplesCounter + 1;
        continue;
    end

    %Execute the object's samplesAcquiredFcn to get the data processed.
    callback = callbacks{aiIndex};
    for j = 1 : length(ai{aiIndex}.Channel)
% fprintf(1, '@daqmanager/flushInputChannel: Processing callback for ''%s''.\n', ai{aiIndex}.Channel(j).ChannelName);
        if strcmpi(class(callback), 'cell')
            if length(callback) > 1
                %TO062405E: This should solve the problems with flushing data and queued SamplesAcquiredFcn callbacks.
                if strcmpi(class(callback{2}), 'aimux')
                    set(callback{2}, 'attemptProcessWithStoppedDevice', 1);
                    feval(callback{1}, ai{aiIndex}, eventdata, callback{2:end});
                    set(callback{2}, 'attemptProcessWithStoppedDevice', 0);
                else
                    feval(callback{1}, ai{aiIndex}, eventdata, callback{2:end});
                end
            else
                feval(callback{1}, ai{aiIndex}, eventdata, callback{2:end});
            end
        elseif strcmpi(class(callback), 'function_handle')
            feval(callback, ai{aiIndex}, eventdata);
        end
    end
    
    samplesAcquired(aiIndex) = get(ai{aiIndex}, 'SamplesAcquired');
    samplesAvailable(aiIndex) = get(ai{aiIndex}, 'SamplesAvailable');
    
    for i = 1 : length(ai)
        if ~isempty(ai{i}.Channel)
            %TO021406B - Do not append to the channel count array, just overwrite onto it. -- Tim O'Connor 2/14/06
            channels(i) = length(ai{i}.Channel);
        else
            channels(i) = 0;%TO021406B
        end
    end
end

if loopCounter >= loopMax
    fprintf(2, '%s - daqmanager/flushInputChannel: I''m giving up on flushing data!\n', datestr(now));
    fprintf(2, ' loopCounter: %s\n loopMax: %s\n samplesAvailable: %s\n samplesAcquired: %s\n expectedSamples: %s\n channels: %s\n triggersExecuted: %s\n', ...
        num2str(loopCounter), num2str(loopMax), num2str(samplesAvailable), num2str(samplesAcquired), num2str(expectedSamples), num2str(channels), num2str(triggersExecuted));
    fprintf(2, ' noChannelCounter: %s\n noTriggerCounter: %s\n noSamplesCounter: %s\n', ...
        num2str(noChannelCounter), num2str(noTriggerCounter), num2str(noSamplesCounter));
    for i = 1 : length(ai)
        ai{i}
    end
    fprintf(2, '%s\n\n', getStackTraceString);
end

return;
% 
% for i = 1 : length(ai)
%     %TO061605B, TO101105C
%     if strcmpi(get(ai{i}, 'Running'), 'On')
%         %error('Device ''%s'' can not be flushed while it is running.', get(ai{i}, 'Name'));%TO101105C
%     end
%     
%     if isempty(ai{i}.Channel)
%         continue;
%     end
%     
%     if get(ai{i}, 'SamplesAvailable') == 0
%         continue;
%     end
%     
%     %First see if queued up samplesAcquiredFcns are processing the data.
% %     available = get(ai{i}, 'SamplesAvailable');
% %     pause(.5);
% %     counter = 0;
% %     while get(ai{i}, 'SamplesAvailable') < available
% %         pause(.5);
% %         counter = counter + 1;
% %         if counter > 600
% %             fprintf(2, '%s - daqmanager/flushInputChannel: Waited too long to flush data, probably because queued SamplesAcquiredFcn callbacks are still executing. Aborting flush...\n', datestr(now));
% %             return;
% %         end
% %     end
%     
%     %TO121305A - This callback was being grabbed inside the while loop.
%     callback = get(ai{i}, 'samplesAcquiredFcn');
%     loopCounter = 0;
%     loopMax = get(ai{i}, 'SamplesAvailable');
%     
%     %TO121505H - Keep an eye on the 'SamplesPerTrigger' too.
%     samplesPerTrigger = get(ai{i}, 'SamplesPerTrigger');
%     
%     %It shouldn't have to loop more times than there are samples.
%     while (get(ai{i}, 'SamplesAvailable') > 0 & loopCounter <= loopMax) | ...
%             (get(ai{i}, 'SamplesAcquired') < samplesPerTrigger & loopCounter <= loopMax)
%         loopCounter = loopCounter + 1;
% 
%         %callback = get(ai{i}, 'samplesAcquiredFcn');%TO121305A
%         if isempty(callback)
%             data = getData(ai{i}, get(ai{i}, 'SamplesAvailable'));
%         elseif strcmpi(class(callback), 'char')
%             eval(callback);
%         else
%             eventdata.Type = 'SamplesAcquired';
%             eventdata.Data.RelSample = NaN;
%             for j = 1 : length(ai{i}.Channel)
%                 if strcmpi(class(callback), 'cell')
%                     if length(callback) > 1
%                         %TO062405E: This should solve the problems with flushing data and queued SamplesAcquiredFcn callbacks.
%                         if strcmpi(class(callback{2}), 'aimux')
%                             set(callback{2}, 'attemptProcessWithStoppedDevice', 1);
%                             feval(callback{1}, ai{i}, eventdata, callback{2:end});
%                             set(callback{2}, 'attemptProcessWithStoppedDevice', 0);
%                         else
%                             feval(callback{1}, ai{i}, eventdata, callback{2:end});
%                         end
%                     else
%                         feval(callback{1}, ai{i}, eventdata, callback{2:end});
%                     end
%                 elseif strcmpi(class(callback), 'function_handle')
%                     feval(callback, ai{i}, eventdata);
%                 else
%                     error('Invalid SamplesAcquiredFcn type - ''%s''', class(callback));
%                 end
%             end
%         end
%         
%         if loopCounter > 20
%             fprintf(2, 'daqmanager/flushInputChannel: I''m giving up on flushing data!\n');
%             return;
%         end
%         
%         drawnow;%TO123005G: This will let interrupting calls get executed, such as other SamplesAcquiredFcn calls.
%     end
% end
% 
% return;