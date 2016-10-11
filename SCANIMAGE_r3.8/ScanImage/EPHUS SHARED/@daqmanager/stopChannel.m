%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Stop sending/recieving data on this channel.
%
%  This function will wait until the 'Running' property for each
%  analogoutput object is not 'On'.
%
%  stopChannel(OBJ, 'name', ...)
%
%% NOTES
%    Channel stop listeners are called after all work is done by this function. - TO012005a
%
%
%%  CHANGES:
%           1/27/04 Tim O'Connor TO12704a: Use "pointers". See daqmanager.m for details.
%           1/28/04 Tim O'Connor TO12804b: Add support for input channels.
%           8/2/04 Tom POlogruto TPMOD080204: Added try/catch for boards with invalid AI or AO.
%           8/6/04 Tim O'Connor TO080604b: Store ao/ai objects in cell arrays.
%           1/20/05 Tim O'Connor TO012005a: Implemented a pair of listeners for monitoring channel stops/starts. See @AOMUX.
%           TPMOD080204 Added Try/Catch to correct for boards without input or output (i.e. PCI 6713 )
%           TO080604b Tim O'Connor 8/6/04 - Switched to storing ao/ai objects in cell arrays. This caused changes to many files.
%                                           It was done because of problems during deletion operations, since invalid objects might have
%                                           been stored in the array, otherwise.
%           TO012005a - Implemented a pair of listeners for monitoring channel stops/starts. See @AOMUX. - Tim O'Connor 1/20/05
%           TO012505b Tim O'Connor 1/25/05 - Iterate over the aos/ais arrays, instead of the gdm(dm.ptr).aos/gdm(dm.ptr).ais arrays.
%           TO022805a Tim O'Connor 2/28/05 - Removed errant warnings.
%           TO100705D Tim O'Connor 10/7/05 - Handle non-existent channels more gracefully.
%           TO112205C Tim O'Connor 11/22/05 - Allow per-channel event listeners. Implement all state/lifecycle listeners using the @CALLBACKMANAGER.
%           VI080508A Vijay Iyer 8/5/08 - Use deleteAOChannels to deal with DAQ toolbox bug affecting sample rate property
%
%% CREDITS
%  Created - Tim O'Connor 11/11/03
%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2003
%
%% **********************************************************************************
function stopChannel(dm, varargin)
global gdm;

if length(varargin) == 1 & strcmpi(class(varargin{1}), 'cell')
    varargin = varargin{1};
end

aoCell = {};
aiCell = {};

%First, actually stop the channels.
%Make a list of the channels.
warnMsg = '';

for i = 1 : length(varargin)
    index = getChannelIndex(dm, varargin{i});
    if index < 1
        warning('Failed to find and stop channel ''%s''', varargin{i});
        continue;
    end
    
    if gdm(dm.ptr).channels(index).state ~= 2
        warnMsg = sprintf('%s\n  Channel ''%s'' has already been stopped or has never been started.', warnMsg, varargin{i});
    end

    ao = getAO(dm, varargin{i});
    ai = getAI(dm, varargin{i});

    if exist('ao') == 1 & ~isempty(ao) & ~isempty(ao.Channel) & ~inList(ao, aoCell) & gdm(dm.ptr).channels(index).state == 2
        aoCell{i} = ao;
        if exist('aos') == 1
            aos(length(aos) + 1) = ao;
        else
            aos = ao;
        end
    end
    if exist('ai') == 1 & ~isempty(ai) & ~isempty(ai.Channel) & ~inList(ai, aiCell) & gdm(dm.ptr).channels(index).state == 2
        aiCell{i} = ai;
        if exist('ais') == 1
            ais(length(ais) + 1) = ai;
        else
            ais = ai;
        end
    end
end

%Generate some warnings, if neccessary.
if ~isempty(warnMsg)
%     warning(strcat('Attempting to stop non-started channel(s).', warnMsg));
elseif exist('aos') == 1 & ~isempty(aos) & ismember('Off', aos.Sending)
%     warning('Attempting to stop channel(s) that have never been started/triggered.');
end

%Stop the analog outputs.
if exist('ais') == 1 & ~isempty(ais)
    stop(ais);
end
if exist('aos') == 1 & ~isempty(aos)
    stop(aos);

    startTime = clock;
    n = 0;%counter
    timeout = 0;%boolean
    timeoutInterval = 10; %Seconds.
    %Wait for the analogoutputs to finish their buffers.
    while ismember('On', aos.Sending) & ... 
            ismember('On', aos.Running) & ...
            ~timeout & ...
            (n < 1000)

        %Implement a simple timeout mechanism. Hardcoding is fine.
        if mod(n, 50) == 0%Don't check on every iteration.
            if (n < 1000) & (abs(etime(clock, startTime)) > timeoutInterval)
                timeout = 1;
                warnMsg = sprintf('Timeout in stopping DAQ object(s): %s seconds.', num2str(timeoutInterval));
                warning(warnMsg);
            elseif n >= 1000
                timeout = 1;
                warning('Timeout in stopping DAQ object(s).');
            end
        end

        n = n + 1;
    end
end

%Remove the channels.
% TPMOD080204 Added Try/Catch to correct for boards without input or output
% (i.e. PCI 6713 )
%TO080604b Tim O'Connor 8/6/04 - Switched to storing ao/ai objects in cell arrays. This caused changes to many files.
%                                It was done because of problems during deletion operations, since invalid objects might have
%                                been stored in the array, otherwise.
%TO012505b Tim O'Connor 1/25/05 - Iterate over the aos/ais arrays, instead of the gdm(dm.ptr).aos/gdm(dm.ptr).ais arrays.
%TO022805a Tim O'Connor 2/28/05 - Removed errant warnings.
if exist('aos') == 1
    for i = 1 : length(aos)
        try
            if ~isempty(aos(i))
                deletionArray = [];
                for j = 1 : length(aos(i).Channel)
                    if any(ismember(lower(varargin), lower(get(aos(i).Channel(j), 'ChannelName'))))
                        deletionArray(length(deletionArray) + 1) = j;
                    end
                end

                if ~isempty(deletionArray)
                    %%%%%%%%%VI080508A: Use deleteAOChannels
                    %delete(aos(i).Channel(deletionArray)); 
                    deleteAOChannels(aos(i),deletionArray); 
                    %%%%%%%%%%%%%%%%%%%%
%                 else %TO022805a
%                     warning('Failed to locate output channel(s) for deletion in set {%s}', varargin{:});
                end
            end
        catch
            warning('Skipping delete of AO on invalid output board #%s. Error: %s', num2str(i), lasterr);
        end
    end
end
if exist('ais') == 1
    for i = 1 : length(ais)
        try
            if ~isempty(ais(i))
                deletionArray = [];
                for j = 1 : length(ais(i).Channel)
                    if any(ismember(lower(varargin), lower(get(ais(i).Channel(j), 'ChannelName'))))
                        deletionArray(length(deletionArray) + 1) = j;
                    end
                end

                if ~isempty(deletionArray)
                    delete(ais(i).Channel(deletionArray));
%                 else %TO022805a
%                     warning('Failed to locate input channel(s) for deletion in set {%s}', varargin{:});
                end
            end
        catch
            warning('Skipping delete of AI on invalid input board #%s. Error: %s', num2str(i), lasterr);
        end
    end
end

%Now, go back and set the correct states.
for i = 1 : length(varargin)
    index = getChannelIndex(dm, varargin{i});
    if index ~= 0
        gdm(dm.ptr).channels(index).state = 1;
    else
        warning('Failed to find, and reset the state of, channel ''%s''.', varargin{i});
    end
end

%TO112205C
% %TO012005a - Implemented a pair of listeners for monitoring channel stops/starts. See @AOMUX. - Tim O'Connor 1/20/05
% if ~isempty(gdm(dm.ptr).channelStopListener)
%     for i = 1 : length(varargin)
%         index = find(strcmpi(gdm(dm.ptr).channelStopListener{:, 1}, varargin{i}));
%         if isempty(index)
%             continue;
%         end
% 
%         if strcmpi(class(gdm(dm.ptr).channelStopListener{index, 2}), 'function_handle')
%             feval(gdm(dm.ptr).channelStopListener{index, 2}, varargin{i});
%         elseif strcmpi(class(gdm(dm.ptr).channelStopListener{index, 2}), 'char')
%             eval(gdm(dm.ptr).channelStopListener{index, 2});
%         elseif strcmpi(class(gdm(dm.ptr).channelStopListener{index, 2}), 'cell')
%             feval(gdm(dm.ptr).channelStopListener{index, 2}{:}, varargin{i});
%         else
%             error('Failed to notify channel start listener, unrecognized callback type: %s', class(gdm(dm.ptr).channelStartListener{index, 2}));
%         end
%     end
% end
% % for i = 1 : size(gdm(dm.ptr).channelStopListener, 1)
% %     index = find(strcmpi(gdm(dm.ptr).channelStopListener{:, 1}, varargin{i}));
% %     if isempty(index)
% %         continue;
% %     end
% % 
% %     if strcmpi(class(gdm(dm.ptr).channelStopListener{i, 2}), 'function_handle')
% %         feval(gdm(dm.ptr).channelStopListener{i, 2}, varargin{i});
% %     elseif strcmpi(class(gdm(dm.ptr).channelStopListener{:, 2}), 'char')
% %         eval(gdm(dm.ptr).channelStopListener{i, 2});
% %     elseif strcmpi(class(gdm(dm.ptr).channelStopListener{:, 2}), 'cell')
% %         feval(gdm(dm.ptr).channelStopListener{i, 2}{:}, varargin{i});
% %     else
% %         error('Failed to notify channel stop listener, unrecognized callback type: %s', class(gdm(dm.ptr).channelStopListener{:, 2}));
% %     end
% % end

return;

%-------------------------------------------------------
function isInList = inList(obj, list)

isInList = 0;

for i = 1 : length(list)

    if strcmpi(class(list), 'cell')
        if obj == list{i}
            isInList = 1;
            return;
        end
    else
        if obj == list(i)
            isInList = 1;
            return;
        end
    end
end

return;