function dm = daqmanager(varargin)
% DAQMANAGER An object that manages data acquisition hardware.
% The basic idea is to deal exclusively with 'channels', each with
% a dedicated purpose. The manager should avoid/resolve conflicts
% between the settings needed by each channel.
%
% USAGE:
%
%   OBJ = daqmanager;
%   OBJ = daqmanager(ADAPTOR);
%   OBJ = daqmanager(ADAPTOR <, 'NO_INPUT', boardIdVector> <, 'NO_OUTPUT', boardIdVector>);
%   OBJ = daqmanager(index);
%   OBJ = daqmanager(OBJ_2);
%
%   Takes nothing, another daqmanager, the pointer index of an existing daqmanager, or a
%   string representing the adaptor (ie. 'nidaq') as arguments.
%
%   The NO_INPUT and NO_OUTPUT properties are used to specify lists of boards
%   that are only good for input/output (respectively)
%
%   Here is an example of how to use this object for data output -
%
%   %Create an instance of the object.
%   dm = daqmanager('nidaq');
%
%   %Name a channel.
%   nameOutputChannel(dm, 1, 1, 'ch1');
%
%   %Set properties, as needed.
%   setAOProperty(dm, 'ch1', 'SampleRate', 1000);
%
%   %Enable the channel.
%   enableChannel(dm, 'ch1');
%
%   %Put data on the channel.
%   putDaqData(dm, 'ch1', data);
%
%   %Start the channel.
%   startChannel(dm, 'ch1');
%
%   %Stop the channel.
%   stopChannel(dm, 'ch1');
%
%
% DATA_STRUCTURE:
%
%   The object returned by this constructor is a struct with a single field.
%   That single field is 'ptr'. It is an index (a pointer) into a global multidimensional
%   struct. This allows a sort of 'pass-by-reference' mechanism. That way, the
%   'data' field of the real 'daqmanager' struct does not get recopied at every
%   function call. Also, the object does not need to be handed back and reassigned
%   for each function call.
%
%   The global, into which 'ptr' indexes, is tentatively called 'gdm'.
%
%   The 'daqmanager' structure is as follows:
%
%     adaptor - the adaptor for which this manager was created.
%     aos - an array of analogoutput objects.
%     channels - an array of Channel structures (one for each "named channel").
%     displayHardwareBuffer - a flag that specifies whether or not to plot the outbound data.
%     debugMessages - A flag that allows advanced error message generation to be toggled on and off. %TO010606E
%     allowMultistart - Allows a channels on a single board to be started separately, without causing errors. May stop running channels. %TO012706F
%
%     The 'channel' structure is as follows:
%
%       data - array of data to be output, only valid if the channel is an output channel.
%       state - off == 0, on == 1, started == 2.
%       boardId - the hardware board-id.
%       channelId - the hardware channel-id.
%       name - a string identifier, this is the key with which to deal with this channel.
%       lastData - the last value in data, for use in padding arrays.
%       aoProps - cell array table.
%       chanProps - cell array table.
%       ioFlag - 0 == output, 1 == input.
%
%   For methods of this class, they may access individual damanager structures using the
%   following notation:
%             global gdm;
%             gdm(dm.ptr).boardId = 0;
%             disp(gdm(dm.ptr).channelId);
%
% Created - Tim O'Connor 11/5/03
%
% Changed:
%          1/26/04 Tim O'Connor TO12604b: Use "pointers".
%          1/27/04 Tim O'Connor TO12704d: Add input channels.
%          8/6/04 Tim O'Connor TO080604b: Store ao/ai objects in cell arrays.
%          11/12/04 Timothy O'Connor TO111204a: Implemented a pair of channel lifecycle listeners. See @AIMUX.
%          1/20/05 Tim O'Connor TO012005a: Implemented a pair of listeners for monitoring channel stops/starts. See @AOMUX.
%          2/22/05 Tim O'Connor TO022205a: Created the index constructor.
%          3/1/05 Tim O'Connor TO030105a: Allow multiple channel lifecycle listeners. See TO011204a.
%          5/4/05 Tim O'Connor TO050405A: Allow saving/loading of daqmanagers.
%          11/22/05 Tim O'Connor TO112205C: Allow per-channel event listeners. Implement all state/lifecycle listeners using the @CALLBACKMANAGER.
%          TO122205A - Upgrade to Matlab 7.1, case sensitivity. -- Tim O'Connor 12/22/05
%          TO010406A - Take into account the change of board IDs from numbers to strings between Traditional NI-DAQ and DAQmx. -- Tim O'Connor 1/4/06
%                      Switch to using the ObjectConstructorName field, which fixes cross-version compatibility and also negates the noInput and noOutput lists.
%          TO010506A - When given a choice between Traditional NIDAQ vs NIDAQmx drivers, go with Traditional NIDAQ (for now). -- Tim O'Connor 1/6/06
%                      Revisit this decision when 'HwDigital' is a valid 'TriggerType' property for NIDAQmx via the daq toolbox.
%                      Open support request - THREAD ID: 1-22CKQ3 (as of 1/5/06)
%          TO010606E - Optimization(s). Created the debugMessages field, because this functionality is necessarily slow. -- Tim O'Connor 1/6/06
%          TO012706F - Added the allowMultistart property. Defaulted it to 1. -- Tim O'Connor 1/27/06
%          TO030606A - The stopFcn must not be executed on a channel restart while adding new channels. -- Tim O'Connor 3/6/06
%          TO032406F - Try to mitigate effects of superfluous calls of the StopFcn callback by using randomized startIDs. -- Tim O'Connor 3/24/06
%          TO070606A - Correct default settings when running with NIDAQmx. -- Tim O'Connor 7/6/06
%          TO072106C - Print the correct message when Traditional is found and NIDAQmx is not. -- Tim O'Connor 7/21/06
%          VI020708A - Cannot preemptively set TriggerCondition property, as its valid values are dependent on the TriggerType value.
%          VI110708A - Don't print message when DAQmx is exclusively found. This is now considered the 'normal' state of things. -- Vijay Iyer 11/07/08
%          VI022709A - Now use the DAQmx driver if it's found alongside the -- Vijay Iyer 2/27/09
%
% Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2003
global gdm;

if nargin == 0
    dm = [];
elseif (nargin == 1) | (nargin == 3) | (nargin == 5)
    if isa(varargin{1}, 'daqmanager')
        dm = varargin{1};
    elseif isnumeric(varargin{1})
        %TO022205a
        if varargin{1} <= length(gdm) & varargin{1} == floor(varargin{1})
            dm.ptr = varargin{1};
        else
            error('@daqmanager/daqmanager(index) - Invalid pointer index: %s', num2str(varargin{1}));
        end
    else        
        %Create a new daqmanager and store it.
        if length(gdm) == 0
            gdm = createNew(varargin{:});
        else
            gdm(length(gdm) + 1) = createNew(varargin{:});
        end

        %Return the pointer.
        dm.ptr = length(gdm);
    end
else
    error('Wrong number of arguments provided to ''daqmanager'' constructor.\n');
end

dm.serialized = [];%TO050405A

dm = class(dm, 'daqmanager');

return;

%-------------------------------------------------------
function dmStruct = createNew(varargin)

dmStruct.adaptor = varargin{1};
% noInput = {'PCI-6713'};
% noOutput = {};
dmStruct.channels = [];

%TO112205C - Switch to @callbackManager based implementation.
% %TO111204a - Implemented a pair of listeners. See @AIMUX. - Tim O'Connor 11/12/04
% dmStruct.channelCreationListeners = {};%TO030105a
% dmStruct.channelRemovalListeners = {};%TO030105a
% 
% %TO012005a - Implemented a pair of listeners for monitoring channel stops/starts. See @AOMUX. - Tim O'Connor 1/20/05
% dmStruct.channelStartListener = {};
% dmStruct.channelStopListener = {};

%TO112205C - Allow per-channel event listeners. -- Tim O'Connor 11/22/05
dmStruct.cbm = callbackmanager;%TO122205A
addEvent(dmStruct.cbm, 'channelCreation');
addEvent(dmStruct.cbm, 'channelRemoval');

%Look for board-ignore flags.
for i = 1 : length(varargin)
    if strcmpi(varargin{i}, 'NO_INPUT') | strcmpi(varargin{i}, 'NOINPUT')
        i = i + 1;
        if iscellstr(varargin{i})
            noInput = [noInput varargin{i}];
        elseif ischar(varargin{i})
            noInput = [noInput {varargin{i}}];
        else
            error('NO_INPUT value provided to ''daqmanager'' constructor must be a cell array of strings.\n');
        end
    elseif strcmpi(varargin{i}, 'NO_OUTPUT') | strcmpi(varargin{i}, 'NOOUTPUT')
        i = i + 1;
        if iscellstr(varargin{i})
            noOutput = [noOutput varargin{i}];
        elseif ischar(varargin{i})
            noOutput = [noOutput {varargin{i}}];
        else
            error('NO_OUTPUT value provided to ''daqmanager'' constructor must be a cell array of strings.\n');
        end
    end
end

info = daqhwinfo(dmStruct.adaptor);

%TO010506A - Choose the better supported drivers.
if strcmpi(dmStruct.adaptor, 'nidaq')
    nidaqIndices = [];
    daqmxIndices = [];
    for i = 1 : length(info.InstalledBoardIds)
        if startsWithIgnoreCase(info.InstalledBoardIds{i}, 'Dev')
            daqmxIndices(length(daqmxIndices) + 1) = i;
        else
            nidaqIndices(length(nidaqIndices) + 1) = i;
        end
    end
    
%     if ~isempty(nidaqIndices)
%         fprintf(1, '%s - @daqmanager: Found both DAQmx and Traditional NIDAQ drivers. Defaulting to ''Traditional NIDAQ''.\n', datestr(now));
%         info.InstalledBoardIds = {info.InstalledBoardIds{nidaqIndices}};
%         info.ObjectConstructorName = reshape({info.ObjectConstructorName{nidaqIndices, :}}, length(nidaqIndices), 3);
%     end

        %TO070606A - Make sure this message is accurate.
    if ~isempty(daqmxIndices) %VI022709A
        %TO072106C - Print the correct message when Traditional is found and NIDAQmx is not. -- Tim O'Connor 7/21/06
        if ~isempty(nidaqIndices)
            fprintf(1, '%s - @daqmanager: Found both NIDAQmx and Traditional NIDAQ drivers. Defaulting to ''NIDAQmx''.\n', datestr(now)); %VI022709A
        else
            fprintf(1, '%s - @daqmanager: Found only DAQmx drivers, using DAQmx...\n', datestr(now));%TO072106C - Print the correct message when Traditional is found and NIDAQmx is not.
        end            
        dmStruct.nidaqmxEnabled = 1;%TO070606A, VI022709A
        info.InstalledBoardIds = {info.InstalledBoardIds{daqmxIndices}}; %VI022709A
        info.ObjectConstructorName = reshape({info.ObjectConstructorName{daqmxIndices, :}}, length(daqmxIndices), 3);
    else
        %fprintf(1, '%s - @daqmanager: Found only NIDAQmx drivers, using NIDAQmx...\n', datestr(now)); %VI110708A
        dmStruct.nidaqmxEnabled = 0;%TO070606A, VI022709A
        info.InstalledBoardIds = {info.InstalledBoardIds{nidaqIndices}}; %VI022709A
        info.ObjectConstructorName = reshape({info.ObjectConstructorName{nidaqIndices, :}}, length(nidaqIndices), 3); %VI022709A
    end
end

for i = 1 : length(info.InstalledBoardIds)
    %TO010406 - Just checking the suffix should be sufficient for the DAQmx convention.
    if i ~= str2num(info.InstalledBoardIds{i}) & i ~= getNumericSuffix(str2num(info.InstalledBoardIds{i}))
        warnMsg = sprintf('Index does not match Board-ID, may cause conflicts later. Index: %s, Board-Id: %s', num2str(i), info.InstalledBoardIds{i});
        warning(warnMsg);
    end
    
    %TO010406A - This is a much cleaner implementation, which should work across more platform (Matlab + NIDAQ/DAQmx) versions.
    %Input
    if isempty(info.ObjectConstructorName{i, 1})
        dmStruct.ais{i} = [];
    else
        dmStruct.ais{i} = eval([info.ObjectConstructorName{i, 1} ';']);
    end
    set(dmStruct.ais{i}, 'Tag', sprintf('@DAQMANAGER_Input_board_%s', num2str(i)));%TO12704d
    %%TO070606A VI020708A
%     if dmStruct.nidaqmxEnabled
%         set(dmStruct.ais{i}, 'TriggerCondition', 'PositiveEdge');
%     end
    %Output
    if isempty(info.ObjectConstructorName{i, 2})
        dmStruct.aos{i} = [];
    else
        dmStruct.aos{i} = eval([info.ObjectConstructorName{i, 2} ';']);
    end
    set(dmStruct.aos{i}, 'Tag', sprintf('@DAQMANAGER_Output_board_%s', num2str(i)));%TO12704d
    %DigitalIO
%     if isempty(info.ObjectConstructorName{i, 3})
%     else
%     end
    
%     %TO080604b Tim O'Connor 8/6/04 - Switched to storing ao/ai objects in cell arrays. This caused changes to many files.
%     %                                It was done because of problems during deletion operations, since invalid objects might have
%     %                                been stored in the array, otherwise.
%     if ~ismember(info.BoardNames{i}, noOutput)
%         try
%             dmStruct.aos{i} = analogoutput(dmStruct.adaptor, i);
%             set(dmStruct.aos{i}, 'Tag', sprintf('@DAQMANAGER_Output_board_%s', num2str(i)));%TO12704d
%         catch
%             fprintf(2, 'Not creating analog output object for %s:%s (boardId: %s) - %s\n', ...
%                 dmStruct.adaptor, info.BoardNames{i}, num2str(info.InstalledBoardIds{i}), strrep(lasterr, sprintf('\n'), ' '));
%         end
%     else
%         dmStruct.aos{i} = [];
%     end
%     
%     if ~ismember(info.BoardNames{i}, noInput)
%         try
%             dmStruct.ais{i} = analoginput(dmStruct.adaptor, i);
%             set(dmStruct.ais{i}, 'Tag', sprintf('@DAQMANAGER_Input_board_%s', num2str(i)));%TO12704d
%         catch
%             fprintf(2, 'Not creating analog input object for %s:%s (boardId: %s) - \n  %s\n', ...
%                 dmStruct.adaptor, info.BoardNames{i}, num2str(info.InstalledBoardIds{i}), strrep(lasterr, sprintf('\n'), ' '));
%         end
%     else
%         dmStruct.ais{i} = [];
%     end
end

dmStruct.displayHardwareBuffer = logical(0);
dmStruct.debugMessages = 0;%TO010606E
dmStruct.displayAOStatus = logical(0);
dmStruct.figures.hardwareBuffers = [];
dmStruct.allowMultistart = 1;%TO012706F
dmStruct.restartingChannelForChannelAddition = 0;%TO030606A
dmStruct.aiStartID = [];%TO032406F
dmStruct.aoStartID = [];%TO032406F
dmStruct.aiStartIDs = zeros(size(info.InstalledBoardIds));%TO032406F
dmStruct.aoStartIDs = zeros(size(info.InstalledBoardIds));%TO032406F

return;