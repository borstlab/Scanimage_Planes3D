% @daqmanager/startAOs - Start the correct analogoutput objects.
%
% SYNTAX
%  count = startAOs(this, name, ...)
%  count = startAOs(this, nameArray)
%   name - A valid channel name.
%   nameArray - A cell array of valid channel names.
%   count - The number of objects actually started (objects that are already running are skipped).
%
% USAGE
%  Start analogoutputs based on a list of channel names (the list may be a cell array).
%
% NOTES
%  Moved from a subfunction in startChannel to a private function in the flass.
%
% CHANGES
%  TO080606A: See @daqmanager/startChannel for changes prior to refactoring. -- Tim O'Connor 8/6/06
%
% Created 8/6/06 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2006
function count = startAOs(dm, varargin)
global gdm;

if length(varargin) == 1 & strcmpi(class(varargin{1}), 'cell')
    varargin = varargin{1};
end

% aoCell = {};%TO022706D
count = 0;
aoIDList = [];
gdm(dm.ptr).aoStartID = rand;

%TO010606E - Some reworking of the conditions. -- Tim O'Connor 1/14/06
for i = 1 : length(varargin)
    index = getChannelIndex(dm, varargin{i});

    %Tim O'Connor 10/4/04 TO100404a - Use `length(aoCell) + 1` instead of aos(i), because if
    %an index gets skipped, it leaves an "invalid object" in its place.
    %TO022706D: Trade `inList` using a cell array for `ismember` using a numeric array. -- Tim O'Connor 2/27/06
%     if ~inList(ao, aoCell) & gdm(dm.ptr).channels(index).ioFlag == 0
    if ~ismember(gdm(dm.ptr).channels(index).boardId, aoIDList) & gdm(dm.ptr).channels(index).ioFlag == 0
        ao = getAO(dm, gdm(dm.ptr).channels(index).boardId);%TO022706D: Don't get the ao unless it's needed (this had been outside the if statement). -- Tim O'Connor 2/27/06
        
        %TO010606E - Don't use `exist` here, it's too slow. -- Tim O'Connor 1/6/06
        if ~isempty(ao.Channel)
        %if exist('aos') & ~isempty(aos) & ~isempty(ao.Channel)
            %TO022305a
            if isempty(aoIDList)
                aos(1) = ao;%TO022305a
%                 aoCell{1} = ao; %TO022706D
                aoIDList = gdm(dm.ptr).channels(index).boardId;%TO022706D
            else
                aos(length(aos) + 1) = ao;%TO022305a
%                 aoCell{length(aos)} = ao;%TO022305a %TO022706D
                aoIDList(length(aos)) = gdm(dm.ptr).channels(index).boardId;%TO022706D
            end
            
            %TO032406F - Use startIDs to block excess event propogations. -- Tim O'Connor 3/24/06
            %TO112205C - Implement Start/Stop/Trigger listeners using the @callbackManager. -- Tim O'Connor 11/22/05
            %            This is a sort of sneaky way to implement it, by "silently" overriding values. It's still the best way to do it
            %            it might be worthwhile to put a warning/error message in setAOProperty/setAIProperty to alert users to this overwrite.
            set(ao, 'StartFcn', {@startFcn_Callback, dm}, 'StopFcn', {@stopFcn_Callback, dm, gdm(dm.ptr).aoStartID}, ...
                'TriggerFcn', {@triggerFcn_Callback, dm});
            gdm(dm.ptr).aoStartIDs(getBoardID(dm, get(ao.Channel(1), 'ChannelName'))) = gdm(dm.ptr).aoStartID;
        end

        %Kluge - See line the createChannels subfunction (~84) for a description of the error.
        %          set(aos(i), 'TransferMode', 'Interrupt');
    end
end

%TO010606E - Don't use `exist` here, it's too slow. -- Tim O'Connor 1/6/06
%if exist('aos') == 1 & ~isempty(aos)
%TO022706D: Swapped `aoCell` for `aoIDList`. -- Tim O'Connor 2/27/06
if ~isempty(aoIDList)
% f = fopen('C:\Matlab6p5\work\daq-states-before-start.txt', 'a');
% fprintf(f, '\r\n----------------------------\r\n%s - AO object(s) properties\r\n', datestr(now));
% captureDaq(f, aos);
% fclose(f);
    start(aos);
    count = length(aos);
end

return;