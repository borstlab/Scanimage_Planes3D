% @daqmanager/startAIs - Start the correct analogoutput objects.
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
function count = startAIs(dm, varargin)
global gdm;

if length(varargin) == 1 & strcmpi(class(varargin{1}), 'cell')
    varargin = varargin{1};
end

aiCell = {};
count = 0;
gdm(dm.ptr).aiStartID = rand;

%TO010606E - Some reworking of the conditions. -- Tim O'Connor 1/14/06
for i = 1 : length(varargin)
    index = getChannelIndex(dm, varargin{i});
    %Tim O'Connor 9/28/04 - This had been requesting getAI using index,
    %not the channel name. TO092804a
    ai = getAI(dm, varargin{i});

    if ~inList(ai, aiCell) & gdm(dm.ptr).channels(index).ioFlag == 1
        if ~isempty(ai) & ~isempty(ai.Channel)
            %TO022305a
            if isempty(aiCell)
                ais(1) = ai;%TO012705d, TO022305a
                aiCell{1} = ai;
            else
                ais(length(ais) + 1) = ai;%TO012705d, TO022305a
                aiCell{length(ais)} = ai;%TO022305a
            end
            
            %TO032406F - Use startIDs to block excess event propogations. -- Tim O'Connor 3/24/06
            %TO112205C - Implement Start/Stop/Trigger listeners using the @callbackManager. -- Tim O'Connor 11/22/05
            %            This is a sort of sneaky way to implement it, by "silently" overriding values. It's still the best way to do it
            %            it might be worthwhile to put a warning/error message in setAOProperty/setAIProperty to alert users to this overwrite.
            set(ai, 'StartFcn', {@startFcn_Callback, dm}, 'StopFcn', {@stopFcn_Callback, dm, gdm(dm.ptr).aiStartID}, ...
                'TriggerFcn', {@triggerFcn_Callback, dm});
            gdm(dm.ptr).aiStartIDs(getBoardID(dm, get(ai.Channel(1), 'ChannelName'))) = gdm(dm.ptr).aiStartID;
        end
    end
end


% a = get(ais)
% ais = analoginput('nidaq', 1);
% addchannel(ais, 1, 'AXOPATCH_200B_1_scaledOutput');
% fnames = fieldnames(a);
% for i = 1 : length(fnames)
%     if ~ismember({'channel', 'eventlog'}, lower(fnames{i}))
%         set(ais, fnames{i}, a.(fnames{i}));
%     end
% end
% gdm.ais{1} = ais;

%TO010606E - Don't use `exist` here, it's too slow. -- Tim O'Connor 1/6/06
%if exist('ais') == 1 & ~isempty(ais)
if ~isempty(aiCell)
% f = fopen('C:\Matlab6p5\work\daq-states-before-start.txt', 'a');
% fprintf(f, '\r\n----------------------------\r\n%s - AI object(s) properties\r\n', datestr(now));
% captureDaq(f, ais);
% fclose(f);
    start(ais);
    count = length(ais);
end

return;