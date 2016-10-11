%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Stop sending/recieving data on this channel.
%
%  Unlike daqmanager/stopChannel, this function will not 
%  wait until the 'Running' property for each analogoutput 
%  object is not 'On'.
%
%  OBJ = stopChannel(OBJ, 'name', ...)
%

%
%%  CHANGES
%           1/27/04 Tim O'Connor TO12704a: Use "pointers". See daqmanager.m for details.
%           8/29/05 Tim O'Connor TO082905C: Fixed bug that occurs when `ao == []`.
%           8/05/08 Vijay Iyer VI080508A: Use deleteChannels, for generality
%
%% CREDITS
%  Created - Tim O'Connor 11/24/03
%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2003
%
%% ***********************************************************
function dm = stopChannelImmediate(dm, varargin)
global gdm;

if length(varargin) == 1 & strcmpi(class(varargin{1}), 'cell')
    varargin = varargin{1};
end

aoCell = {};
aos = [];%TO082905C

%First, actually stop the channels.
%Make a list of the channels.
warnMsg = '';
for i = 1 : length(varargin)
    if gdm(dm.ptr).channels(getChannelIndex(dm, varargin{i})).state ~= 2
        warnMsg = sprintf('%s\n  Channel ''%s'' has already been stopped or has never been started.', warnMsg, varargin{i});
    end
    
    ao = getAO(dm, varargin{i});
    
    if ~inList(ao, aoCell)
        aoCell{i} = ao;
        if ~isempty(ao) %TO082905C
            aos(length(aos) + 1) = ao;
        end
    end
end

if ~isempty(warnMsg)
    warning(strcat('Attempting to stop non-started channel(s).', warnMsg));
end

%TO082905C
if ~isempty(aos)
    stop(aos);
end

%Remove the channels.
for i = 1 : length(gdm(dm.ptr).aos)
    %%%%VI080508A: Use deleteChannels for generality
    %delete(gdm(dm.ptr).aos{i}.Channel); 
    deleteChannels(gdm(dm.ptr).aos{i});
    %%%%%%%%%%%%   
end

%Now, go back and set the correct states.
for i = 1 : length(varargin)
    index = getChannelIndex(dm, varargin{i});
    gdm(dm.ptr).channels(index).state = 1;
end

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