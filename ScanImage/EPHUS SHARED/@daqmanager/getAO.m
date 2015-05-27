%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  Retrieve an analog output object.
%%
%%  AO = getAO(OBJ, channelName)
%%  Retrieve the object containing the named channel.
%%
%%  AO = getAO(OBJ, boardId)
%%  Retrieve the object with the specified boardId.
%%
%%  AO = getAO(OBJ, 'ChannelIndex', channelIndex)
%%  For internal class use only.
%%
%%  Created - Tim O'Connor 11/6/03
%%
%%  Changed:
%%           1/27/04 Tim O'Connor TO12704a: Use "pointers". See daqmanager.m for details.
%%           TO012005b - Fix direct indexing into AO/AI array. -- Tim O'Connor 1/20/05
%%           TO010606E - Optimization(s). Is there any good reason to be checking the 'Tag'? -- Tim O'Connor 1/6/06
%%           TO011206A: Tim O'Connor 1/12/06 - Watch out for not found objects in the optimized form. Related to TO010606E. -- Tim O'Connor 1/12/06
%%           TO072106A: Fixed typo ("ai" --> "ao"). -- Tim O'Connor 7/21/06
%%           TO091106C: Added `global gdm;` at the top of the primary function. -- Tim O'Connor 9/11/06
%%
%%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2003
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function ao = getAO(dm, varargin)
global gdm; %TO091106C

ao = [];

if length(varargin) == 2
    ao = getAOByBoardId(dm, gdm(dm.ptr).channels(varargin{2}).boardId);%TO012005b %TO072106A
elseif strcmpi(class(varargin{1}), 'char')
    ao = getAOByChannelName(dm, varargin{1});
else
    ao = getAOByBoardId(dm, varargin{1});
end

return;

%-------------------------------------------------------
%TO012005b
function ao = getAOByBoardId(dm, boardId)
global gdm;

%TO010606E
if boardId <= length(gdm(dm.ptr).aos)
    ao = gdm(dm.ptr).aos{boardId};
    %TO011206A - Watch out for empties in the optimized form. -- Tim O'Connor 1/12/06
    if ~isempty(ao)
        return;
    end
end

%TO010606E
% for i = 1 : length(gdm(dm.ptr).aos)
%     tag = get(gdm(dm.ptr).aos{i}, 'Tag');
% 
%     if str2num(tag(end)) == boardId
%         ao = gdm(dm.ptr).aos{i};
%         return;
%     end
% end

error('Output board-id not found: %s', num2str(boardId));
% %Make sure the boardId is okay.
% if boardId > length(gdm(dm.ptr).aos)
%     errmsg = sprintf('boardId out of range: 1-%s', num2str(length(gdm(dm.ptr).aos)));
%     error(errmsg);
% end
% 
% %TO012005b
% ao = gdm(dm.ptr).aos{boardId + 1};

return;

%-------------------------------------------------------
function ao = getAOByChannelName(dm, channelName)
global gdm;

if isempty(channelName)
    error('channelName must not be empty.');
end

ao = [];
index = getChannelIndex(dm, channelName);

if index < 1
    error('Invalid channelName: ''%s''', channelName);
end

if gdm(dm.ptr).channels(index).ioFlag ~= 0
    return;
elseif index > 0
    ao = getAOByBoardId(dm, gdm(dm.ptr).channels(index).boardId);%TO012005b
else
    errMsg = sprintf('No channel found with name: ''%s''', channelName);
    error(errMsg);
end

return;