%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  Retrieve an analog output object.
%%
%%  AI = getAI(OBJ, channelName)
%%  Retrieve the object containing the named channel.
%%
%%  AI = getAI(OBJ, boardId)
%%  Retrieve the object with the specified boardId.
%%
%%  AO = getAO(OBJ, 'ChannelIndex', channelIndex)
%%  For internal class use only.
%%
%%  Created - Tim O'Connor 11/27/04
%%
%%  Changed
%%   TO021805a: Tim O'Connor 2/18/05 - Reordered an if/else construct to make more sense.
%%   TO021805b: Tim O'Connor 2/18/05 - Changed a ~= to an ==.
%%   TO061605A: Tim O'Connor 6/16/05 - Watch out for empty tags.
%%   TO010606E: Tim O'Connor 1/6/06 - Optimization(s). Is there any good reason to be checking the 'Tag'?
%%   TO011206A: Tim O'Connor 1/12/06 - Watch out for not found objects in the optimized form. Related to TO010606E. -- Tim O'Connor 1/12/06
%%
%%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2003
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function ai = getAI(dm, varargin)
global gdm;

ai = [];

if length(varargin) == 2
    ai = getAIByBoardID(dm, gdm(dm.ptr).channels(varargin{2}).boardId);%TO012005b
elseif strcmpi(class(varargin{1}), 'char')
    ai = getAIByChannelName(dm, varargin{1});
else
    ai = getAIByBoardId(dm, varargin{1});
end

return;

%-------------------------------------------------------
%TO012005b
function ai = getAIByBoardId(dm, boardId)
global gdm;

%TO010606E
if boardId <= length(gdm(dm.ptr).ais)
    ai = gdm(dm.ptr).ais{boardId};
    %TO011206A - Watch out for empties in the optimized form. -- Tim O'Connor 1/12/06
    if ~isempty(ai)
        return;
    end
end

%TO010606E
% for i = 1 : length(gdm(dm.ptr).ais)
%     tag = get(gdm(dm.ptr).ais{i}, 'Tag');
%     %TO061605A
%     if ~isempty(tag)
%         if str2num(tag(end)) == boardId
%             ai = gdm(dm.ptr).ais{i};
%             return;
%         end
%     end
% end

error('Input board-id not found: %s', num2str(boardId));

% %Make sure the boardId is okay.
% if boardId > length(gdm(dm.ptr).ais)
%     errmsg = sprintf('boardId out of range: 1-%s : %s', num2str(length(gdm(dm.ptr).ais)), num2str(boardId));
%     error(errmsg);
% end
% 
% %TO012005b
% ai = gdm(dm.ptr).ais{boardId};

return;

%-------------------------------------------------------
function ai = getAIByChannelName(dm, channelName)
global gdm;

if isempty(channelName)
    error('channelName must not be empty.');
end

ai = [];
index = getChannelIndex(dm, channelName);

if index > 0
    %TO012005b
    %TO021805a - Moved `gdm(dm.ptr).channels(index).ioFlag ~= 1` to be under `if index > 0`, intead of having `if index > 0` be an elseif to the ioFlag check.
    if gdm(dm.ptr).channels(index).ioFlag == 1
        %TO021805b - Changed `gdm(dm.ptr).channels(index).ioFlag ~= 1` to `gdm(dm.ptr).channels(index).ioFlag == 1`
        ai = getAIByBoardId(dm, gdm(dm.ptr).channels(index).boardId);
    end
else
    errMsg = sprintf('No channel found with name: ''%s''', channelName);
    error(errMsg);
end

return;