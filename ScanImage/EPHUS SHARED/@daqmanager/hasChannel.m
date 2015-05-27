%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  See if a channel exists.
%%
%%  Returns 0 if the channel does not exist, a number
%%  greater than 0 otherwise.
%%
%%  logical = hasChannel(OBJ, name)
%%  logical = hasChannel(OBJ, boardId, channelId,ioflag) 
%%      ioFlag - 1 if the channel is an input channel, 0 if it's an output channel.
%%
%%  Created - Tim O'Connor 2/26/04
%%
%%  Changed:
%%   TO012005c Tim O'Connor 1/20/05 - The varargin{:} expansion seems to produce cells
%%                                    in some cases.
%%   TO0400505A Tim O'Connor 4/5/05 - Watch out for queries on empty strings.
%%   VI103006 Vijay Iyer 10/30/06 - The board/channel-ID search must also have an "ioflag"
%%
%%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2003
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function yOrN = hasChannel(dm, varargin)

yOrN = 0;

if nargin ~=2 && nargin~=4 %VI103006
    error('Function must have either 2 or 4 arguments');
end

%TO012005c
if length(varargin) == 1
    if strcmpi(class(varargin{1}), 'cell')
        varargin = varargin{1};
    end
end
if length(varargin) == 1
    %TO0400505A
    if isempty(varargin{1})
        return;
    end
    yOrN = getChannelIndex(dm, varargin{1});
else %4 argumetn case
    yOrN = getChannelIndex(dm, varargin{1}, varargin{2},varargin{3});
end
% yOrN = getChannelIndex(dm, varargin{:});

return;
