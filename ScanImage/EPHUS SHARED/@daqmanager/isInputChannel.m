%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  See if a specified input channel exists.
%%
%%  Returns 0 if the channel does not exist, a number
%%  greater than 0 otherwise.
%%
%%  logical = isInputChannel(OBJ, name)
%%  logical = isInputChannel(OBJ, boardId, channelId)
%%
%%  Created - Tim O'Connor 11/12/04
%%
%%  MODIFICATIONS
%       VI022008A Vijay Iyer 2/20/2008 -- Syntax error for passing on varargin
%       VI022008B Vijay Iyer 2/20/2008 -- Access ioFlag via channels field, not directly from gdm
%       VI022008C Vijay Iyer 2/20/2009 -- Added in the missing key piece of logic
%
%%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2004
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
function yOrN = isInputChannel(dm, varargin)
global gdm;

yOrN = getChannelIndex(dm, varargin{:}); %VI022008A -- was 'varargin' instead of 'varargin{:}'

if yOrN > 0
    if gdm(dm.ptr).channels(yOrN).ioFlag ~= 1 %VI022008B 
        yOrN = 0; %VI022008C
    end
else
    yOrN = 0;
end

yOrN = logical(yOrN);

return;