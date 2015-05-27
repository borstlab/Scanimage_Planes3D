%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  Retrieve an analogoutput's field for a specified channel.
%%
%%  fieldValue ... = getAOField(OBJ, boardId, channelId, fieldName, ...)
%%
%%  Created - Tim O'Connor 11/24/03
%%
%%  Changed:
%%
%%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2003
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function varargout = getAOField(dm, channelName, varargin)

nargs = nargin;
if length(varargin) == 1 & strcmpi(class(varargin{1}), 'cell')
    varargin = varargin{1};
    nargs = length(varargin);
end

if nargs < nargout + 2
    error('Too many output arguments.');
elseif nargs > nargout + 2
    warning('Too few output arguments.');
end

ao = getAO(dm, channelName);
if isempty(ao);
    errMsg = sprintf('No channel found with name ''%s''.', channelName);
    error(errMsg);
end

for i = 1 : length(varargin)
    varargout{i} = getfield(ao, varargin{i});
end

return;