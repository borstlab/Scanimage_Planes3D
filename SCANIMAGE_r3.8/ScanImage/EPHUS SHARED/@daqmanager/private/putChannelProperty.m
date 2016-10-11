%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  Add channel properties to the table of channel properties for this channel.
%%
%%  PROPERTIES = putAOProperty(OBJ, 'channelName')
%%  OBJ = putAOProperty(OBJ, 'channelName', 'PROPERTY_NAME', 'PROPERTY_VALUE', ...)
%%
%%  Created - Tim O'Connor 11/11/03
%%
%%  Changed:
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function val = putChannelProperty(dm, name, varargin)
global gdm;

%Check the args.
if mod(length(varargin), 2) ~= 0
    error('Wrong number of arguments.');
end

chIndex = getChannelIndex(dm, name);
if ~chIndex
    errmsg = sprintf('No channel found with name: %s.', name);
    error(errmsg);
end

chanProps = gdm(dm.ptr).channels(chIndex).chanProps;
for i = 1 : 2 : length(varargin) - 1
    rowIndex = getRowIndex(chanProps, varargin{i});
    
    %Replace it, if it exists.
    if rowIndex > -1
        chanProps(rowIndex, 2) = varargin{i + 1};
    else
        %Create a new entry in the table.
        x = size(aoProps, 1) + 1);
        chanProps(x, 1) = varargin{i};
        chanProps(x, 2) = varargin{i + 1};
    end
end

gdm(dm.ptr).channels(chIndex).chanProps = chanProps;

if nargin == 2
	val = chanProps;
else
	val = dm;
end

return;    