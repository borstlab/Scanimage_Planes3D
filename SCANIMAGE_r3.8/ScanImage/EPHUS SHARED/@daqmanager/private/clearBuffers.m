% @daqmanager/clearBuffers - Clear the buffers on the correct analoginput objects.
%
% SYNTAX
%  clearBuffers(this, name, ...)
%  clearBuffers(this, nameArray)
%   name - A valid channel name.
%   nameArray - A cell array of valid channel names.
%
% USAGE
%  Clear analoginputs based on a list of channel names (the list may be a cell array).
%
% NOTES
%  Moved from a subfunction in startChannel to a private function in the flass.
%
% CHANGES
%  TO080606A: See @daqmanager/startChannel for changes prior to refactoring. -- Tim O'Connor 8/6/06
%
% Created 8/6/06 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2006
function clearBuffers(dm, varargin)
global gdm;

if length(varargin) == 1 & strcmpi(class(varargin{1}), 'cell')
    varargin = varargin{1};
end

%Iterate over all analog outputs and active channels.
for i = 1 : length(varargin)

    index = getChannelIndex(dm, varargin{i});
    
    %If this is an input object, clear the data.
    if gdm(dm.ptr).channels(index).ioFlag == 1
% fprintf(1, '@daqmanager/private/clearBuffers: Clearing buffers for channel ''%s''...\n', gdm(dm.ptr).channels(index).name);
        gdm(dm.ptr).channels(index).data = [];
    end
    
end
return;