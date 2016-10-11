% clearAimuxSemaphores - Forcibly clear all semaphores for aimux instances.
%
% SYNTAX
%  clearAimuxSemaphores
%  clearAimuxSemaphores(indices)
%   indices - Clears semaphores only for instances referenced by these pointer values.
%
% USAGE
%  This function may both cause and correct instability due to timing.
%
% NOTES
%
% CHANGES
%
% Created 3/30/06 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2006
function clearAimuxSemaphores(varargin)
global analoginputmultiplexers;

if isempty(varargin)
    indices = 1 : length(analoginputmultiplexers);
else
    indices = [varargin{:}];
end

for i = 1 : length(indices)
    try
        forciblyClearAllSemaphores(aimux(indices(i)));
    catch
        warning('An error occured while clearing semaphores for aimux instance %s: %s\n', num2str(indices(i)), lasterr);
    end
end

return;