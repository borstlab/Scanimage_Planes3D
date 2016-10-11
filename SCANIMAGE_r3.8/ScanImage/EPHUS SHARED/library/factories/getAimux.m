% getAimux
%
% SYNTAX
%  aim = getAimux
%
%  Factory method, returns "the" aimux.
%
% NOTES
%
% CHANGES
%
% Created 2/24/05 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function aim = getAimux
global analoginputmultiplexers;

aim = [];

if isempty(analoginputmultiplexers)
    aim = aimux(getDaqmanager);
else
    aim = aimux(2);
end

return;