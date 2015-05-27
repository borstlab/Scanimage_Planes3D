% getAimux
%
% SYNTAX
%  aom = getAomux
%
%  Factory method, returns "the" aomux.
%
% NOTES
%
% CHANGES
%
% Created 2/24/05 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function aom = getAomux
global analogoutputmultiplexers;

aom = [];

if isempty(analogoutputmultiplexers)
    aom = aomux(getDaqmanager);
else
    aom = aomux(1);
end

return;