% getDaqmanager
%
% SYNTAX
%  dm = getDaqmanager
%
%  Factory method, returns "the" daqmanager.
%
% NOTES:
%
% CHANGES:
%
% Created 2/2/05 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function dm = getDaqmanager
global gdm;

dm = [];

if isempty(gdm)
    dm = daqmanager('nidaq');
else
    dm = daqmanager(1);
end

return;