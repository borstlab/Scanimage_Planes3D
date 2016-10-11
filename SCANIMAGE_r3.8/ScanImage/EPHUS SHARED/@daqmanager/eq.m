%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  '==' operator
%%
%%  Relies on the results of the underlying analog output objects.
%%  Of course, both objects must be of class 'daqmanager' and non-empty/non-zero.
%%
%%  Created - Tim O'Connor 11/6/03
%%
%%  Changed:
%%           1/27/04 Tim O'Connor TO12704a: Use "pointers". See daqmanager.m for details.
%%
%%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2003
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function bool = eq(dm1, dm2)
global gdm;

bool = 0;

if ~strcmpi(class(dm1), 'daqmanager')
    return;
elseif ~strcmpi(class(dm2), 'daqmanager')
    return;
end

if ~dm1 & dm2
    return;
elseif ~dm2
    return;    
end

if length(gdm(dm1.ptr).aos) ~= length(gdm(dm2.ptr).aos)
    return;
end

if length(gdm(dm1.ptr).channels) ~= length(gdm(dm2.ptr).channels)
    return;
end

bool = 1;
if (dm1.ptr == dm2.ptr)
    return;
end

for i = 1:length(dm1.aos)
    bool = bool & (dm1.aos{i} == dm2.aos{i});
end

return;