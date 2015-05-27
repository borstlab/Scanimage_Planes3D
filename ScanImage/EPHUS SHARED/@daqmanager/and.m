%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  '&' operator
%%
%%  Returns true if the objects exist.
%%
%%  Created - Tim O'Connor 11/6/03
%%
%%  Changed:
%%           1/27/04 Tim O'Connor TO12704a: Use "pointers". See daqmanager.m for details.
%%
%%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2003
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function bool = and(dm1, dm2)

if strcmpi(class(dm1), 'daqmanager') & dm1.ptr > 0
    dm1 = 1;
else
    dm1 = 0;
end
    
if strcmpi(class(dm2), 'daqmanager') & dm2.ptr > 0
    dm2 = 1;
else
    dm2 = 0;
end

bool = dm1 & dm2;

return;