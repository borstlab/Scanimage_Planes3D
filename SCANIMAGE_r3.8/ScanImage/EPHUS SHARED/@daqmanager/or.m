%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  '|' operator
%%
%%  Returns true if both objects exist and are non-zero.
%%
%%  Created - Tim O'Connor 11/6/03
%%
%%  Changed:
%%
%%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2003
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function bool = and(dm1, dm2)

if strcmpi(class(dm1), 'daqmanager')
    dm1 = 1;
end
    
if strcmpi(class(dm2), 'daqmanager')
    dm2 = 1;
end

bool = dm1 | dm2;

return;