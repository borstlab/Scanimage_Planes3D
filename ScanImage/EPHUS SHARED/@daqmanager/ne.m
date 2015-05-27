%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  '~=' operator
%%
%%  The exact opposite of the '==' operator.
%%
%%  Created - Tim O'Connor 11/6/03
%%
%%  Changed:
%%
%%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2003
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function bool = ne(dm1, dm2)

bool = 1;

if dm1 == dm2
    bool = 0;
end

return;