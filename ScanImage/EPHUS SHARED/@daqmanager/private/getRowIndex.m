%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  Find the row in the cell array corresponding to the given key.
%%
%%  row = getRowIndex(arr, 'key')
%%
%%  Created - Tim O'Connor 11/11/03
%%
%%  Changed:
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function row = getRowIndex(arr, key)

[x y] = size(arr);

row = -1;

for i = 1 : x
    if strcmp(arr{i, 1}, key)
        row = i;
        return;
    end
end

return;