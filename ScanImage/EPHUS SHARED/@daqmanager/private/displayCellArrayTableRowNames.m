%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  Print key value pairs from cell array.
%%
%%  displayCellArrayTable(arr)
%%
%%  Created - Tim O'Connor 11/11/03
%%
%%  Changed:
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function displayCellArrayTableRowNames(arr)

if mod(length(arr), 2) ~= 0
    error('Invalid cell array size, must have an even number of elements.');
end

[x y] = size(arr);
for i = 1 : x - 1
    fprintf(1, '%s, ', arr{i});
end

fprintf(1, '%s\n', arr{i});

return;