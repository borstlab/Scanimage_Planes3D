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

if size(arr, 2) ~= 2
    error('Invalid cell array size, must have 2 of columns.');
end

fprintf(1, 'Displaying cell array table of size [%s]:\n\n', num2str(size(arr)));

[x y] = size(arr);
for i = 1 : x
    name = arr{i, 1};
    if strcmpi(class(name), 'cell')
        name = name{1};
    end
    fprintf('Row %s - ', num2str(i));

    if isempty(arr{i, 2})
        fprintf(1, '%s: EMPTY\n', name);
    elseif strcmpi(class(arr{i, 2}), 'cell')
        celldisp(arr{i, 2}, name);
    elseif isnumeric(arr{i, 2})
        fprintf(1, '%s: %s\n', name, num2str(arr{i, 2}));
    elseif strcmpi(class(arr{i, 2}), 'char')
        fprintf(1, '%s: %s\n', name, arr{i, 2});
    else
        fprintf(1, '%s: %s of size [%s]\n', name, class(arr{i, 2}), num2str(size(arr{i, 2})));
    end
end

fprintf(1, '\n');

return;