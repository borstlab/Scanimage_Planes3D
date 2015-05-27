function argsStr = parseArgumentCellArray(argsCell)
%PARSEARGUMENTCELLARRAY Parses vectorial cell array contents reprenting an argument list into a vectorial (one-line) string
%% SYNTAX
%   argsStr = parseArgumentCellArray(argsCell)
%       argsCell: a vectorial cell array representing arguments to a fucntion
%       argsStr: a string representing the contents of argsCell as a one-line string
%% NOTES
%   Cell array contents can contain multi-line cell arrays, strings, or numeric arrays. These can be represented using one line by using the ';' character.
%   This function inserts ';' characters where multi-row arrays (cell, string, or numeric) are encountered.
%% CREDITS 
%   Created 6/9/08 by Vijay Iyer
%   Janelia Farm Research Campus/Howard Hughes Medical Institute
%% CHANGES
%   VI061808 -- Handle empty cell array case -- Vijay Iyer 6/18/08
%% *****************************************

if ~iscell(argsCell) || (~isempty(argsCell) && ~isvector(argsCell)) %VI061808A
    error('Function presently only works with vectorial cell arrays');
end

argsStr = '{';
for i=1:length(argsCell)
    if iscell(argsCell{i})
        newStr =  parseArgumentCellArray(argsCell{i});
    elseif isnumeric(argsCell{i})
        newStr = '[';
        for j=1:size(argsCell{i},1)
            newStr = [newStr num2str(argsCell{i}(j,:))];
            if j ~= size(argsCell{i},1) %this is not the last row
                newStr = [newStr ';'];
            end
        end
        newStr = [newStr ']'];
    elseif ischar(argsCell{i})
        if ~isvector(argsCell{i})
            error('This cell array parser presently only accepts vectorial strings as elements of the cell array');
        else
            newStr = ['''' argsCell{i} ''''];
        end
    elseif isa(argsCell{i},'function_handle')
        newStr = func2str(argsCell{i});
    else
        error(['Element # ' num2str(i) ' of the input cell array was not of a recognized type']);
    end


    %Append new string to the array string
    if i==1
        argsStr = [argsStr newStr]; %#ok<AGROW>
    else
        argsStr = [argsStr ' ' newStr];
    end
    
end

%Terminate string
argsStr = [argsStr '}'];
            
        
        
