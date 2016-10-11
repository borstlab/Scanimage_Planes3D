% SYNTAX
%  interleaved = interleaveArrays(arr1, arr2)
%   arr1 - An array.
%   arr2 - An array of the same type and length as arr1.
%   interleaved - An array of the type and twice the length of arr1, where the even indexed values
%                 come from arr1 and the odd indexed values come from arr2.
%
% USAGE
%  This is useful for combining separate name and value lists into a single ordered name-value pair list.
%  See @daqmanager/startChannel/setProperties.
%
% NOTES
%  This will work with cell arrays as well as numeric arrays, provided both arrays of the same type.
%  Numeric types may be automatically upcast, as per Matlab conventions.
%
% CHANGES
%
% Created 1/13/06 - Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function interleaved = interleaveArrays(arr1, arr2)

if length(arr1) ~= length(arr2)
    error('To interleave arrays they must both have the same length.');
end

c1 = lower(class(arr1));
c2 = lower(class(arr2));

if ~(strcmp(c1, 'cell') & strcmp(c2, 'cell')) | (isnumeric(arr1) & isnumeric(arr2))
    error('To interleave arrays they must both have the same type.');
end

if isnumeric(arr1)
    interleaved = zeros(1, length(arr1) + length(arr2), 1);
%     interleaved(1:2:end-1) = arr1;
%     interleaved(2:2:end) = arr2;
else
    interleaved = cell(1, length(arr1) + length(arr2));
%     interleaved(1:2:end-1) = arr1;
%     interleaved(2:2:end) = arr2;
end

interleaved(1:2:end-1) = arr1;
interleaved(2:2:end) = arr2;

return;