% getNumericSuffix - Retrieve the numeric suffix from a string.
%
% SYNTAX
%  suffix = getNumericSuffix(str)
%   str - A string that may or may not end with a set of numbers.
%   suffix - The string of numbers at the end of str, if no numbers exist, empty is returned.
%
% USAGE
%
% NOTES
%  See TO100305A.
%
% CHANGES
%
% Created 10/3/05 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function suffix = getNumericSuffix(str)

suffix = [];
for i = length(str): -1 : 1
    num = str2num(str(i : end));
    if ~isempty(num)
        suffix = num;
    else
        return;
    end
end