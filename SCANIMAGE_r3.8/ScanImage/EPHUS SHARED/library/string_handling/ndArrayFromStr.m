function a = ndArrayFromStr(s)
% NDARRAY2STR - Converts an n-dimensional array into a single line string encoding.
%
% Created: Timothy O'Connor 2/25/04
% Copyright: Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2004
% 
% SYNTAX
%     a = ndArrayFromStr(s)
%     
% ARGUMENTS
%     s - An string encoded by `ndArray2String`.
%
% RETURNS
%     a - The matrix decoded from s.
%     
% DESCRIPTION
%  The encoding algorithm works as follows:
%     b = size(a);
%     s = strcat(mat2str(b), '&', mat2str(reshape(a, [1 prod(b)]), 17))
%
%  The encoded string appears as two Matlab style array specifications,
%  such that calling `eval` on them will result in arrays. The two
%  arrays are separated by an ampersand ('&').
%
%  The string may be decoded in the following way: 
%     [bs s] = strtok(s, '&');
%     as = strtok(s, '&');
%     a = reshape(str2num(as), str2num(bs));
%
%  If a string is encountered, which contains only 1 numeric array, it
%  will be treated as the data array. That is to say, the size is optional
%  for 1 and 2 dimensional arrays.
%
% EXAMPLES
%     a = zeros(2, 2, 2);
%     a(1, :, :) = magic(2);
%     a(2, :, :) = magic(2);
%     s = ndArray2Str(a)
%     s =
%         [2 2 2]&[1 1 4 4 3 3 2 2]
%
%     b = ndArrayFromStr(s);
%     isequal(a, b)
%     ans =
%           1
%           
% See Also NDARRAY2STR, RESHAPE, MAT2STR, STR2NUM
a = [];

%Input checking...
if ~strcmpi(class(s), 'char')
    error('MATLAB:badopt', 'Input to ndArrayFromStr must be a string.');
end

if isempty(s)
    return;
end

%Parse out the size.
[bs s] = strtok(s, '&');

%Parse out the data.
as = strtok(s, '&');

if isempty(as)
    %This is a 1 or 2 dimensional array.
    a = str2num(bs);
    return;
end

%Reconstruct the array.
a = reshape(str2num(as), str2num(bs));