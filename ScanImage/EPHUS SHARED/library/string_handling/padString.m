function out=padString(str,len,value,direction)
% PADSTRING   - Pads character array with specified string.
%   PADSTRING(str,len,value,direction) will take the input string str and make it 
%   of length len.  If the string is longer than len, it will be truncated starting 
%   from the beginning out = str(1:len).
%
%   If the str is shorter then len, it will be padded with value either
%   before or after the array (designated by direction as 'pre' or 'post'
%   with 'pre' as default)
% 
%   Useful for making string representations of numbers the same length for
%   indexing into automatically generated filenames.
% 
%   Ex: filename=['mybasename' padstring('2',4,0)];
%   Will produce filename = 'mybasename0002'
% 
%   See also PADLENGTH

% Changes:
% 	TPMOD1 (2/4/04) - Changed function so that it can output a smaller string 
%   if len < length(str) and commented it like PADSTRING.

if nargin <2
    error('padString: Requires at least 2 inputs');
elseif nargin == 2
    direction='pre';
    value='0';
elseif nargin == 3
    direction='pre';
end
out=str;
if length(str)>=len
	out=out(1:len);
    return
end
if isnumeric(value)
    value=num2str(value);
end
pad=repmat(value,1,len-length(str));
pad=pad(1:len-length(str));
if strcmp(direction,'pre')
    out=[pad out];
elseif strcmp(direction,'post')
    out=[out pad];
end

