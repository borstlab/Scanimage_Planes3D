function out=padLength(array,len,value,direction)
% PADLENGTH   - Pads numeric array with specified value.
%  PADLENGTH(array,len,value,direction) will take the input array and make 
% 	it of length len.  If the array is longer than len, it will be 
% 	truncated starting from the beginning out = str(1:len).
% 
% 	If the array is shorter then len, it will be padded with value either
% 	before or after the array (designated by direction as 'pre' or 'post'
% 	with 'post' as default).
% 
% 	See also PADSTRING, ISEQUAL

% 	Changes:
% 		TPMOD1 (2/4/04) - Changed function and commented it like PADSTRING.

if nargin <2
    error('padLength: Requires at least 2 inputs');
elseif nargin == 2
    direction='post';
    value=0;
elseif nargin == 3
    direction='post';
end
out=array;
if length(array)>=len
	out=out(1:len);
    return
end

pad=repmat(value,1,len-length(array));
pad=pad(1:len-length(array));
if strcmp(direction,'pre')
    out=[pad out];
elseif strcmp(direction,'post')
    out=[out pad];
end
