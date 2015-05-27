function out = addForPmtOffsets(input, binfactor)
global state gh

% This function takes a 2D input and adds along rows to produce 
% another 2D array shortened in rows to rows/binfactor;
%
% Written By: Thomas Pologruto
% Cold Spring Harbor Labs
% February 9, 2001.

rows = size(input,1);
columns = size(input,2);

if binfactor == 1
	out = input;
else
	out = reshape(input, binfactor, (rows/binfactor), columns);
	out = sum(out);
	out = reshape(out, (rows/binfactor), columns);
end

