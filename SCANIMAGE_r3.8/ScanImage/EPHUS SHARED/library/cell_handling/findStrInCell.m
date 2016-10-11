function [varargout] = findStrInCell(cellstr,str)
% FINDSTRINCELL   - locates indices of string in cell array of strings.
% 	FINDSTRINCELL(cellstr, str) will find the location of the str in the 
% 	cell array of string cellstr.  If it is not there, it will output an [].
% 	
% 	See also FINDSTRINCELLI

r=[];
c=[];
if iscellstr(cellstr)
	if ischar(str)
        if nargout >= 1
            [varargout{1:nargout}]=ind2sub(size(cellstr),find(strcmp(cellstr,str)));
        else
            varargout{1}=find(strcmp(cellstr,str));
        end
	else
		error('findStrInCell: 2nd input must be a character array');
	end
else
	error('findStrInCell: 1st input must be a cell array of strings');
end
