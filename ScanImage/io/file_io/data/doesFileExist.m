function out = doesFileExist(fullfilename)
% DOESFILEEXIST   - checks if file exists.
% 	DOESFILEEXIST(filename) checks if the file or directory specified exists.  
% 	Returns 1 if it does, and 0 if not.
% 	
% 	See also DIR

d = dir(fullfilename);
if isempty(d)
	out=0;
else
	out=1;
end
