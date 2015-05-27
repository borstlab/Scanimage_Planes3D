function value=getParamFromName(name,param,direction)
% GETPARAMFROMNAME   - Parses string input based on '_' delimiter and returns value.
%   GETPARAMFROMNAME Parses name to reveal parameter.
%   Name must have the following form:  '_' + value + param
%
%   Example: getParamFromName('_10epoch_1channel','ep') would return numeric 10.
% 
%   See also

if nargin < 3
	direction=1;
end

value=0;
spacers=[strfind(name,'_') length(name)+1];
index=strfind(name,param);

if isempty(index)
	beep;
	disp(['getParamFromName: Parameter ' param ' not found']);
	return
elseif length(index)>1
	index=index(end);
end

bumperleft=max(spacers(find(spacers <= index)));
bumperright=min(spacers(find(spacers >= index)));

if direction==1
	value=str2num(name(bumperleft+1:index-1));
else
	value=str2num(name(index+length(param):bumperright-1));
end
if isempty(value)
	value=0;
end
