function out=translate(in,dictionary)
%TRANSLATE   - Dictionary lookup and exchange.
% 	TRANSLATE takes the input string (or cell array of strings) and a dictionary 
%   which is used as a lookup, and outputs a cell array 
% 	of the translated input.
% 	
% 	The dictionary is itself a cell array of lookup/return, where lookup is
% 	a string and return can be any valid MATLAB expression.

%   Example:    input={'GFP' 'YFP' 'CFP'};
%               dict={'GFP','gfp','YFP',[1 2],'CFP','AAAA'};
%               output=translate(input,dict);
%                 
%               output = {'gfp'    [1x2 double]    'AAAA'}
% 
%   See also 

inputchar=0;
out='';
strin='';
if ischar(in)
	in={in};
	inputchar=1;
elseif ~iscellstr(in)
	error('translate: input must be a string or cell array of strings');
end

uniquestrings=unique(in);
for unStringCounter=1:length(uniquestrings)
	dictionaryPlace=min(find(strcmpi(dictionary,uniquestrings{unStringCounter})))+1;
	if ~isempty(dictionaryPlace)
		in(strcmp(in,uniquestrings{unStringCounter}))=dictionary(dictionaryPlace);
	end
end
	
if inputchar
	out=in{1};
else
	out=in;
end
