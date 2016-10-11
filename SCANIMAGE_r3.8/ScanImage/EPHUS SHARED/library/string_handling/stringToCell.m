function out=stringToCell(inString)
% STRINGTOCELL   - Removes quotes from inString and outputs as cell array.
% 	STRINGTOCELL(inString) will take the input string inString and format it by
% 	stripping away extra '' marks, and output the result as a cell array.
%
% 	See also TOKENIZE

% Changes:
% 	TPMOD1 (2/4/04) - Changed function and commented it.

out=[];
if ~ischar(inString)
	return
end
if ~strcmp(inString(1),'{')
	if all(strcmp('''',{inString(1),inString(end)}))				
		out={inString(2:end-1)};
	end
	return
end
quotes=findstr(inString, '''');
if length(quotes)>2
	for counter=1:2:length(quotes)
		out=[out, {inString(quotes(counter)+1:quotes(counter+1)-1)}];
	end
end




