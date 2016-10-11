function [handles,tags]=findWithTag(parent,string,varargin)
% FINDWITHTAG   - Returns handles with Tag matching input string (or part threreof).
%   FINDWITHTAG searches through the tags for all the objects on the parent
%   to try to find a string match.
%
%   varargin can be used to pass parameters to findobj.
%
%   Ex:  findWithTag(gcf,'mean','type','line')
%       searches and returns all possible occurrences of the word 'mean' in the
%       tag of any line object on the current figure.
%
% See also FINDWITHNAME, FINDFILEWITHNAME

handles=[];
if ~ishandle(parent)
	return
end

htemp=findobj(parent,varargin{:});
if isempty(htemp)
	return
else
	inarray=zeros(1,length(htemp));
	tags=get(htemp,'Tag');
	if ~iscell(tags)
		tags={tags};
	end
	for counter=1:length(htemp)
		inarray(counter)=~isempty(findstr(tags{counter},string));
	end
	handles=htemp(logical(inarray));
	tags=tags(logical(inarray));
end


