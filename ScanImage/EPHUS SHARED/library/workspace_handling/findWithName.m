function [varnames]=findWithName(string,classname,globalflag)
% FINDWITHNAME   - Finds and returns variables containing specified string.
%   FINDWITHNAME searches through the names of all the variables in the
%   workspace.  Can only look for global variables if globalflag is set to 1.
%
% See also FINDWITHTAG, FINDFILEWITHNAME, GETVARNAMESFROMCLASS

varnames={};
if nargin < 2
	classname='';
	globalflag=0;
elseif  nargin < 3
	globalflag=0;
end

varnames = getVarNamesFromClass(classname,globalflag);
 
inarray=zeros(1,length(varnames));
for counter=1:length(varnames)
	inarray(counter)=~isempty(findstr(varnames{counter},string));
end

varnames=varnames(logical(inarray));

