function [varnames] = getVarNamesFromClass(ref_class,globalflag)
%GETVARNAMESFROMCLASS   - Locates workspace variables based on class.
%   GETVARNAMESFROMCLASS will output the string names of any workspace
%   variables from a certain class referred by ref_class.  If globalflag is
%   set to 1, the workspace is only searched for globals of that class.
%
%   See also FINDWITHNAME

if nargin < 2
	globalflag=0;
end

if globalflag
	htemp=evalin('base','whos(''global'')');
else
	htemp=evalin('base','whos');
end
% Assign the variable names to a cell array
[varnames{1:length(htemp)}] = deal(htemp.name);

% if sorting by class, filter by that class.
if ~isempty(ref_class)
    [classnames{1:length(htemp)}] = deal(htemp.class);
    varnames=varnames(strcmpi(ref_class,classnames));
end
