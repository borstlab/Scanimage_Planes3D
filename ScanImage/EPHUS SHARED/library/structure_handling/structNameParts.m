function [outTopName, outStructName, outFieldName] = structNameParts(s)
% STRUCTNAMEPARTS   - Parse structure into parts based on periods.
%   STRUCTNAMEPARTS will take the input string s which is a structure name
%   with subfileds indexed using the '.' notation (ex.
%   'state.acq.numberOfFrames' and output  the outTopName,outStructName, and
%   the outFieldName, all of which are strings suitable for use with the
%   dynamic fieldnames referencing in MATLAB 6.5+.
%
%   Ex:  [outTopName, outStructName, outFieldName] = structNameParts('state.acq.numberOfFrames')
%
%   outTopName = 'state'
%
%   outStructName = 'state.acq'
% 
%   outFieldName = 'numberOfFrames'
%
%   See also FIELDNAMES, ORDERFIELDS, PARSESTRUCTSTRING

% Changes:
% 	TPMOD1 (2/4/04) - Commented.

s=deblank(s);
periods=findstr(s, '.');
if length(periods)>0
	outTopName=s(1:periods(1)-1);
	outStructName=s(1:periods(length(periods))-1);
	outFieldName=s(periods(length(periods))+1:length(s));
else
	outTopName=s;
	outStructName=[];
	outFieldName=[];
end