function [output,handles]=getDataFromHandle(handle,varargin)
% GETDATAFROMHANDLE   - Formats YData from handles on axes as array.
%   GETDATAFROMHANDLE Collects all the data in graphics objects from the parent handle, and returns it as a cell
%   array or matrix.  You can specify the type of data to get (YData, XData, CData, etc...), but by default it only
%   will query objects of type 'line'.
% 
%   Ex: getDataFromAxes(ax,['return_as_matrix', BOOL],['type', STRING],['dataToGetFromObject', STRING])
%
%   See also 

if ~ishandle(handle)
	error('getDataFromAxes: first input must be an axes or figure');
end

% Initialize parameters and ouput
output=[];
return_as_matrix=1;
dataToGetFromObject='';
type='line';

% Parse the variable inputs
for inputCounter=1:2:length(varargin)
	value=varargin{inputCounter+1};
	if strcmpi(varargin{inputCounter},'return_as_matrix')
		return_as_matrix=value;
	elseif strcmpi(varargin{inputCounter},'type')
		type=value;
	elseif strcmpi(varargin{inputCounter},'dataToGetFromObject')
		dataToGetFromObject=value;
	end
end

% Determine the property to get for the object type selected.
if isempty(dataToGetFromObject)
	if strcmpi(type,'Image')
		dataToGetFromObject='CData';
	elseif strcmpi(type,'Line')
		dataToGetFromObject='YData';
	else
		error('getDataFromAxes: type of object must be a line or an image');
	end
end

% Get handles to all the objects.
handles=findobj(handle,'type',type);

% Get all the properties of all the handles selected, and convert to a
% matrix if necessary.
if isempty(handles)
	return
elseif return_as_matrix
	output=cell2mat(get(handles,dataToGetFromObject));
else
	output=get(handles,dataToGetFromObject);
end
		

