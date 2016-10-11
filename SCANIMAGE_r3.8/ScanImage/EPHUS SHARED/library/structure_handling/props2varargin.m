function out=props2varargin(str)
%PROPS2VARARGIN   - converts a structure to a param/value pair type cell array.
% 	PROPS2VARARGIN function converts the properties from a structure ( i.e. from a get(h) call)
% 	to a varargin type input for copying objects props.
% 	
% 	See also STRUCT2CELL

out={};
if ~isstruct(str) | isempty(str)
    return
end
fn=fieldnames(str);
out=reshape([fn struct2cell(str)]',1,2*length(fn));