function boolean = hasMethod(obj, method)
% HASMETHOD - Determines if a specified object/class implements the requested method.
%
% Created: Timothy O'Connor 3/5/04
% Copyright: Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2004
% 
% SYNTAX
%     boolean = hasMethod(obj, method)
%     
% ARGUMENTS
%     obj - An instance of an object or a string representing the object's
%                       class.
%     method - A string, the name of the method in question.
%
% RETURNS
%     boolean - True, if and only if, the object/class has a method with the same name.
%               Does not evaluate method signatures or behavior.
%
% See Also METHODS, ISMETHOD, ISMEMBER
warning('DEPRECATED - use ''ismethod''');
boolean = logical(0);

if isempty(obj) | isnumeric(obj)
    return;
end
cl = class(obj);
if strcmpi(cl, 'struct') | strcmpi(cl, 'function_handle')
    return;
end

if ismember(method, methods(obj))
    boolean = logical(1);
end

return;