function string = var2Str(name)
% VAR2STR - Converts any variable into a human readable/editable string, which may be
%           reparsed later.
%
% Created: Timothy O'Connor 2/26/04
% Copyright: Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2004
% 
% USAGE
%     string = var2Str(variable)
%     
% ARGUMENTS
%     name - The name of the variable.
%
% RETURNS
%     string - A user readable/editable, computer parsable string, with a Matlab-like
%              syntax.
%     
% DESCRIPTION
%  
% SYNTAX
%     [class] [size] <name>[index] = <value>;
%
%     class - A string, containing no spaces, which would be the result of calling `class my_var`.
%             This field is optional, but recommended. If it is not specified, the type is implied by the
%             associated value literal.
%
%     size - A scalar number, or an array (delinated with '[' and ']'), specifying the size of the variable.
%            This field is optional, but recommended. If it is not specified, the size is inferred by the
%            name and value literal.
%
%     name - A string, containing no spaces, which is the name of the variable.
%
%     index - A number, enclosed by '(' and ')', which declares which element in the variable named <name> is 
%             being set. This is only used in the case of multidimensional structures, and is considered an 
%             error otherwise. The index would be used to imply the size, if it was not specified.
%
%     value - A literal representation of the variable's value.
%
%     The trailing ';' character is not strictly necessary in many cases.
%     Assignments that span multiple lines must have all intermediary lines terminated by an ellipsis ('...').
%
% LITERALS
%     numbers - Scalars may be written directly. 1 or 2 dimensional arrays may be specified in the same format
%               as accepted on the Matlab command-line. More than 2 dimensional arrays are specified in the
%               format described in ndArray2Str.
%               The following types (classes) are all considered to be numbers: logical, double, single, int8, uint8, 
%               int16, uint16, int32, uint32, int64, uint64.
%
%     strings - Any group of one or more characters, bounded by single quotes ('). To use a '-character inside
%               a string, it must be escaped by a quote (the same as in the Matlab string-literal syntax).
%
%     cell - Any set of comma separated variable declarations, encased in a '{' and '}'. Anonymous structures, nested
%            inside a cell array, must start with a '.' if their type is implied. The fields of an anonymous structure
%            must be enclosed with in '(' and ')', to declare that those fields all belong to a single struct.
%
%     struct - Each field of a struct is considered a separate variable, each entry contains the fully qualified
%              name of the variable (leaf/node) on the tree. There is currently no way of neatly specifying the shape
%              of an n-dimensional structure as the root variable. Therefore, only 1 dimensional structures are supported.
%              If an n-dimensional structure is encoded, it will get reshaped to 1 dimension automatically.
%
%     object - The classname, enclosed by '<' and '>', followed by the object's internal structure wrapped in a cell array.
%              It is recommended that all objects implement their own string encodings, even if they rely heavily
%              on this function (and it's sister). Recovery from a string is not possible, without at least a helper
%              method and a class instance. varFromStr does not directly support object recovery. The objects will
%              need to implement the fromString method to recover themselves. It is advised that they implement
%              their own literal encoding, according to this syntax, by implementing the toString method.
%
%     function handle - The string representing the function name, preceeded by an '@'. This is the same as the way
%                       that function handles are expressed on the command line.
%
% EXAMPLES
%     myVar(1).a = [1 2 3 4; 5 6 7 8];
%     myVar(1).b = 2;
%     myVar(1).c = 'a';
%     myVar(1).d = {'asdf', 1, myVar, {'1', '2'}};
%     myVar(2).a = [2 2 2 2; 2 2 2 2];
%
%     var2Str('myVar')
%     ans =
%         double [2 4] myVar(1).a = [2 4]&[1 5 2 6 3 7 4 8];
%         double [2 4] myVar(2).a = [2 4]&[2 2 2 2 2 2 2 2];
%         double [1 1] myVar(1).b = 2;
%         double [0 0] myVar(2).b = [];
%         char [1 1] myVar(1).c = 'a';
%         double [0 0] myVar(2).c = [];
%         cell [1 5] myVar(1).d = {char [1 4] 'asdf', double [1 1] 1, struct [1 1] (struct [1 1] .a = [2 4]&[1 5 2 6 3 7 4 8]; struct [1 1] .b = 2; struct [1 1] .c = 'a'; ), cell [1 2] {'1', '2'}, function_handle [1 1]  @var2Str};
%         double [0 0] myVar(2).d = [];
%
% NOTES
%     Trying to bulk encode multiple instances of a single field in a multidimensional struct will not work. Using the example shown above, the following
%     commands will not work:
%        var2Str('myVar.d')
%        var2Str('myVar(:).d')
%
%     Instead, use the root node ('myVar' above), or index into a specific element in the array ('myVar(2).d' above).
%
% See Also VARFROMSTR, NDARRAY2STR
if isempty(name)
    error('No variable name specified.');
end

if ~strcmpi(class(name), 'char')
    error(sprintf('Variable name must be a string: %s', class(name)));
end

ex = evalin('caller', strcat('exist(''', name, ''') | structFieldExists(''', name, ''')'));
if ~ex
    error(sprintf('Failed to find and string-encode variable ''%s''', name));
end

%Grab the data to be encoded.
val = evalin('caller', name);

classname = class(val);

%Start building the string.
string = sprintf('%s %s %s = ', classname, mat2str(size(val)), name);

if isnumeric(val)
    %Take advantage of the ndArray2Str and ndArrayFromStr functions.
    string = sprintf('%s%s', string, ndArray2Str(val));
elseif strcmpi(classname, 'char')
    %Self-explanatory.
    string = sprintf('%s''%s''', string, val);
elseif strcmpi(classname, 'struct')
    %Clear this, since recursion will handle it.
    string = '';
    
    %This ends up just being a series of variables, with long names.
    fnames = fieldnames(val);    
    
    for i = 1 : length(fnames)
       
        %It might be nice to come up with a way that uses less copying but this is otherwise fairly elegant.
        for j = 1 : prod(size(val))        
            %This is sort of painful, but it's necessary to snag all elements
            %in a structure-array of arbitrary dimension.
            index = ind2sub(size(val), j);

%             for k = 1 : length(index) - 1
%                 %Kind of messy, but we want to imply the shape of a multidimensional structure.
%                 indexString = strcat(indexString, num2str(index(k)), ', ');
%             end
%             indexString = strcat(indexString, num2str(index(end)));
            
            newVarName = sprintf('%s(%s).%s', name, num2str(j), fnames{i});
            eval(sprintf('%s = val(%s).%s;', newVarName, num2str(j), fnames{i}));

            %Recurse over all fields (and subfields).
            str = var2Str(newVarName);
            
            %Append, but don't use strcat, since that'll strip out whitespace.
            %Carriage returns and semicolons will be handled in the recursion.
            string = sprintf('%s%s', string, str);
        end
    end

    %None of the clean-up is needed here.
    return;
elseif strcmpi(classname, 'cell')
    if size(val) > 0    
        val = reshape(val, [prod(size(val)) 1]);

        %This can get tricky, because of nesting.
        string = sprintf('%s{', string);
        for i = 1 : length(val) - 1
            string = sprintf('%s%s %s %s, ', string, class(val{i}), mat2str(size(val{i})), encodeLiteralInCell(val{i}));
        end
        string = sprintf('%s%s %s %s};\n', string, class(val{end}), mat2str(size(val{end})), encodeLiteralInCell(val{end}));
    else
        string = sprintf('%s{};', string);
    end
    
    return;
elseif strcmpi(classname, 'function_handle')
    %Prefix an '@' and let it go on it's merry way.
    string = sprintf('%s@%s', string, func2str(val));
else
    if hasMethod(classname, 'toString')
        string = sprintf('%s%s', string, toString(val));
    elseif hasMethod(classname, 'fromString')
        %Cast it to a struct and print it out, let the object
        %recover itself on the other side.
        objStruct = struct(val);
        string = sprintf('%s<%s>{%s}', string, classname, var2Str('objStruct'));
    else
        string = sprintf('%sUNSUPPORTED_TYPE', string);
        warning(sprintf('Can not encode variable ''%s'' into a string, it''s type (%s) is not supported.', name, classname));
    end
end

%Terminate with a ';'.
if string(end) ~= ';'
    string = sprintf('%s;\n', string);
end

return;

%-----------------------------------------------------------------------------
function string = encodeLiteralInCell(val)

classname = class(val);
string = '';

if isnumeric(val)
    %Take advantage of the ndArray2Str and ndArrayFromStr functions.
    string = sprintf('%s', ndArray2Str(val));
elseif strcmpi(classname, 'char')
    %Self-explanatory.
    string = sprintf('''%s''', val);
elseif strcmpi(classname, 'struct')
    %A struct will never be passed in from the parent, hence it's safe
    %to recurse anonymous structs here.
    fnames = fieldnames(val);

    string = '(';
    for i = 1 : length(fnames) - 1
        for j = 1 : prod(size(val))
            fname = sprintf('val(%s).%s', num2str(j), fnames{i});
            sizeString = mat2str(eval(sprintf('size(%s)', fname)));
            classString = eval(sprintf('class(%s)', fname));
            if strcmpi(eval(sprintf('class(%s)', fname)), 'struct')
                sprintf('%s%s %s %s', string, classString, sizeString, eval(sprintf('encodeLiteralInCell(%s)', fname)));
            else
                %Non-structs can just get encoded.
                string = sprintf('%s%s %s .%s = %s ', string, classString, sizeString, fnames{i}, eval(sprintf('encodeLiteralInCell(%s)', fname)));
            end
        end
    end
    fname = sprintf('val(%s).%s', num2str(prod(size(val))), fnames{end});
    sizeString = mat2str(eval(sprintf('size(%s)', fname)));
    classString = eval(sprintf('class(%s)', fname));
    if strcmpi(eval(sprintf('class(%s)', fname)), 'struct')
        sprintf('%s%s %s %s', string, classString, sizeString, eval(sprintf('encodeLiteralInCell(%s)', fname)));
    elseif strcmpi(eval(sprintf('class(%s)', fname)), 'cell')
    else
        %Non-structs can just get encoded.
        string = sprintf('%s%s %s .%s = %s ', string, classString, sizeString, fnames{i}, eval(sprintf('encodeLiteralInCell(%s)', fname)));
    end
    
    string = sprintf('%s)', string);
elseif strcmpi(classname, 'cell')
    if size(val) > 0    
        val = reshape(val, [prod(size(val)) 1]);
        
        %This can get tricky, because of nesting.
        string = sprintf('%s{', string);
        for i = 1 : length(val) - 1
            string = sprintf('%s%s %s %s, ', string, class(val{i}), mat2str(size(val{i})), encodeLiteralInCell(val{i}));
        end
        string = sprintf('%s%s %s  %s}', string, class(val{end}), mat2str(size(val{end})), encodeLiteralInCell(val{end}));
    else
        string = sprintf('%s{}', string);
    end
elseif strcmpi(classname, 'function_handle')
    %Prefix an '@' and let it go on it's merry way.
    string = sprintf('@%s', func2str(val));
else
    if hasMethod(classname, 'toString')
        string = sprintf('%s%s', string, toString(val));
    elseif hasMethod(classname, 'fromString')
        %Cast it to a struct and print it out, let the object
        %recover itself on the other side.
        objStruct = struct(val);
        string = sprintf('<%s>{%s}', classname, var2Str('objStruct'));
    else
        string = sprintf('%sUNSUPPORTED_TYPE', string);
        warning(sprintf('Can not encode variable ''%s'' into a string, it''s type (%s) is not supported.', name, classname));
    end
end

return;
