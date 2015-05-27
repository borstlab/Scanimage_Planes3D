function varargout = varFromStr(source, varargin)
% VAR2STR - Converts any variable into a human readable/editable string, which may
%           reparsed later.
%
% Created: Timothy O'Connor 2/26/04
% Copyright: Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2004
% 
% USAGE
%     varFromStr(string)
%     varFromStr(handle)
%     varFromStr(string, propertyname, propertyvalue, ...)
%     variableStruct = varFromStr(source)
%     
% ARGUMENTS
%     source - The string to be parsed or a handle to a text file to be parsed.
%     propertyname - The name associated with a key-value pair.
%     propertyvalue - The value associated with a key-value pair.
%     
%     PropteryNames and PropertyValues MUST be specified in pairs. Any number of pairs may be specified, 
%     in any order, given that each PropteryName immediately preceeds a PropertyValue.
%
% RETURNS
%     Nothing. Parsed variables are placed into 'caller' space.
%     Warning: This may clobber variables, to avoid that, set the 'NoClobber' property
%              to 'On'.
%
%     variableStruct - All variables recovered are set as fields in this structure.
%     
% DESCRIPTION
%
% NOTE: Line breaks, comments, and source types
%     The use of carriage returns, line feeds, and form feeds will get parsed differently depending on the
%     type of source specified. In a file, each line may have a comment. In a string, all characters from
%     the comment character ('%') to the end of the string are ignored.
%
% NOTE: Ellipsis
%     In the case of a string as the source, the ellipsis is not supported.
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
%     myStr = strcat('double [2 4] myVar(1).a = [2 4]&[1 5 2 6 3 7 4 8];', ...
%         'double [2 4] myVar(2).a = [2 4]&[2 2 2 2 2 2 2 2];', ...
%         'double [1 1] myVar(1).b = 2;', ...
%         'double [0 0] myVar(2).b = [];', ...
%         'char [1 1] myVar(1).c = 'a';', ...
%         'double [0 0] myVar(2).c = [];', ...
%         'cell [1 5] myVar(1).d = {char [1 4] 'asdf', double [1 1] 1, struct [1 1] (struct [1 1] .a = [2 4]&[1 5 2 6 3 7 4 8]; struct [1 1] .b = 2; struct [1 1] .c = 'a'; ), cell [1 2] {'1', '2'}, function_handle [1 1]  @var2Str};', ...
%         'double [0 0] myVar(2).d = [];');
%     varFromStr(myStr);
%
%     myVar
%     ans = 
%          1x2 struct with fields:
%              a
%              b
%              c
%              d
%
%     myVar(1)
%     ans =
%          a: [2x4 double]
%          b: 2
%          c: 'a'
%          d: {'asdf' [1] [1x1 struct] {1x2 cell}}
%
%     myVar(2)
%     ans =
%          a: [2x4 double]
%          b: []
%          c: []
%          d: []
%
% See Also VAR2STR, NDARRAY2STR, GETNEXTWORD
noClobber = 0;

%Check number of arguments.
if mod(nargin - 1, 2) ~= 0
    error('MATLAB:badopt', 'All key-value pairs must consist of exactly 2 arguments.');
end

if nargout > 1
    error('Too many output arguments to varFromStr: %s (must be 0 or 1)', num2str(nargout));
end

for i = 1 : 2 : nargin - 1
    if strcmpi(varargin{i}, 'NOCLOBBER')
        noClobber = varargin{i + 1};
        
        if ~isnumeric(noClobber)
            error('MATLAB:badopt', sprintf('Invalid NOCLOBBER value. Must be numeric: %s', class(noClobber)));
        end
        
        if length(noClobber) > 1
            error('MATLAB:badopt', sprintf('Invalid NOCLOBBER value. Must be of length 1.'));
        end
    end
end

if strcmpi(class(source), 'char')
    %Let the exceptions fly.
    variables = recoverVariables(source);
elseif isFileHandle(source)
    lineNumber = 1;
    line = fgets(source);
    
    %Keep reading until the EOF.
    while line ~= -1
        %Trailing whitespace will screw up ellipsis detection.
        line = deblank(line);
        
        %Secondary loop, to pick up ellipsis continuations.
        while line ~= -1 & length(line) >= 3
            if ~strcmp(line(end - 2 : end), '...')
                break;
            end
            
            line = fgets(source);
            lineNumber = lineNumber + 1;
            
            %Append.
            if line ~= -1
                %Replace the ellipsis with whitespace, make it into a single line.
                line = [line(1 : end - 3) ' ' line];
            end
        end
        
        try
            nextVar = recoverVariables(line);
            
            %Put it on the returnable struct.
            names = fieldnames(nextVar);
            for i = 1 : length(names)
                variables.(names{i}) =  nextVar.(names{i});
            end
        catch
            %Trap this one and try to give a hint about where the error is.
            fprintf(2, 'Failed to parse variable - FileID: %s  Line: %s Error: %s\n', num2str(fid), num2str(lineNumber), lasterr);
        end
        
        line = fgets(source);    
        lineNumber = lineNumber + 1;
    end
end

if nargout > 0
    %Send back a struct.
    varargout{1} = variables;
end

%Nothing left to do.
if isempty(variables)
    return;
end

if nargout == 0
    %Put the variables into caller space.
    names = fieldnames(variables);
    for i = 1 : length(names)
        loadVar = logical(1);
        
        if noClobber
            if evalin('caller', sprintf('exist(''%s'') == 1', names{i}))
                %Don't clobber it.
                warning('Can not load variable ''%s'', because it already exists. Disable the ''NOCLOBBER'' property to change this behavior.', names{i});
                loadVar = logical(0);
            end
        end
        
        if loadVar
            evalin('caller', sprintf('%s = variables.%s;', names{i}, names{i}));
        end
    end
end

return;

%--------------------------------------------------
function variables = recoverVariables(string)

variables = [];

declaredClass = 0;
declaredShape = 0;
declaredIndex = 0;

%Default assumptions, these should (almost) always be overridden
%by contextualized choices.
class = 'double';
impliedClass = '';
impliedSize = '';
shape = [1 1];
index = 1;

% %Find line feeds (LF, \n), carriage returns (CR, \r), and form feeds (FF, \f).
% lineBreaks = find(string == sprintf('\n') | string == sprintf('\r') | string == sprintf('\f'));
% lastBreak = 1;

%Buffer the words here, until their meaning is determined.
%When parsing a formal specification the cells would contain type, size, name, and value (respectively).
declarations = cell(4, 1);
declarations{1} = '';
declarations{2} = '';
declarations{3} = '';
declarations{4} = '';
dCount = 1;

%Set up the parsing indices.
[word remainderIndex commentIndex] = getNextWord(string);
ri = remainderIndex;

while remainderIndex > 0
% fprintf(1, 'outer: ''%s''\n', word);
    %Sort things out into class, 
    if strcmpi(word, '=')
        dCount = 1;
        while remainderIndex > 0

            [word remainderIndex] = getNextWord(string(ri : commentIndex));
            ri = ri + remainderIndex - 1;
% fprintf(1, 'inner: %s\n', word);
            if strcmpi(word, ';')
                break;
            end

            declarations{4} = [declarations{4} word];
        end
    elseif dCount < 3
        %Collect the class and size.
        declarations{dCount} = word;
        dCount = dCount + 1;
    else
        %Keep appending to the variable's name.
        declarations{3} = [declarations{3} word];
    end

    if ~isempty(declarations{4})
% declarations{1}
        declarations{1} = '';
        declarations{2} = '';
        declarations{3} = '';
        declarations{4} = '';
    end

    [word remainderIndex commentIndex] = getNextWord(string(ri : end));
    ri = ri + remainderIndex - 1;
end

declarations{:}

return;