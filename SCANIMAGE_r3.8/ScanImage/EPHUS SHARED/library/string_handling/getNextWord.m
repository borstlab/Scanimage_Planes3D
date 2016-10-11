function varargout = getNextWord(string, varargin)
% GETNEXTWORD  - Parses out the next 'word' from a string.
%
% Created: Tim O'Connor 2/20/04
% Copyright: Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2004
% 
% SYNTAX
%     word = getNextWord(string)
%     [word remainderIndex] = getNextWord(string)
%     [word remainderIndex commentIndex] = getNextWord(string)
%     [word remainderIndex commentIndex parser] = getNextWord(string)
%
%     getNextWord(string, PropertyName, PropertyValue, ...)
%     
% ARGUMENTS
%     STRING - Any valid Matlab character array.
%     PROPERTYNAME - The name associated with a key-value pair.
%     PROPERTYVALUE - The value associated with a key-value pair.
%     
%     PropteryNames and PropertyValues MUST be specified in pairs. Any number of pairs may be specified, 
%     in any order, given that each PropertyName immediately preceeds a PropertyValue.
%     
% PROPERTIES
%     DELIMITER - Any set of charactera to delimit words. Setting the delimiter to an empty string ('')
%                 causes it to accept any whitespace as a delimiter (this is the default).
%     COMMENT - Any single character to signal the start of a comment. If set to empty, no comments are 
%               allowed. The default is '%'.
%     STRING - Any single character used to flag the start/end of a string, two adjacent STRINGDELIMITERs 
%              are escaped into a single instance of the STRINGDELIMITER.
%              The default is '.
%     GROUPERS - A cell array of grouping character pairs, which are used to block up words. For
%                example: {'()', '[]', '{}'} would allow any of these pairs to result in their contents
%                forming a single word. Nested occurences are assumed to be part of the whole block-word.
%                The order of the pairs is important, as the first character starts, and the second ends
%                each block-word. The contents are returned, exclusive of the grouping characters.
%                See EXAMPLES, below, for further clarification.
%                The default is {'()', '[]', '{}'}.
%    OPERATORS - An array of characters, which when found outside of a block are considered to be full
%                words on their own, regardless of delimiters around them. Setting this to the empty
%                string means there are no operators defined. Operators get returned as words.
%                Simply put, operators act as both words and delimiters.
%                The default operators are '.', '+', '-', '*', '/', '<', '>', '?', '~', '^', ';', and '='.
%    PARSER - A precompiled set of parser options, represented by a struct.
%    RETURNPARSER - A flag determining if the parser struct should be returned. This is useful if the
%                   same set of rules will be used many times, as it can avoid recompilation on subsequent
%                   calls. The parser will be returned as the last value in the vargout cell array.
%
%    Special characters lose their meanings inside of word-blocks (STRING, GROUPERS). That is to say, the
%    comment-delimiter has no special meaning when found inside a string-delimiter or group-delimiter; the
%    string-delimiter has no special meaning when found inside the group-delimiter; the group-delimiter has 
%    no meaning when found inside the string-delimiter; the delimiter has no meaning inside the 
%    string-delimiter and group-delimiter. Operators have no meaning inside string-delimiters or 
%    group-delimiters.
%
% RETURNS
%     WORD - The next whole word in the string. A whole word is defined as:
%             A contiguous group of non-delimiter and non-comment characters.
%             A set of characters, bounded by STRINGDELIMITER.
%     REMAINDERINDEX - The index to the start of the rest of the string, following the word. This excludes 
%                      any delimiters which trail the word. Is set to 0, if the end-of-string has been reached.
%                      The REMAINDERINDEX is inclusive to the first character in the next word.
%     COMMENTINDEX - The index following the last non-delimiter character before the first comment character, inclusive.
%                    All characters beyond this point should be considered to be part of the comment. Is set to be 
%                    length(string) if there is no comment character present or comments are disabled.
%
% POSTCONDITIONS
%     Given the call `[word remainderIndex commentIndex] = getNextWord(string)` the following expressions will 
%     evaluate to be true:
%         ismember(word, string)
%         remainderIndex >= 0
%         commentIndex <= length(string) + 1
%         remainderIndex < commentIndex
%         if remainderIndex > 0
%           ~ismember(string(remainderIndex), [DELIMITER STRING GROUPERS])
%         if commentIndex < length(string)
%           string(lastWordEnd + 1 : commentIndex) == DELIMITER | COMMENT
%     
% DESCRIPTION
%     word = getNextWord(string) - Returns the first full word in the string.
% 
%     [word stringRemainder] - getNextWord(string) - Returns the first full word in the string, and 
%                              the remainder of the string.
%     
% EXAMPLE
%
%    myString = 'function varargout = getNextWord(string, varargin, ''Delimiter'', '' '') %Comment';
%
%    [word remainderIndex commentIndex] = getNextWord(myString)
%    ri = remainderIndex;
% 
%     while remainderIndex
%         [word remainderIndex] = getNextWord(myString(ri : commentIndex))
%         ri = remainderIndex + ri - 1;
%     end
%
% EXAMPLE_OUTPUT
%
%    word =
%          function
%    remainderIndex =
%          10
%    commentIndex =
%          69
% 
%    word =
%          varargout
%    remainderIndex =
%          11
% 
%    word =
%          =
%    remainderIndex =
%          3
% 
%    word =
%          getNextWord
%    remainderIndex =
%          12
%
%    word =
%          string, varargin, 'Delimiter', ' '
%    remainderIndex =
%          0
%
% In the above example, all the meaningful parts of a function call/declaration are extracted for
% independent examination and processing. Subsequent (recursive) calls by the user of this function
% can be used to parse out and process block-words, such as those inside the parentheses and quotes.
% Using the default options should enable a user of this parser to interpret Matlab code or anything 
% syntactically similar.
%
% EXAMPLE
%
%    s = '''I''''''''m'' a test string.'
%    s =
%       'I''''m' a test string.
%    getNextWord(s)
%    ans =
%         I''m
%
% ToDo: It might be worthwhile to bump up the single character items into character classes.
%       Specifically, delimiter should accept all whitespace, not just ' '.
%       Then again, this is already highly functional, and that may just be overkill.
%
% Changed: 
%          Tim O'Connor 4/7/04 - Allow classes of delimiters. - TO040704a
%          Tim O'Connor 4/7/04 - Compile the parser options into a reusable struct. - TO040704b
%           
% See Also STRTOK, TEXTREAD, TOKENIZE, STRINGTOCELL

%Idiot proofing...
if ~strcmpi(class(string), 'char')
    error('MATLAB:badopt', 'The first argument to getNextWord must be a Matlab character array (string).');
end

%Default return values.
word = '';
remainderIndex = 0;
commentIndex = length(string);
returnParser = 0;

%That was easy.
if isempty(string)
    varargout{1} = word;
    
    if nargout > 1
        varargout{2} = remainderIndex;
    end
    
    if nargout > 2
        varargout{3} = commentIndex;
    end
    
    return;
end

%Check number of arguments.
if mod(nargin - 1, 2) ~= 0
    error('MATLAB:badopt', 'All key-value pairs must consist of exactly 2 arguments.');
end

%Default properties.
parser.delimiter = sprintf(' \n\t\r');
parser.comment = '%';
parser.strDelim = '''';
parser.groupers = {'()', '[]', '{}'};
parser.groupStart = '';
parser.groupEnd = '';
parser.grouperString = '';
parser.operators = '.+-*/<>?~^=;';

%Process key-value pairs.
for i = 1 : 2 : nargin - 1
    if strcmpi(varargin{i}, 'DELIMITER')
        parser.delimiter = varargin{i + 1};

        if ~strcmpi(class(parser.delimiter), 'char') 
            %TO040704a - No longer enforces that 'delimiter' be of length 1.
            error('MATLAB:badopt', sprintf('Invalid DELIMITER: ''%s''', parser.delimiter));
        end
    elseif strcmpi(varargin{i}, 'PARSER')
        parser = varargin{i + 1};
        
        if ~strcmpi(class(parser), 'struct')
            error('MATLAB:badopt', sprintf('Invalid parser, must be a struct: %s', class(parser)));
        end
        
        if length(fieldnames(parser)) ~= 8
            error('MATLAB:badopt', sprintf('Invalid parser construct, must have 8 fields: %s', num2str(length(fieldnames(parser)))));
        end
    elseif strcmpi(varargin{i}, 'COMMENT')
        parser.comment = varargin{i + 1};

        if ~strcmpi(class(parser.comment), 'char') | length(parser.comment) > 1
            error('MATLAB:badopt', sprintf('Invalid COMMENT delimiter: ''%s''', parser.delimiter));
        end

        %This will always compare false against any character (I think).
        if isempty(parser.comment)
            parser.comment = -1;
        end
    elseif strcmpi(varargin{i}, 'STRING')
        strDelim = varargin{i + 1};

        if ~strcmpi(class(strDelim), 'char') | length(strDelim) ~= 1
            error('MATLAB:badopt', sprintf('Invalid STRING delimiter: ''%s''', parser.delimiter));
        end        
    elseif strcmpi(varargin{i}, 'GROUPERS')
        parser.groupers = varargin{i + 1};
        parser.grouperString = '';

        for j = i : length(parser.groupers)
            if ~strcmpi(class(parser.groupers{j}), 'char') | length(parser.groupers{j}) ~= 2
                error('MATLAB:badopt', sprintf('Invalid STRING delimiter: ''%s''', parser.delimiter));
            end
            
            grouper = parser.groupers{j};
            parser.groupStart(j) = grouper(1);
        end
    elseif strcmpi(varargin{i}, 'OPERATORS')
        parser.operators = varargin{i + 1};
        
        if ~strcmpi(class(parser.strDelim), 'char')
            error('MATLAB:badopt', sprintf('Invalid OPERATOR list: ''%s''', parser.delimiter));
        end
    elseif strcmpi(varargin{i}, 'RETURNPARSER')
        returnParser = varargin{i + 1};
        
        if ~isnumeric(returnParser)
            error('MATLAB:badopt', sprintf('Invalid RETURNPARSER flag, must be numeric: %s', class(returnParser)));
        end
    else
        error('MATLAB:badopt', sprintf('Invalid property specified: ''%s'' = %s', varargin{i}, varargin{i + 1}));
    end
end

%Break these out of their cell array.
for i = 1 : length(parser.groupers)
    %Make a set of group start characters.
    grouper = parser.groupers{i};
    parser.groupStart(i) = grouper(1);
    parser.groupEnd(i) = grouper(2);
    
    %We need to know the closers too.
    parser.grouperString = strcat(parser.grouperString, grouper);
end

%Indices bounding the word (inclusive).
wordStart = 1;
wordEnd = length(string);

%Remove leading delimiters.
for i = 1 : length(string)
    if ~ismember(string(i), parser.delimiter)%TO040704a
        wordStart = i;
        break;
    else
        wordStart = i;
    end
end

%Allow the outer parsing loop to be terminated from a nested loop.
stop = 0;

for i = wordStart : length(string)
    if any(string(i) == parser.delimiter)%TO040704a
        %The most simple case.
        wordEnd = i - 1;
        break;
    elseif string(i) == parser.comment
        wordEnd = i - 1;%The word stopped before this.
        commentIndex = i + 1;%The comment starts after this.
        break;%Done.
    elseif string(i) == parser.strDelim
        if i == 1 | any(string(i - 1) == parser.delimiter)%TO040704a
            %Found the beginning of a word.
            wordStart = i + 1;
            word = string(i + 1);
            
            %Scan through and find the end of the word.
            %NOTE: You can't change the loop index from inside a Matlab `for` loop, 
            %use a `while` loop instead.
            j = i + 2;
            while j < length(string)
                if string(j) == parser.strDelim
                    if string(j + 1) == parser.strDelim
                        %Append to the word.
                        word = strcat(word, string(j));
                        
                        %Skip an escaped strDelim.
                        j = j + 1;
                    else                        
                        %We're not on an escaped strDelim.
                        wordEnd = j - 1;
                        stop = 1;
                        j = length(string);
                    end
                else
                    %Append to the word.
                    word = strcat(word, string(j));
                end

                %I wish a for loop would've worked...
                j = j + 1;
            end
        
            break;
        elseif i < length(string) & string(i + 1) ~= parser.strDelim & string(i - 1) ~= parser.strDelim 
            %string(i + 1) ~= strDelim  states that this strDelim is not escaping another strDelim
            %string(i - 1) ~= strDelim states that this strDelim is not, itself, escaped.

            %Found the end of a word.
            wordEnd = i - 1;
            break;
        else
            wordEnd = i;
        end
    elseif any(string(i) == parser.groupStart)
        if i == 1 | any(string(i - 1) == parser.delimiter)%TO040704a
            %Found the beginning of a word.
            wordStart = i + 1;
            
            %Get the ending grouper character.
            parser.groupEnd = parser.grouperString(find(parser.grouperString == string(i)) + 1);

            groupStartCount = 1;
            groupEndCount = 0;
            
            %Scan through and find the end of the word.            
            for j = i + 1 : length(string)
                %Allow nesting of groups.
                if string(j) == string(i)
                    groupStartCount = groupStartCount + 1;
                end
                
                if string(j) == parser.groupEnd
                    groupEndCount = groupEndCount + 1;
                end
                
                if groupEndCount == groupStartCount
                    wordEnd = j - 1;
                    break;
                end
            end

            stop = 1;
            break;
        else
            %Found the end of a word.
            wordEnd = i - 1;
            break;
        end
    elseif any(string(i) == parser.operators)
        if i == 1 | any(string(i - 1) == parser.delimiter)%TO040704a
            %Found an operator as a word.
            wordStart = i;
            wordEnd = i;
        else
            %Found an operator as a delimiter.
            wordEnd = i - 1;
        end
        
        stop = 1;
        break;
    else
        %If this character is not a delimiter, comment-delimiter, string-delimiter, or group-delimiter
        %then just keep incrementing.
        wordEnd = i;
    end
    
    %Allow the outer parsing loop to be terminated from a nested loop.
    if stop
        break;
    end
end

%Get the word.
if isempty(word)
    word = string(wordStart : wordEnd);
end
varargout{1} = word;

if nargout > 1

    %Only do this if requested.
    if wordEnd < length(string)
        
        %Drop the closing grouper, if it exists.
        if wordStart > 1
            if any(string(wordStart - 1) == parser.groupStart) & any(string(wordEnd + 1) == parser.groupEnd)
                wordEnd = wordEnd + 1;
            end
        end
        
        %Scan through, look for non-delimiter, non-comment-delimiter characters.
        for i = wordEnd + 1 : length(string)
            if ~ismember(string(i), parser.delimiter) & string(i) ~= parser.comment%TO040704a
                remainderIndex = i;
                break;
            end
        end
    else
        %There's nothing left.
        remainderIndex = 0;
    end
    
    varargout{2} = remainderIndex;
end

if nargout > 2
    %Only do this if requested.
    if commentIndex == length(string) & wordEnd < length(string)
        %Scan through, looking for trailing comments.
        for i = wordEnd + 1 : length(string)
            if string(i) == parser.strDelim
                %Watch out for meaningless comment delimiters inside strings.
                continue;
            elseif any(string(i) == parser.groupStart)
                %Watch out for meaningless comment delimiters inside word-blocks.
                continue;
            elseif string(i) == parser.comment
                %Got it.
                commentIndex = i;
                break;
            end
        end
    end
    
    if commentIndex > wordEnd
        for i = commentIndex - 1 : -1 : wordEnd + 1
            if any(string(i) == parser.delimiter)
                %Found trailing delimiters, between the final word and the comment, it can be safely removed.
                commentIndex = i;
            else 
                break;
            end
        end
    end
    
    varargout{3} = commentIndex;
end

%Return the parser struct, if requested. - TO040704b
if nargout == 4 | returnParser
    varargout{length(varargout) + 1} = parser;
end

return;

% Here's a little profiling script, to determine performance:
% 
% t = 0;
% tio = 0;
% n = 50;
% for i = 1 : n
%     f = fopen('getNextWord.m');
% 
%     line = fgetl(f);
% 
%     tic;
% 
%     while isempty(line) | line ~= -1
%         
%         line = fgetl(f);
%     end
%     
%     tio = tio + toc;
%     
%     fclose(f);
% end
% 
% fprintf(1, '\n\nRead ''getNextWord.m'' %s times in an average of %s seconds.\n', num2str(n), num2str(tio / n));
% 
% for i = 1 : n
%     f = fopen('getNextWord.m');
% 
%     line = fgetl(f);
% 
%     tic;
% 
%     [w r c] = getNextWord(line);
%     while isempty(line) | line ~= -1
% 
%         if ~isempty(line)
%             while r
%                 [w r c] = getNextWord(line(ri : c));
%                 ri = r + ri;
%             end
%         end
%         
%         line = fgetl(f);
%     end
%     
%     t = t + toc;
%     
%     fclose(f);
% end
% 
% fprintf(1, 'Parsed ''getNextWord.m'' %s times in an average of %s seconds.\n', num2str(n), num2str(t / n));
% fprintf(1, 'Average time spent parsing (accounting for I/O overhead): %s seconds.\n\n', num2str((t - tio) / n));