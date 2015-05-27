% getLastErrorStack - Retrieve the a formatted string containing the last error message and stack trace leading to the error, suitable for printing.
%
%%  SYNTAX
%   str = getLastErrorStack
%   str = getLastErrorStack(errorLogFile)
%   str = getLastErrorStack(exception)
%       errorLogFile: name of a file to which to append error stack information
%       exception: A Matlab MException class instance.
%
%%  NOTES
%   Relies on getStackTraceString and The Mathworks's (new as of version 7) lasterror function.
%
%%  CHANGES
%   VI081508A: Added option to append error to an error log file -- Vijay Iyer 8/15/08
%   TO091010B: Allow this to take the new Matlab exception class as an argument. -- Tim O'Connor 9/10/10
%
%% CREDITS
% Created 11/30/07 - Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2007
function str = getLastErrorStack(varargin)

%TO091010B
if ~isempty(varargin) && strcmpi(class(varargin{1}), 'MException')
    ME = varargin{1};
    err.message = ['Exception (' ME.identifier '): ' ME.message];
    err.stack = varargin{1}.stack;
    err.identifier = ME.identifier;
else
    err = lasterror;
end

str = sprintf('LastError - ''%s'': \n\t%s\n\tRoot cause:\n\t%s\n', ...
    err.identifier, ...
    strrep(err.message, char(10), char([10 9 32 32])), ...
    strrep(getStackTraceString(err.stack), char(10), char([10 9])));

%TO091010B
if ~isempty(varargin) && strcmpi(class(varargin{1}), 'MException')
    ME = varargin{1};
    for i = 1 : length(ME.cause)
        try
            str = sprintf('%s\nCause:\n\t%s', str, getLastErrorStack(ME.cause{i}));
        catch
            fprintf(2, 'getLastErrorStack - Failed to parse exception cause(s): %s\n', lasterr);
        end
    end
end

if ~isempty(varargin)
    try
        fid = fopen(varargin{1},'a');
        if fid           
            fprintf(fid,'\n************%s*******************\n%s',datestr(clock),str);
        end
        fclose(fid);
    end
end

return;