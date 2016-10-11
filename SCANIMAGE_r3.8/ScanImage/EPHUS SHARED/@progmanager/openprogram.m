function varargout = openprogram(obj, programInfo, varargin)
%OPENPROGRAM   - @progmanager method for starting/loading a program object, creating it if necesary
% See also ADDPROGRAM, STARTPROGRAM
%
% SYNTAX
%  openprogram(progmanager, prog)
%  prog = openprogram(progmanager, programName)
%  prog = openprogram(progmanager, programInfo) - See @program/program.
%  prog = openprogram(progmanager,..,varargin)
%   progmanager - The program manager.
%   prog - The @program object to be opened.
%   programName - String, representing name of the program to be opened (creates @program via program(programName,programName,programName))
%   programInfo - Cell array, which can consist of 2 elements--{programName programMFileName}--or can consist of 3 or more elements comprising valid arguments to @program constructor, e.g. {programName mnemonicName mFileName} or {programName mainGuiMnemonic mainGuiFile subGuiMnemonic subGuiFile}
%   varargin -- Set of arguments to pass to the program's 'constructor' (its genericStartFcn -- see this function for documentation of required/optional input args)
%
% NOTES
%   Second argument programInfo can be a program itself, or the information required to create a new program (either a string or a string cell array)
%
% CHANGES
%  TO053008F - Allow multiple syntaxes. Documented the syntax, since the original file's documentation was useless. -- Tim O'Connor 5/30/08
%  VI053108A - Restore second argument as a program_object. Handled syntax clean-up in @program constructor itself. -- Vijay Iyer 5/31/08
%  VI053108B - Allow varargin to pass arguments to each individual program's 'constructor' (i.e. its genericStartFcn() function) -- Vijay Iyer 5/31/08
%  VI060208A - Handle common case where a single-gui program has an 'alias' property in a convenient way -- Vijay Iyer 6/2/08

%TO053008F (Commented), VI053108A (Restored with mods)
% Parse inputs.
if nargin < 2
    error(['@progmanager/openprogram: requires at least 2 input variables']); 
end

% if length(varargin) == 2 %VI053108A
if ischar(programInfo)
    program_obj = program(programInfo, programInfo, programInfo);
    if nargout > 0
        varargout{1} = program_obj;
    end
elseif iscellstr(programInfo)
    if length(programInfo)== 1 %do this, so cell array can be used even when not needed
        program_obj = program(programInfo{1},programInfo{1},programInfo{1});
    elseif length(programInfo)== 2 %VI060208A
        program_obj = program(programInfo{1},programInfo{1},programInfo{2});
    else %length(programInfo) >=3
        program_obj = program(programInfo{:});
    end
    
    if nargout > 0
        varargout{1} = program_obj;
    end
elseif isa(programInfo,'program')
    program_obj = programInfo;
    if nargout > 0
        varargout{1} = program_obj;
    end
else
    error('Second argument must be a string, string cell array, or a @program object');
end

addprogram(obj,program_obj);
if isstarted(obj,program_obj)
    reloadprogram(obj,program_obj);
else
    startprogram(obj,program_obj,varargin{:});
end

% if the progmanager display is on, update it.
if getProgmanagerDefaults(obj,'ProgmanagerDisplayOn')
    progmanagerdisp(obj);
end

