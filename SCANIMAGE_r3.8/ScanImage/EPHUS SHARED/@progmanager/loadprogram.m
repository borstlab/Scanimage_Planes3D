function varargout=loadprogram(obj,program_id,filename)
%LOADPROGRAM   - @progmanager method for loading a program from a program object stored on disk.
%   LOADPROGRAM(obj) will add the program stored in a MAT file to the
%   program manager by calling the file browser for file selection.
%
% 	LOADPROGRAM(obj,program_id) will add the program stored in a user selected MAT file to the
%   program manager using the program_id path setup.
%
% 	LOADPROGRAM(obj,program_id,filename) will add the program stored in a MAT file filename to the
%   program manager using the program_id path setup.
%
% See also OPENPROGRAM

% Parse inputs.
if nargin < 2
    program_name='';
    program_obj=program;
    filename='';
elseif nargin==2
    [program_name,program_obj]=parseProgramID(program_id);
    filename='';
else
    [program_name,program_obj]=parseProgramID(program_id);
end


% pick out a correct path to look for files.
if isempty(filename) & ~isempty(program_name)
    search_path=pwd;
    if exist(getProgramProp(obj,program_name,'program_object_filename'))==2
        search_path=fileparts(getProgramProp(obj,program_name,'program_object_filename'));
    end
    cd(search_path);
end
[program_obj,filename]=loadprogram(program_obj,filename);   % Load program from disk.
if ~isempty(program_obj) & isa(program_obj,'program')
    openprogram(obj,program_obj);
end

if nargout==1
    varargout{1}=filename;
end