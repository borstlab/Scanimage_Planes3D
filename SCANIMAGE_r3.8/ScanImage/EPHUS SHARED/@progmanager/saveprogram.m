function varargout=saveprogram(obj,program_id,filename)
%SAVEPROGRAM   - @progmanager save program method.
%  SAVEPROGRAM(obj,program_id) will save a program object to a MAT file stored on
%  disk.  The default file extension is .PMP (Program Manager Program).  
%  With no input arguments, the default is to open the file browser.
%
%  SAVEPROGRAM(obj,filename) will save obj into the filename given.
%
%  See also LOADPROGRAM

% Parse inputs.
out=0;
if nargin < 2
    error(['@progmanager/saveprogram: requires 2 input variables, a progmanager and a program object, name, or handle.']);
else
    [program_name,program_obj]=parseProgramID(program_id);
end
if nargin == 2
    filename='';
end

% Select file from disk if none was supplied.
if isempty(filename)
    search_path=pwd;
    if exist(getProgramProp(obj,program_name,'program_object_filename'))==2
        search_path=fileparts(getProgramProp(obj,program_name,'program_object_filename'));
    end
    [fname,pname]=uiputfile(fullfile(search_path,[program_name '.pmp']),['Save ' program_name ' As..']);
    if isnumeric(fname)
        return
    end
    filename=fullfile(pname,fname);
end
save(filename,'program_obj', '-mat');
setProgramProp(obj,program_name,'program_object_filename',filename); %Remember file we loaded object from.
setProgramProp(obj,program_name,'program_needs_saving',0); % Saved so we dont have to worry about cecking on close.
% set(getMenuHandles(obj,program_name,'Tag','save'),'Enable','Off');
% pass out output if the user requests one.
if nargout==1
    varargout{1}=1;
end