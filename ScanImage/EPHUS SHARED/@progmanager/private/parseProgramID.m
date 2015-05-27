function [program_name,program_obj]=parseProgramID(program_id)
% PARSEPROGRAMID   - @progmanager private method for parsing a program identifier.
% 	PARSEPROGRAMID(program_id) will parse the program_id and extract the program name and 
% 	program_object reference from it.  
% 	
% 	The program_id can be a: string program name, a program object, or a 
% 	handle to a graphics object that is part of the program.  
% 	
%   Returns empty string name and default object if invalid.
%
% 	See also CLOSEPROGRAM, SAVEPROGRAM
%
%  CHANGES
%   TO123005R - This needs to do some error checking. Why am I surprised that it doesn't? -- Tim O'Connor 12/30/05

program_name='';
program_obj=program;
global progmanagerglobal

if isa(program_id,'program') % If it is a program object.
    program_name=get(program_id,'program_name');
    program_obj=program_id;
elseif ischar(program_id) % If it is a program name.
    program_name=program_id;
    if isprogram(progmanager,program_name)
        program_obj=progmanagerglobal.programs.(program_name).program_object;
    end
elseif ishandle(program_id)
    parent_figure=getParent(program_id,'figure');   % Get Parent Figure.
    userdata=get(parent_figure,'UserData');
    if ~isempty(userdata) & isstruct(userdata) & isfield(userdata,'pmobj') & isfield(userdata,'progname')
        program_obj=userdata.program_obj;
        program_name=userdata.progname;
    end
else
    %TO123005R It was returning the empty string when a cell array was passed in. That's some pretty odd behavior that was not expected.
    error('Program IDs must be valid handles or @program object instances: %s', class(program_id));
end
