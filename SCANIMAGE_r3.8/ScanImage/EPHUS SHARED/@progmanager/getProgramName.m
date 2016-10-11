function prog_name = getProgramName(prog_manager_obj,program_id)
%GETPROGRAMNAME   -  @progmanager method for outputing the program name from figure.
%   GETPROGRAMNAME(prog_manager_obj,handle) returns the name of the
%   program that the program_id belongs.
%
%   See also SHOWGUIS, SHOWPROGRAMS, GETGUINAME, PARSEPROGRAMID

[prog_name]=parseProgramID(program_id);
