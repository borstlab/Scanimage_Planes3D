function setglobal(prog_object,variable_name,gui_name,program_name,value,varargin)
% SETGLOBAL   - @progmanager method that sets value of a variable with reference.
%   SETGLOBAL sets the value of the variable specified.  The user must pass
%   in the name of the GUI and Program where the variable lies.
%
%   The program name from within a function in a GUI may be determined by calling
%   getProgramName(obj,gcf) function.  
%
%   The main gui name for a program can be gotten by calling
%   getMainGUIName(obj,program_name);
%
%   The gui name may be determined from within a function in a GUI  by calling
%   getGUIName(obj,gcf) function.  
%
%   varargin are the indices to the array if you want to subsindex into the
%   data stored in the variable.
%
%   Example: (if gui name was 'main' in program 'scanimage')
%
%         setlocal(prog_object,'name','main','newone');
%        
%   is the same as specifying the gui and program name of the
%   variable.
%            
%   See also SETLOCAL, PROGMANAGER, UPDATEGUISFROMVARIABLE
%
% Changes:
%   TP032304a: Supported Strucutures.
%   TO060904a: Fixed else <op>, which was printing erroneous junk. -- Tim O'Connor 6/9/04
%   TO122804a: Make program and gui names case insensitive for set/get purposes. -- Tim O'Connor 12/28/04
%   TO080305A: This can cause serious errors (massive memory allocation, hundreds of MBs) if strings are provided as the varargin. I don't know why, exactly, or how to fix it. -- Tim O'Connor 8/3/05
global progmanagerglobal

% %TO080905TEST1
% if strcmpi(variable_name, 'enable') && strcmpi(program_name, 'userFcns')
% getStackTraceString
% value
% end

%TO122804a - Start
programNames = fieldnames(progmanagerglobal.programs);
programMatches = find(strcmpi(program_name, programNames));
if length(programMatches) > 1
    error('@progmanager/getglobalgh: ambiguous program structures found for ''%s''', program_name);
end
if ~isempty(programMatches)
    program_name = programNames{programMatches};
end

guiNames = fieldnames(progmanagerglobal.programs.(program_name));
guiMatches = find(strcmpi(gui_name, guiNames));
if length(programMatches) > 1
    error('@progmanager/getglobalgh: ambiguous program structures found for GUI ''%s'' in program ''%s''', gui_name, program_name);
end
if ~isempty(programMatches)
    gui_name = guiNames{guiMatches};
end

if isempty(programMatches) | isempty(guiMatches)
    error(['@progmanager/getglobalgh: invalid handle tag ' handle_tag ' for GUI ' gui_name ' in program ' program_name]);
end
%TO122804a - End

% if strcmpi(program_name, 'mapper')
%     if strcmpi(variable_name, 'flashNumber')
% fprintf(1, 'flashNumber = %s\n%s', num2str(value), getStackTraceString);
%     end
% end

% If there is no field, initialize one to an empty.
if ~isfield(progmanagerglobal.programs.(program_name).(gui_name).variables,variable_name)
    progmanagerglobal.programs.(program_name).(gui_name).variables.(variable_name)=[];
end

if isstruct(progmanagerglobal.programs.(program_name).(gui_name).variables.(variable_name))
    % Is there a fieldname supplied?
    fieldname_loc=cellfun('isclass',varargin,'char');
    fieldname=varargin(fieldname_loc);
    fieldname(strcmp(fieldname,':'))=[];
    fieldname_index=find(fieldname_loc);
    arrayindex=varargin(fieldname_index+1:end);

    if length(fieldname) > 1
        error('@progmanager/setglobal: multiple fieldnames for structure supplied, but only 1 allowed.');
    elseif length(fieldname)==1
        fieldname=tokenize(fieldname{1},'.');
        structname=['progmanagerglobal.programs.' program_name '.' gui_name '.variables.' variable_name];
        for subfieldcounter=1:length(fieldname)
            structname=[structname '.' (fieldname{subfieldcounter})];
        end
    else %TO060904a This used to say `isempty(fieldname)` here.
        structname=['progmanagerglobal.programs.' program_name '.' gui_name '.variables.' variable_name];
    end
    if ~isempty(arrayindex)
        structname=[structname '(['];
        for arrayindexcounter=1:length(arrayindex)
            if arrayindexcounter < length(arrayindex)
                structname=[structname num2str(arrayindex{arrayindexcounter}) '],['];
            else
                structname=[structname num2str(arrayindex{arrayindexcounter}) '])'];
            end
        end
    end
    eval([structname '=value;']);
else
    if isempty(varargin)
        progmanagerglobal.programs.(program_name).(gui_name).variables.(variable_name)=value;
    else
        %TO080305A - This can cause serious errors (massive memory allocation, hundreds of MBs) if strings are provided as the varargin. I don't know why, exactly, or how to fix it. -- Tim O'Connor 8/3/05
        progmanagerglobal.programs.(program_name).(gui_name).variables.(variable_name)(varargin{:})=value;
    end
    updateGUIsFromVariable(program_name,gui_name,variable_name);
end