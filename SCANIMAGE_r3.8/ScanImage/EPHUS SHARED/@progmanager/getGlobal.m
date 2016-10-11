function out=getglobal(prog_object,variable_name,gui_name,program_name,varargin)
% GETGLOBAL   - @progmanager method that gets value of a variable with reference.
%   GETGLOBAL gets the value of the variable specified.  The user must pass
%   in the name of the GUI and Program where the variable lies.
%
%  SYNTAX
%        var = getGlobal(obj, variableName, guiName, programName)
%        var = getGlobal(obj, variableName, guiName, programName, [startIndex endIndex])
%
%   The program name from within a function in a GUI may be determined by calling
%   getProgramName(prog_object,handle) function.  
%
%   The gui name may be determined from within a function in a GUI  by calling
%   getGUIName(prog_object,handle) function.  
%
%   varargin are the indices to the array if you want to subsindex into the
%   data stored in the variable.
%
%   Example: (if gui name was 'main' in program 'scanimage')
%
%         val=getglobal(prog_object,'name','main','scanimage',1:10,2:3);
%        
%   is the same as specifying the gui and program name of the
%   variable and indexing the data there by (1:10,2:3).
%
%   Support for Strucutres is now included.  varargin is assumed now to
%   contain the indices into the structure (first and smae as above
%   indexing for arrays), followed by a single string which is the path to
%   the variable int he structure, whicht en can be followed by additional
%   indices to the strucutre (this allows for indexing like
%   struct(1,2).data(2:10)).
%
%   For example, the structure options with fields new=1 and old='aaa' was
%   stored in a variable called currentoptions:
%
%       options.new=[1 2 3];
% 		options.old='aaa';
% 		setlocal(progmanager,hObject,'currentoptions',options);
%   
%       We could get the data stored in options.new(1:2) by specifying:
%           getlocal(progmanager,hObject,'currentoptions','new',1:2);
%         
%   See also GETLOCAL, GETMAIN, SETLOCAL, SETGLOBAL, SETMAIN, PROGMANAGER

% Changes:
%   TP032304a: Added support for structure arrays.
%   TO040604a: Fixed a subsref bug. -- Tim O'Connor 4/6/04
%   TO060804c: Removed an answer getting printed. -- Tim O'Connor 4/6/04
%   TO122804a: Make program and gui names case insensitive for set/get purposes. -- Tim O'Connor 12/28/04
%   TO032406G: Cleaned up yet another error message, so that it is both legible and informative. -- Tim O'Connor 3/24/06

global progmanagerglobal
if nargin >= 4
    programNames = fieldnames(progmanagerglobal.programs);
    programMatches = find(strcmpi(program_name, programNames));
    if length(programMatches) > 1
        error('@progmanager/getglobal: ambiguous program structures found for ''%s''', program_name);
    end
    if ~isempty(programMatches)
        program_name = programNames{programMatches};
    end
    
    guiNames = fieldnames(progmanagerglobal.programs.(program_name));
    guiMatches = find(strcmpi(gui_name, guiNames));
    if length(programMatches) > 1
        error('@progmanager/getglobal: ambiguous program structures found for GUI ''%s'' in program ''%s''', gui_name, program_name);
    end
    if ~isempty(programMatches)
        gui_name = guiNames{guiMatches};
    end
    
    if isempty(programMatches) | isempty(guiMatches)
        error(['@progmanager/getglobal: invalid handle tag ' handle_tag ' for GUI ' gui_name ' in program ' program_name]);
    end

    if isfield(progmanagerglobal.programs.(program_name).(gui_name).variables,variable_name)
        if isstruct(progmanagerglobal.programs.(program_name).(gui_name).variables.(variable_name))
            % Is there a fieldname supplied?

            fieldname_loc=cellfun('isclass',varargin,'char');
            fieldname=varargin(fieldname_loc);
            fieldname(strcmp(fieldname,':'))=[];
            fieldname_index=find(fieldname_loc);

            if length(fieldname) > 1
                error('@progmanager/getglobal: multiple fieldnames for structure supplied, but only 1 allowed.');
            elseif length(fieldname)==1
                fieldname=tokenize(fieldname{1},'.');
                arrayindex=varargin(fieldname_index+1:end);
                for subfieldcounter=1:length(fieldname)
                    if subfieldcounter==1
                        out=progmanagerglobal.programs.(program_name).(gui_name).variables.(variable_name).(fieldname{subfieldcounter});
                    else
                        out=out.(fieldname{subfieldcounter});
                    end
                end
                out=out(arrayindex{:});
            else %TO060804c Removed `isempty(fieldname)` here. -- Tim O'Connor 6/8/04
                out=progmanagerglobal.programs.(program_name).(gui_name).variables.(variable_name);
            end
        elseif nargin > 4
            out=progmanagerglobal.programs.(program_name).(gui_name).variables.(variable_name)(varargin{:});
        else
            %4/6/04 - Some objects don't like doing a subsref with an empty cell array as the index. -- Tim O'Connor TO040604a
            %         Specifically, a timer object will choke.
            out=progmanagerglobal.programs.(program_name).(gui_name).variables.(variable_name);
        end
    else
        error('@progmanager/getglobal: invalid variable name: %s:%s:%s\n%s\n', program_name, gui_name, variable_name, getStackTraceString);%TO032406G
    end
else
    error('@progmanager/getglobal: must supply 4 inputs.  See help for details.');
end