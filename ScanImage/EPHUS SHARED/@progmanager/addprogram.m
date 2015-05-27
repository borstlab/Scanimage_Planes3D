function addprogram(obj,program_obj)
%ADDPROGRAM   - @progmanager method for adding a program to a progmanager object.
%   ADDPROGRAM method for progmanager to add a new program.
%   Must supply a progmanager object and a program object to be loaded.
%
%   The program object can be created using the program constructor
%   function.  See PROGRAM for details on cosntruction.
%
% See also CLOSEPROGRAM, STARTPROGRAM, PROGRAM
%
% CHANGES
%  TO032905a: Fixed case sensitivity. -- Tim O'Connor 3/29/05

if nargin < 2
    error(['@progmanager/addprogram: requires 2 input variables.']);
end

if ~isa(program_obj,'program')
    error(['@progmanager/addprogram: second input must be a program object.']);
end

global progmanagerglobal

% Parse the input program object for program name and main gui name.
program_name=get(program_obj,'program_name');
main_gui_name=get(program_obj,'main_gui_name');
guinames=getaliases(program_obj);

% What if the main GUI is not in the program aliases?
if ~ismember(guinames,main_gui_name)
    error(['@progmanager/addprogram: mainGUIname was not is not included in list of GUI aliases.']);
end

% Look at default GUIDE options.
GUIDE_opts=getProgmanagerDefaults(progmanager,'GUIDE_opts');%TO032905a - Fixed case sensitivity.
singleton=1;    % Default allow only 1 copy of each GUI...
if isfield(GUIDE_opts,'singleton')
    singleton=GUIDE_opts.singleton;
end
uniqueGUINames={};  % GUI mfile names.
% Program names must be unique, so check to see if a program with name
% program_name already exists.
if ~isprogram(obj,program_name)
    % if the main GUI name is an alias, then add it to the program manager.
    progmanagerglobal.programs.(program_name).mainGUIname=main_gui_name;  
    tempstruct=struct;  % initialize structure to set in progmanager.
    %Parse the guinames input to see if there are any aliases.
    for guiCounter=1:2:length(guinames)
        % Get the name and alias for the GUI from the guinames cell
        % array.
        name_of_Alias=guinames{guiCounter};
        name_of_GUI=guinames{guiCounter+1};
        % Here we want to make sure the name_of_Alias is unique.  Also
        % want to be sure that if we have singleton GUIOPT is set to 1,
        % then we also have unique GUI names as well.
        if ~isfield(tempstruct,name_of_Alias)
            tempstruct.(name_of_Alias)=[];
            % If the name_of_Alias was unique, we need to now check to
            % see if we need to worry about the GUI names being unique.
            % They have to be unique if the GUIDEOPT 'singleton' in the progmanager
            % is set to 1.
            if  singleton & any(strcmpi(uniqueGUINames,name_of_GUI)) % It is a singleton GUI but there is an mfile with this name already included...
                tempstruct=rmfield(tempstruct,name_of_Alias);    % Remove name of GUI and Alias From progmanager.
                error(['@progmanager/addprogram: GUI name ' name_of_GUI ' was not unique and GUIDEOPT singleton was set to 1.']);
            else
                tempstruct.(name_of_Alias).m_filename=name_of_GUI;  % Set strucutre with mfilename
                uniqueGUINames=unique([uniqueGUINames {name_of_GUI}]);  % Add M file name to list of unique m filenames.
            end
        else
            error(['@progmanager/addprogram: GUI name or alias ' name_of_Alias ' was not unique.']);
        end
        progmanagerglobal.programs.(program_name).guinames=tempstruct;   % Write structure into global array.
    end
    progmanagerglobal.programs.(program_name).started=0;    % Write structure into global array.
    progmanagerglobal.programs.(program_name).program_object=program_obj;  % Store a copy of the program object in the global array. 
    progmanagerglobal.programs.(program_name).program_object_filename=get(program_obj,'filename');  % If program was loaded from disk, store the filename here.
    progmanagerglobal.programs.(program_name).program_needs_saving=0;      % Flag to see if program needs saving
else
    error(['@progmanager/addprogram: Program ' program_name ' is already added to program manager.']);
end