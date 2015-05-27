function reloadprogram(obj,program_obj)
%RELOADPROGRAM   - @progmanager method for reloading a program already running from a program PMP file.
%   RELOADPROGRAM will identify the program object, validate it, and will
%   then load variables and figure properties of the object to the current
%   settings.  it will then call the generifUpdateFunction for each GUI to
%   make sure nothing breaks on loading.
%

if nargin < 2
    error(['@progmanager/reloadprogram: requires 2 input variables.']);
end

if ~isa(program_obj,'program')
    error(['@progmanager/reloadprogram: second input must be a program object.']);
end
    
% Parse the input program object.
program_name=get(program_obj,'program_name');
allnames=getaliases(program_obj);
guinames=allnames(1:2:end);

global progmanagerglobal

if ~isprogram(obj,program_name)
    error(['@progmanager/reloadprogram: program ' program_name ' does not exist.']);
end

% Check versions on reloading.
if ~checkVersion(obj,program_obj)
    beep;
    warning('@PROGMANAGER/reloadprogram: Versions do not match.  Program may not operate correctly.  Update versions and resave program.');
end

% Now count through all the gui aliases and load them.
for counter=1:length(guinames)
    currentFigHandle=getHandleFromname(obj,guinames{counter},program_name);
    UserData=get(currentFigHandle,'UserData');
    UserData.program_obj=program_obj; % Remember the program object in each figure.
    program_fig_properties=getfigprops(program_obj,guinames{counter}); % Get program default figure properties for GUI.
    if ~any(cellfun('isempty',program_fig_properties))  %If they are all valid and not empty...
        set(currentFigHandle,program_fig_properties{:});   % Set the GUI properties to the ones from the program object.
    end
    vars_to_load=getvariables(program_obj,guinames{counter});
    if ~isempty(vars_to_load)
        var_names=fieldnames(vars_to_load);
        for var_counter=1:length(var_names)
            setglobal(obj, var_names{var_counter}, guinames{counter}, program_name,vars_to_load.(var_names{var_counter}));
        end
        try
            feval(progmanagerglobal.programs.(program_name).guinames.(guinames{counter}).funchandle,'genericUpdateFcn',currentFigHandle,[],currentFigHandle);   
        end
    end
end