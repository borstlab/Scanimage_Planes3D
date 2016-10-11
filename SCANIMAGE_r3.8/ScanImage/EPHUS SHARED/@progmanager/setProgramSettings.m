% PROGMANAGER/setProgramSettings
%
% SYNTAX
%  setProgramSettings(progmanager, hObject, settings)
%   progmanager - Program Manager.
%   hObject - Program handle.
%   settings - A structure of settingsobjects, one for each gui in the program.
%
% USAGE
%
% NOTES
%
% CHANGES
%  TO080905F: Watch out for empty variable lists. -- Tim O'Connor 8/9/05
%  TO081605A: Changed `genericLoadSettings` into `genericPreLoadSettings` and `genericPostLoadSettings`.
%  TO112305A: Use guiObject instead of hObject, because hObject refers to only one gui in the program. -- Tim O'Connor 11/23/05
%  TO113005A: Get/set metadata (String) for GUI elements when saving/loading settings. -- Tim O'Connor 11/30/05
%  TO062306D: Created a lightweight configuration (miniSettings), mainly for use in cycles. Only important run-time variables should get this value. -- Tim O'Connor 6/23/06
%  TO062806G: Removed the return value (`settings`) from the function declaration. -- Tim O'Connor 6/28/06
%
% Created 7/16/05 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function setProgramSettings(this, hObject, settings)
global progmanagerglobal;

[programName, programObj] = parseProgramID(hObject);

guinames = fieldnames(progmanagerglobal.programs.(programName).guinames);

if length(guinames) ~= length(fieldnames(settings))
    warning('Mismatch - number of settings objects does not match number of available GUIs for program: %s', programName);
end

for i = 1 : length(guinames)
    if ~isfield(settings, guinames{i})
        warning('No settings found for GUI %s in program %s.', guinames{i}, programName);
        continue;
    end
    if ~strcmp(programName, getProgramName(settings.(guinames{i}))) | ~strcmp(guinames{i}, getGuiName(settings.(guinames{i})))
        warning('Settings misapplied. Recieved settings for %s:%s in %s:%s.', getProgramName(settings.(guinames{i})), getGuiName(settings.(guinames{i})), ...
            programName, guinames{i});
    end
    
    %TO062306D
    if strcmpi(getMeta(settings.(guinames{i}), 'SETTINGS_TYPE'), 'LIGHT')
        preEvent = 'genericPreLoadMiniSettings';
        postEvent = 'genericPostLoadMiniSettings';
    else
        preEvent = 'genericPreLoadSettings';
        postEvent = 'genericPostLoadSettings';
    end
    
    try
        %TO062306D
        feval(progmanagerglobal.programs.(programName).guinames.(guinames{i}).funchandle, preEvent, ...
            progmanagerglobal.programs.(programName).guinames.(guinames{i}).fighandle, [], ...
            progmanagerglobal.programs.(programName).guinames.(guinames{i}).fighandle);
    catch
        %TO062306D
        warning('Encountered a malfunctioning %s function for %s:%s - %s', preEvent, programName, guinames{i}, getLastErrorStack);
    end
 
    version = feval(progmanagerglobal.programs.(programName).guinames.(guinames{i}).funchandle, 'getVersion', ...
        progmanagerglobal.programs.(programName).guinames.(guinames{i}).fighandle, [], ...
        progmanagerglobal.programs.(programName).guinames.(guinames{i}).fighandle);
    if version ~= getVersion(settings.(guinames{i}))
        fprintf(1, 'Warning: Version mismatch for %s''s settings in program %s. Expected: %s,  Found: %s\n', guinames{i}, programName, num2str(version), num2str(getVersion(settings.(guinames{i}))));
    end
    
    variables = get(settings.(guinames{i}));
    %TO080905F - Watch out for empties.
    if isempty(variables)
        variableNames = {};
    else
        variableNames = fieldnames(variables);
    end

%     otherguis=progmanagerglobal.programs.(program_name).(gui_name).variableGUIs.(variable_name);
%     for guicounter=1:length(otherguis)
%         [obj_name,gui_name_gui,prog_name_gui]=parseStructString(otherguis{guicounter});
%         setGUIValue(progmanager,progmanagerglobal.programs.(prog_name_gui).(gui_name_gui).guihandles.(obj_name),...
%             progmanagerglobal.programs.(program_name).(gui_name).variables.(variable_name));
%     end
    %TO112305A - Use guiObject instead of hObject, because hObject refers to only one gui in the program. -- Tim O'Connor 11/23/05
    guiObject = progmanagerglobal.programs.(programName).guinames.(guinames{i}).fighandle;

    s = 'setLocalBatch(this, guiObject';
    for j = 1 : length(variableNames)
        s = [s ', ''' variableNames{j} ''', variables.' variableNames{j}];
% fprintf(1, 'Loading %s:%s:%s\n', programName, guinames{i}, variableNames{j});
        %TO113005A
        if isfield(progmanagerglobal.programs.(programName).(guinames{i}).variableGUIs, variableNames{j})
            guis = progmanagerglobal.programs.(programName).(guinames{i}).variableGUIs.(variableNames{j});

            try
                %This has some potential bugs (clobbering), when a variable is tied to multiple "list" type GUI elements. For now, I think it should be okay. -- Tim O'Connor 11/28/05
                for k = 1 : length(guis)
                    [tiedGuiElement, tiedGuiName, tiedProgramName] = parseStructString(guis{k});
                    if any(strcmpi(get(progmanagerglobal.programs.(tiedProgramName).(tiedGuiName).guihandles.(tiedGuiElement), 'Style'), {'popupmenu', 'listbox'}))
% fprintf(1, 'setting from metadata: ''%s'' - ''%s''\n', variableNames{j}, getMeta(settings.(guinames{i}), variableNames{j}));
                        set(progmanagerglobal.programs.(tiedProgramName).(tiedGuiName).guihandles.(tiedGuiElement), 'String', getMeta(settings.(guinames{i}), variableNames{j}));
                    end
                end
            catch
                warning('Failed to load metadata for %s:%s:%s. - %s', programName, guinames{i}, variableNames{j}, lasterr);
            end
        end
    end
    eval([s ');']);
% fprintf(1, 'Setting variables for %s:%s - \n%s);\n\n', programName, guinames{i}, s);
% s
% f = fieldnames(variables);
% for j = 1 : length(f)
%     fprintf(1, '%s:\n', f{j});
%     variables.(f{j})
%     fprintf(1, '-------\n\n');
% end
%     try
        %TO062306D
        feval(progmanagerglobal.programs.(programName).guinames.(guinames{i}).funchandle, postEvent, ...
            progmanagerglobal.programs.(programName).guinames.(guinames{i}).fighandle, [], ...
            progmanagerglobal.programs.(programName).guinames.(guinames{i}).fighandle);
%     catch
%         %TO062306D
%         warning('Encountered a malfunctioning %s function for %s:%s - %s', postEvent, programName, guinames{i}, lasterr);
%     end
end

return;