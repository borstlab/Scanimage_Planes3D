% PROGMANAGER/getProgramMiniSettings
%
% SYNTAX
%  settings = getProgramMiniSettings(progmanager, hObject)
%   progmanager - Program Manager.
%   hObject - Program handle.
%   settings - A structure of settingsobjects, one for each gui in the program.
%
% USAGE
%
% NOTES
%  Copy & paste from @progmanager/getProgramSettings.m
%
% CHANGES
%  TO081605A: Changed `genericSaveSettings` into `genericPreSaveSettings` and `genericPostSaveSettings`.
%  TO112805B: Use `bitand` when checking config flags. -- Tim O'Connor 11/28/05
%  TO113005A: Get/set metadata (String) for GUI elements when saving/loading settings. -- Tim O'Connor 11/30/05
%  TO062306D: Created a lightweight configuration (miniSettings), mainly for use in cycles. Only important run-time variables should get this value. -- Tim O'Connor 6/23/06
%
% Created 6/23/06 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2006
function settings = getProgramMiniSettings(this, hObject)
global progmanagerglobal;

settings = [];
[programName, programObj] = parseProgramID(hObject);
guinames = fieldnames(progmanagerglobal.programs.(programName).guinames);

for i = 1 : length(guinames)
    try
        feval(progmanagerglobal.programs.(programName).guinames.(guinames{i}).funchandle, 'genericPreSaveSettings', ...
            progmanagerglobal.programs.(programName).guinames.(guinames{i}).fighandle, [], ...
            progmanagerglobal.programs.(programName).guinames.(guinames{i}).fighandle);
    catch
        warning('Encountered a malfunctioning genericPreSaveSettings function for %s:%s - %s', programName, guinames{i}, lasterr);
    end

    version = feval(progmanagerglobal.programs.(programName).guinames.(guinames{i}).funchandle, 'getVersion', ...
        progmanagerglobal.programs.(programName).guinames.(guinames{i}).fighandle, [], ...
        progmanagerglobal.programs.(programName).guinames.(guinames{i}).fighandle);
    settings.(guinames{i}) = settingsobject(programName, guinames{i}, version);
    variableNames = fieldnames(progmanagerglobal.programs.(programName).(guinames{i}).configflags);
    setMeta(settings.(guinames{i}), 'SETTINGS_TYPE', 'LIGHT');%TO062306D

    for j = 1 : length(variableNames)
        if bitand(progmanagerglobal.programs.(programName).(guinames{i}).configflags.(variableNames{j}), 4) %TO112805B, TO062306D
% fprintf(1, '@progmanager/getMiniSettings - Saving: progmanagerglobal.programs.(%s).(%s).configflags.(%s)\n', programName, guinames{i}, variableNames{j});
% fprintf(1, 'Setting %s:%s:%s\n', programName, guinames{i}, variableNames{j});
            set(settings.(guinames{i}), variableNames{j}, progmanagerglobal.programs.(programName).(guinames{i}).variables.(variableNames{j}));

            %TO113005A
            if isfield(progmanagerglobal.programs.(programName).(guinames{i}).variableGUIs, variableNames{j})
                guis = progmanagerglobal.programs.(programName).(guinames{i}).variableGUIs.(variableNames{j});
                try
                    %This has some potential bugs (clobbering), when a variable is tied to multiple "list" type GUI elements. For now, I think it should be okay. -- Tim O'Connor 11/28/05
                    for k = 1 : length(guis)
                        [tiedGuiElement, tiedGuiName, tiedProgramName] = parseStructString(guis{k});
                        if any(strcmpi(get(progmanagerglobal.programs.(tiedProgramName).(tiedGuiName).guihandles.(tiedGuiElement), 'Style'), {'popupmenu', 'listbox'}))
                            setMeta(settings.(guinames{i}), variableNames{j}, get(progmanagerglobal.programs.(tiedProgramName).(tiedGuiName).guihandles.(tiedGuiElement), 'String'));
                        end
                    end
                catch
                    warning('Failed to capture metadata for %s:%s:%s. - %s', programName, guinames{i}, variableNames{j}, lasterr);
                end
            end
% else
% fprintf(1, '@progmanager/getMiniSettings - Ignoring: progmanagerglobal.programs.(%s).(%s).configflags.(%s)\n', programName, guinames{i}, variableNames{j});
        end
    end
    
    try
        feval(progmanagerglobal.programs.(programName).guinames.(guinames{i}).funchandle, 'genericPostSaveSettings', ...
            progmanagerglobal.programs.(programName).guinames.(guinames{i}).fighandle, [], ...
            progmanagerglobal.programs.(programName).guinames.(guinames{i}).fighandle);
    catch
        warning('Encountered a malfunctioning genericPostSaveSettings function for %s:%s - %s', programName, guinames{i}, lasterr);
    end
end

return;