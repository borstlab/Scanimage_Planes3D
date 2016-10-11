function setglobalgh(prog_object,handle_tag,gui_name,program_name,varargin)
% SETGLOBALGH   - @progmanager method that sets properties for handles in GUI object with name TAG handle_tag.
%   SETGLOBALGH sets properties of the handle to a GUI object with name TAG handle_tag.  This
%   is useful for setting properties of the various GUI objects when they
%   are not tied to a variable (like an axes handle).
%            
%   See also GETGLOBALGH
%
%   Changes:
%      Update the variable when changing properties. -- Tim O'Connor 4/2/04 (TO040204a)
%      Make sure that the gui's value is updated, if necessary. -- Tim O'Connor 6/4/04 (TO060404a)
%      Style is not a supported attribute of an 'axes' object. - 6/7/04 Tim O'Connor (TO060704a)
%      Check for bad/empty UserData. - 6/30/04 Tim O'Connor (TO063004b)
%      Watch out for errors here. -- Tim O'Connor 12/23/04 (TO122304b)
%      Make program and gui names case insensitive for set/get purposes. -- Tim O'Connor 12/28/04 (TO122804a)
%      If it's an axes object, set it immediately and return. -- Tim O'Connor 12/28/04 (TO122804b)
%      Set the proper value for listboxes when the string gets changed. -- Tim O'Connor 2/16/05 (TO021605b)
%      Wrapped cell array expansion in brackets, to make it pass into strcmpi as a single cell array. -- Tim O'Connor 5/24/05 (TO052405A)
%      Watch out for empty string arrays. -- Tim O'Connor 7/22/05 (TO072205A)
%      Make sure the value stays in range. -- Tim O'Connor 7/25/05 (TO072505A)
%      Fixed call to cellfun, wrapped with a find. -- Tim O'Connor 9/9/05 (TO090905A)
%      Handle errors with the ListBoxTop property of listboxes when the String property is an empty cell array. -- Tim O'Connor 9/9/05 (TO090905B)
%      Make the empty value for listboxes and popupmenus a cell array. -- Tim O'Connor 9/12/05 (TO091205A)
%      The strcmp didn't like having an empty cell array compared to a non-empty one. -- Tim O'Connor 9/14/05 (TO091405B)
%      Optimization. -- Tim O'Connor 2/27/06 (TO022706D)
global progmanagerglobal
if nargin >= 4
    %TO122804a - Start
    programNames = fieldnames(progmanagerglobal.programs);
    programMatches = find(strcmpi(program_name, programNames));
    if length(programMatches) > 1
        error('@progmanager/setglobalgh: ambiguous program structures found for ''%s''', program_name);
    end
    if ~isempty(programMatches)
        program_name = programNames{programMatches};
    end

    guiNames = fieldnames(progmanagerglobal.programs.(program_name));
    guiMatches = find(strcmpi(gui_name, guiNames));
    if length(programMatches) > 1
        error('@progmanager/setglobalgh: ambiguous program structures found for GUI ''%s'' in program ''%s''', gui_name, program_name);
    end
    if ~isempty(programMatches)
        gui_name = guiNames{guiMatches};
    end
    
    if isempty(programMatches) | isempty(guiMatches)
        error(['@progmanager/setglobalgh: invalid handle tag ' handle_tag ' for GUI ' gui_name ' in program ' program_name]);
    end
    %TO122804a - End
%     if isfield(progmanagerglobal.programs.(program_name).(gui_name).guihandles,handle_tag)
%         set(progmanagerglobal.programs.(program_name).(gui_name).guihandles.(handle_tag),varargin{:});
%     else
%         error(['@progmanager/setglobalgh: invalid handle tag ' handle_tag ' for GUI ' gui_name ' in program ' program_name]);
%     end
else
    error('@progmanager/setglobalgh: must supply 4 inputs.  See help for details.');
end

% fprintf(1, '\nprogmanagerglobal.programs.%s.%s.guihandles.%s\n\n', program_name, gui_name, handle_tag);

%Only need to update the variable if the Value/String/Min/Max has changed. - 4/2/04 Tim O'Connor TO040204a
%Style is not a supported attribute of an 'axes' object. - 6/7/04 Tim O'Connor TO060704a
style = '';
if ~strcmpi(get(progmanagerglobal.programs.(program_name).(gui_name).guihandles.(handle_tag), 'Type'), 'axes')
    style = get(progmanagerglobal.programs.(program_name).(gui_name).guihandles.(handle_tag), 'style');
% else
%     %If it's an axes object, set it now and return. -- 12/28/04 Tim O'Connor TO122804b
%     set(progmanagerglobal.programs.(program_name).(gui_name).guihandles.(handle_tag), varargin{:});
%     return;
end

%TO060404a - This used to handle 'edit', 'listbox', and 'popupmenu' together.
%            Since there's extra operations for 'listbox' and 'popupmenu'
%            I broke them out.
%TO090905A - The cellfun calls should then be passed to a find and used to index into the cell array. -- Tim O'Connor 9/9/05
%TO022706D - Cache the following statement, since in the elseif portions it is recomputed and consumes a lot of time. -- Tim O'Connor 2/27/06
cachedStatement = lower(varargin(find(cellfun('isclass',varargin,'char') == 1)));
if strcmpi(style,'edit') & any(ismember({'string', 'min', 'max'}, cachedStatement )) %TO090905A
    % We updated the GUI, so we should now make sure it is ok...
    updateVariableFromGUI(prog_object,progmanagerglobal.programs.(program_name).(gui_name).guihandles.(handle_tag));
elseif ismember(lower(style),{'listbox','popupmenu'}) & any(ismember({'string', 'min', 'max'}, cachedStatement )) %TO090905A %TO022706D
    %Make sure that the gui's value is updated, if necessary. -- Tim O'Connor 6/4/04 TO060404a
    %For example, if the string for a popupmenu is changed, the correct item must be selected before/after
    %the change. Previously, it wouldn't update the selection to match the tied variable, when changing the
    %'string', 'min', or 'max' properties.
    userdata = get(progmanagerglobal.programs.(program_name).(gui_name).guihandles.(handle_tag), 'UserData');
    if ~isempty(userdata)
        [var_name, gui_name, program_name] = parseStructString(userdata.variable);
        try
            if ~strcmpi(userdata.Class, 'char')
                %TO122304b - Watch out for errors here. -- Tim O'Connor 12/23/04
                set(progmanagerglobal.programs.(program_name).(gui_name).guihandles.(handle_tag), 'Value', ...
                    progmanagerglobal.programs.(program_name).(gui_name).variables.(var_name));
            else
                %TO021605b: Set the proper value for listboxes when the string gets changed. -- Tim O'Connor 2/16/05
                %TO052405A: Wrapped cell array expansion in brackets, to make it pass into strcmpi as a single cell array. -- Tim O'Connor 5/24/05
                index = find(strcmpi({varargin{1:2:end}}, 'String'));
                if  isempty(index) | index < 1
                    str = get(progmanagerglobal.programs.(program_name).(gui_name).guihandles.(handle_tag), 'String');
                else
                    str = varargin{index + 1};
                    if isempty(str)
                        %Make sure this is a cell array, when it's empty, not just a string (and definitely not numeric). -- Tim O'Connor 9/12/05 TO091205A
                        varargin{index + 1} = {};
                        str = {};
                    end
                end

                %TO091405B - The strcmp didn't like having an empty cell array compared to a non-empty one. -- Tim O'Connor 9/14/05
                index = [];
                if ~isempty(progmanagerglobal.programs.(program_name).(gui_name).variables.(var_name))
                    index = find(strcmp(progmanagerglobal.programs.(program_name).(gui_name).variables.(var_name), str));
                end
                if isempty(index) | index < 1
                    if strcmpi(class(str), 'cell')
                        if ~isempty(str)%TO072205A - Watch out for empty string arrays.
                            progmanagerglobal.programs.(program_name).(gui_name).variables.(var_name) = str{1};
                            %TO072505A - Make sure the value stays in range.
                            if get(progmanagerglobal.programs.(program_name).(gui_name).guihandles.(handle_tag), 'Value') > 1
                                set(progmanagerglobal.programs.(program_name).(gui_name).guihandles.(handle_tag), 'Value', 1);
                            end
                        else
                            progmanagerglobal.programs.(program_name).(gui_name).variables.(var_name) = '';
                            if strcmpi(style, 'listbox')
                                %TO090905B - Watch out for this ListBoxTop value when setting empties.
                                set(progmanagerglobal.programs.(program_name).(gui_name).guihandles.(handle_tag), 'ListBoxTop', 1);
                            end
                        end
                    else
                        progmanagerglobal.programs.(program_name).(gui_name).variables.(var_name) = str;
                    end
                else
                    if strcmpi(class(str), 'cell')
                        progmanagerglobal.programs.(program_name).(gui_name).variables.(var_name) = str{index};
                        set(progmanagerglobal.programs.(program_name).(gui_name).guihandles.(handle_tag), 'Value', index);
                    else
                        progmanagerglobal.programs.(program_name).(gui_name).variables.(var_name) = str;
                        set(progmanagerglobal.programs.(program_name).(gui_name).guihandles.(handle_tag), 'Value', 1);
                    end
                end
            end
        catch
            error('Error setting GUI element value for %s-%s-%s: %s', program_name, gui_name, handle_tag, lasterr);
        end
        % This was screwing stuff up. - TO021605b -- Tim O'Connor 2/16/05
        % We updated the GUI, so we should now make sure it is ok...
        %updateVariableFromGUI(prog_object,progmanagerglobal.programs.(program_name).(gui_name).guihandles.(handle_tag));
    else
        %TO063004b - This error came up due to other errors that shouldn't occur, but watch for it anyway.
        warning('Invalid (empty) UserData for %s:%s::%s.', program_name, gui_name, handle_tag);
        %TO120705B - Make this message a little more verbose to help with debugging, should it occur. -- Tim O'Connor 12/7/05
        fprintf(2, '\n---------------------\nDebugging info:\n progmanagerglobal.programs.%s.%s.guihandles.%s\n%s\n---------------------\n', program_name, gui_name, handle_tag, getStackTraceString);
    end
elseif ~ismember(lower(style),{'text','frame','edit'}) & any(ismember({'value', 'min', 'max'}, cachedStatement)) %TO022706D
% elseif ~ismember(lower(style),{'text','frame','edit'}) & any(ismember({'value', 'min', 'max'}, lower(varargin(cellfun('isclass',varargin,'char')))))
    % We updated the GUI, so we should now make sure it is ok...
    updateVariableFromGUI(prog_object,progmanagerglobal.programs.(program_name).(gui_name).guihandles.(handle_tag));
end

%DEBUG
% if strcmpi(program_name, 'stimulator') && strcmpi(handle_tag, 'startButton')
%     fprintf(1, '@progmanager/setGlobalGh: ''%s'':''%s''\n%s', program_name, handle_tag, getStackTraceString);
% end

set(progmanagerglobal.programs.(program_name).(gui_name).guihandles.(handle_tag), varargin{:});