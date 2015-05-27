function value_out=getGUIValue(prog_obj,handle)
%GETGUIVALUE   - Gets data in GUI handle specified, as well as its type.
%   GETGUIVALUE gets the value of the GUI with handle based on its style (such
%   as edit, popupmenu, slider, etc...) and defined by the user in the
%   class field of the handle UserData strucutre.
%
%   See also SETGUIVALUE
%
%  Changed
%      TO040804b: Fixed error (see below). Made string comparisons case insensitive. -- Tim O'Connor 4/8/04
%      TO060804d: Fixed error with random use of `strval` and `str_val` as variable names. -- Tim O'Connor 6/8/04
%      TO083104a: Make sure the variable classes are case insensitive. -- Tim O'Connor 8/31/04 
%      TO101204g: Added some new variable classes (array, etc) and modifiers. -- Tim O'Connor 10/12/04
%      TO122904a: Allow popupmenus and listboxes to properly and transparently support the char class for their variables. -- Tim O'Connor 12/29/04
%      TO020905a: Handle the case of an empty String field for a listbox/popupmenu properly. -- Tim O'Connor 2/9/05

% Yields correct value and type of the output.
% Start UICONTROL Switch Yard.
style=get(handle, 'Style');         % Style of uicontrol
str_val=get(handle, 'String');       % String of uicontrol

value=get(handle, 'Value');       % Numeric Value of uicontrol
userdata=get(handle,'UserData');    % UserData of uicontroluser

% If there is no strucutre for the GUI, it was not declared as a GUI in the
% program manager, and cannot be updated.
if ~isstruct(userdata)
    error('getGUIValue: invalid GUI object.');
    return
else
    userdataFieldNames=fieldnames(userdata);    % UserData Structure Field Names of uicontroluser
end

% Now set the value of the GUI.
if strcmpi(style,'edit')    % Input a string to set GUI to..only applies to edit boxes.
    if any(strcmpi(userdataFieldNames,'class'))
        this_class=userdataFieldNames{strcmpi(userdataFieldNames,'class')};
        
        %TO122904a - All this crap used to be executed at the top of the function, even though it's specific to edit boxes.
        if iscellstr(str_val)
            str_val=str_val{1};
        end
        string2val=str2num(str_val);          % Convert string to array.  Empty if string had any letters in it.

        %Make sure these are case insensitive. -- Tim O'Connor 8/31/04 TO083104a
        if strcmpi(userdata.(this_class),'char')
            value_out=str_val;%TO060804d
        elseif strcmpi(userdata.(this_class),'double') | strcmpi(userdata.(this_class),'numeric')
            %TO040804b - The second part of the expression above was doing
            %a compare against "this_class" not "userdata.(this_class)".
            value_out=double(string2val);
        elseif strcmpi(userdata.(this_class),'bool')
            if string2val >= 1
                value_out=1;
            else
                value_out=0;
            end
        elseif strcmpi(userdata.(this_class),'int')
            value_out=round(string2val);
        elseif any(strcmpi(userdata.(this_class), {'array', 'numericarray'}))%TO101204g
            value_out = ndArrayFromStr(str_val);
        else
            warning('@progmanager/getGUIValue found an unsupported class specification in the UserData field of handle: ''%s::%s''; Class: ''%s''', ...
                get(getParent(handle, 'figure'), 'Tag'), get(handle, 'Tag'), userdata.(this_class));
        end
    else
        value_out=str_val;%TO060804d
    end
elseif any(strcmpi(style, {'popupmenu', 'listbox'}))
    %TO122904a
    this_class=userdataFieldNames{strcmpi(userdataFieldNames,'class')};

    if strcmpi(userdata.(this_class),'char')
        if iscell(str_val)
            %TO020905a - Handle the empty str_val case. - Tim O'Connor 2/9/05
            if ~isempty(str_val)
                value_out = str_val{value};
            else
                warning('@progmanager/getGUIValue: Encountered a %s with no options (''String'' is empty) for handle: ''%s::%s''', style, get(getParent(handle, 'figure'), 'Tag'), get(handle, 'Tag'));
                value_out = '';
            end
        else
            if value ~= 1
                warning('@progmanager/getGUIValue: found incompatible value and string properties for handle: ''%s::%s''', get(getParent(handle, 'figure'), 'Tag'), get(handle, 'Tag'));
            end
            value_out = str_val;
        end
    elseif any(strcmpi(userdata.(this_class), {'numeric', 'double', 'int', 'integer'}))
        value_out = value;
    else
        warning('@progmanager/getGUIValue found an unsupported class specification in the UserData field of handle: ''%s::%s''; Class: ''%s''', ...
            get(getParent(handle, 'figure'), 'Tag'), get(handle, 'Tag'), userdata.(this_class));
    end
elseif any(strcmpi(style, {'checkbox', 'radiobutton', 'togglebutton', 'slider'} ))
    value_out=value;
end

%TO101204g - Allow a length constraints to be enforced.
if any(strcmpi(userdataFieldNames, 'maxlength'))
    if length(value) > userdata.MaxLength
        warning('Variable for GUI element ''%s::%s'' is too long and is being truncated to length: %s', ...
            get(getParent(handle, 'figure'), 'Tag'), get(handle, 'Tag'), num2str(userdata.MaxLength));
        value_out = value_out(1 : userdata.MaxLength);
    end
end
if any(strcmpi(userdataFieldNames, 'minlength'))
    if length(value) > userdata.MinLength
        warning('Variable for GUI element ''%s::%s'' is too short and is being padded to length: %s', ...
            get(getParent(handle, 'figure'), 'Tag'), get(handle, 'Tag'), num2str(userdata.MinLength));
        value_out(end + 1 : userdata.MinLength) = 0;
    end
end

return;