function setGUIValue(prog_obj,handle,value)
%SETGUIVALUE   - Sets data in GUI handle to value.
%   SETGUIVALUE verifies the value of the GUI with handle based on its style (such
%   as edit, popupmenu, slider, etc...) and updates the appropriate field
%   with that value.
%
%   See also GETGUIVALUE

% Changes:
%   % TPMOD031704a: Fixed checking of min and max.
%     TO063004a: Put in some meaningful error messages. -- Tim O'Connor 6/30/04
%     TO101204g: Added some new variable classes (array, etc) and modifiers. -- Tim O'Connor 10/12/04
%     TO101204h: Account for 'text' style gui elements. -- Tim O'Connor 10/12/04
%     TO101204i: Make class specifications case insensitive. -- Tim O'Connor 10/12/04
%     TO101204j: Make class assumptions based on data type and gui element style. -- Tim O'Connor 10/12/04
%     TO101904a: The strrep was added, to make things "scroll" better inside text boxes. -- Timothy O'Connor 10/19/04
%     TO122904a: Allow popupmenus and listboxes to properly and transparently support the char class for their variables. -- Tim O'Connor 12/29/04
%     TO21605c: Call getStackTraceString, instead of just warning, which can be useless because it sucks. -- Tim O'Connor 2/16/05
%     TO091205B: Handle empty values (specifically cell arrays) for popupmenus and listboxes. -- Tim O'Connor 9/12/05
%     TO030706F: Had to incorporate a calls to `any` to keep array sizes consistent across logical operators. -- Tim O'Connor 3/7/06
%
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2004

% Yields correct value and type of the output.
%Start UICONTROL Switch Yard.
style=get(handle, 'Style');         % Style of uicontrol
strval=get(handle, 'String');       % String of uicontrol
cur_val=get(handle, 'Value');      % Numeric Value of uicontrol
userdata=get(handle,'UserData');    % UserData of uicontroluser

% vStr = '';
% if isnumeric(value)
%     vStr = ndArray2Str(value);
% else
%     vStr = value;
% end
% fprintf(1, '%s-%s: %s\n', get(getParent(handle, 'figure'), 'Tag'), get(handle, 'Tag'), vStr);

% TPMOD031704a: Fixed checking of min and max.
if isfield(userdata,'Max') 
    maxval=userdata.Max;
else
    maxval=get(handle, 'Max');          % Max value of uicontrol
end

if isfield(userdata,'Min') 
    minval=userdata.Min;
else
    minval=get(handle, 'Min');          % Max value of uicontrol
end

% If there is no strucutre for the GUI, it was not declared as a GUI in the
% program manager, and cannot be updated.
if ~isstruct(userdata)
    error('setGUIValue: invalid GUI object.');
    return
else
    userdataFieldNames=fieldnames(userdata);    % UserData Structure Field Names of uicontroluser
end

% Is the value we want set a from a certain class?
% If it is, we must format it correctly.
% By default, the class is a character array (strval).
if any(strcmpi(userdataFieldNames,'class'))
    this_class=userdataFieldNames{strcmpi(userdataFieldNames,'class')};
    if strcmpi(userdata.(this_class),'int')%TO101204i
        if ischar(value)
            value=0;
        else
            value=round(value);
        end
    elseif strcmpi(userdata.(this_class),'bool')%TO101204i
        if ischar(value)
            value=0;
        elseif value >= 1
            value=1;
        else
            value=0;
        end
    elseif strcmpi(userdata.(this_class), {'double', 'numeric'})%TO101204i
        if ischar(value) 
            value=0;
        else
            value=double(value);
        end
    elseif strcmpi(userdata.(this_class),'char')%TO101204i, 
        if ischar(value)
            % If the value passed is a character array (string) then just set it and
            % be done.
            if any(strcmpi(style, {'popupmenu', 'listbox'}))
                if iscell(strval)
                    index = find(strcmp(strval, value));
                    if isempty(index)
                        index = 1;
                        %TO21605c - This (getStackTraceString) works better than simply warning, for debugging.
                        warning('@progmanager/setGUIValue: could not find ''%s'' as an option in %s::%s\n%s', value, ...
                            get(getParent(handle, 'figure'), 'Tag'), get(handle, 'Tag'), getStackTraceString);
                    end
                    set(handle, 'Value', index);
                end
            else
                %TO122904a - Don't reset the string on popupmenus and listboxes when the value gets changed.
                set(handle, 'String', value);
            end
            return;
        end
    elseif strcmpi(userdata.(this_class), {'array', 'numericarray'})%TO101204g
        %No-Op. Anything to be done here?
    end
elseif ischar(value) & strcmpi(style,{'edit', 'text'})%TO101204j
    % If the value passed is a character array (sring) then just set it and
    % be done.
    set(handle,'String',value);
    return;
%TO030706F: Had to incorporate a calls to `any` to keep array sizes consistent across logical operators. -- Tim O'Connor 3/7/06
elseif isnumeric(value) & ~any((strcmpi(style,{'edit', 'text'})) | ...
        any((strcmpi(style, {'checkbox', 'radiobutton', 'togglebutton', 'slider'})) & length(value) == 1))%TO101204j
    % If they pass a numeric, and its not declared as numeric (or
    % int,double,etc..) then it is an error.
    %TO063004a - This message must tell which object has the problem, or else it's worthless.
    %TO101204j - Undeclared numerics are now allowed, under the right circumstances.
    error('setGUIValue: numeric value passed to string object - %s::%s', get(getParent(handle, 'figure'), 'Tag'), get(handle, 'Tag'));
end

%  If there are otehr fields restricting the value, make sure value adheres to them.
if any(strcmpi(userdataFieldNames,'min'))
    value=max(minval,value);
end
if any(strcmpi(userdataFieldNames,'max'))
    value=min(maxval,value);
end
if any(strcmpi(userdataFieldNames,'lastvalid'))
    userdata.('lastvalid')=value;
    set(handle,'UserData',userdata);
end
%TO101204g - Allow a length constraints to be enforced.
if any(strcmpi(userdataFieldNames, 'maxlength'))
    if length(value) > userdata.MaxLength
        warning('Variable for GUI element ''%s::%s'' is too long and is being truncated to length: %s', ...
            get(getParent(handle, 'figure'), 'Tag'), get(handle, 'Tag'), num2str(userdata.MaxLength));
        value = value(1 : userdata.MaxLength);
    end
end
if any(strcmpi(userdataFieldNames, 'minlength'))
    if length(value) > userdata.MinLength
        warning('Variable for GUI element ''%s::%s'' is too short and is being padded to length: %s', ...
            get(getParent(handle, 'figure'), 'Tag'), get(handle, 'Tag'), num2str(userdata.MinLength));
        value(end + 1 : userdata.MinLength) = 0;
    end
end

%TO101204g - Downconvert arrays into something that will fit.
if length(value) > 1 & ~strcmpi(style, {'edit', 'text', 'listbox'})
    warning('Array variable found tied to non-array-supporting GUI element ''%s::%s'' of type %s.', ...
        get(getParent(handle, 'figure'), 'Tag'), get(handle, 'Tag'), style);

    if strcmpi(style, 'slider')
        value = value(1);
    else
        value = any(value);
    end
end

% Now set the value of the GUI.
if any(strcmpi(style,{'edit', 'text'}))    % Input a string to set GUI to..only applies to edit boxes. %TO101204h - Include 'text'.
    if length(value) == 1
        set(handle,'String',num2str(value));
    else
        %TO101204g
        %TO101904a - The strrep was added, to make things "scroll" better inside text boxes.
        set(handle, 'String', strrep(ndArray2Str(value), '&', ' & '));
    end
elseif any(strcmpi(style, {'checkbox', 'radiobutton', 'togglebutton', 'slider'}))
    set(handle,'Value',value);
elseif any(strcmpi(style, {'popupmenu', 'listbox'}))
    %TO091205B - Handle empty values (specifically cell arrays). -- Tim O'Connor 9/12/05
    if ~isempty(value)
        if any(size(value)>1)
            value=value(1);
        end
        if value < 1 
            value=1;
        elseif value > size(strval,1)
            value=size(strval,1);
        end
        set(handle,'Value',value,'Min',1,'Max',size(strval,1));
    else
        %This can be a tricky case, when dealing with various empty values.
        set(handle, 'Value', 1, 'Min', 1, 'Max', max(1, size(strval, 1)));
        if strcmpi(style, 'listbox')
            %This property is just retarded and causes stupid problems. Keep an eye on it.
            set(handle, 'ListBoxTop', 1);
        end
    end
else
    warning('No recognized GUI element style for %s-%s: %s', get(getParent(handle, 'figure'), 'Tag'), get(handle, 'Tag'), style);
end