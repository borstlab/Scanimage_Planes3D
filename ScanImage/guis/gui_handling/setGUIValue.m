function setGUIValue(handle, val)
% set the value displayed in a GUI to the given value 

switch get(handle,'Type')
    case 'uicontrol'
        switch get(handle, 'Style')
            case 'edit'
                if hasUserDataField(handle, 'Numeric')
                    if getUserDataField(handle, 'Numeric') & ischar(val)
                        val=str2num(val);
                    end
                end
                set(handle, 'String', val);
            case 'text'
                set(handle, 'String', val);
            case 'slider'
                if ischar(val)
                    val=str2num(val)
                end
                set(handle, 'Value', val);
            case 'popupmenu'
                if ischar(val)
                    val=str2num(val)
                end
                set(handle, 'Value', val);
            case 'checkbox'
                set(handle, 'Value', val);
            case 'togglebutton'
                set(handle, 'Value', val);
            case 'radiobutton'
                set(handle, 'Value', val);
            otherwise
                disp(['setGUIValue: Style not implemented ' get(handle, 'Style')]);
        end
        
    case 'uimenu'
        if val
            set(handle,'Checked','on');
        else
            set(handle,'Checked','off');
        end
        
    otherwise
        assert(false);
end

