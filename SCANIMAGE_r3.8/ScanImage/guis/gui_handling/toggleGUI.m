function toggleGUI(h,mode,exclude)
%TOGGLEGUI Enables/disables all components of a given GUI (except for those listed in 'exclude').

    if nargin < 3 || isempty(exclude)
        exclude = -1;
	elseif iscell(exclude)
		exclude = cell2mat(exclude);
	end

    % ensure case-insensitivy
    if strcmpi(mode,'on')
        mode = 'On';
    elseif strcmpi(mode,'off')
        mode = 'Off';
    end

    for child = get(h, 'children')'
        if ~ismember(child,exclude)
            if strcmp(get(child,'Type'),'uipanel')
                toggleUIPanel(child,mode,exclude);
            else
                set(child,'Enable',mode);
            end
        end
    end

end
