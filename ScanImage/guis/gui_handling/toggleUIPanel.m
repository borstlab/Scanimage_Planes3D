function toggleUIPanel(h,mode,exclude)
%TOGGLEUIPANEL Enables/disables all of a uipanel's children.
%   In lieu of an 'Enable' property for uipanel objects, this function will achieve equivalent functionality.

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

    if strcmp(get(h,'Type'),'uipanel')
        for panelChild = get(h,'children')'
            if ~ismember(panelChild,exclude)
                set(panelChild,'Enable',mode);
            end
        end
    end
end

