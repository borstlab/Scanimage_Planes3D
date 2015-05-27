function verifyEomConfig
%% function verifyEomConfig
%   Verifies/constrains EOM settings when settings are loaded/applied
%
%% CHANGES
%   VI020909A: Actually fill in empty array elements as was apparently originally intended -- Vijay Iyer 02/09/09
%   VI020909B: Removed obsolete powerBoxNormCoodString -- Vijay Iyer 02/09/09
%% ***************************************************

global state gh;

if ~state.init.eom.pockelsOn %VI011609A
    return;
end

try
    %TO051804b - Aleksander's rig keeps saving this as a 'char'.
    if strcmpi(class(state.init.eom.uncagingMapper.enabled), 'char')
        state.init.eom.uncagingMapper.enabled = zeros(state.init.eom.numberOfBeams, 1);
    end
    if length(state.init.eom.min) < state.init.eom.numberOfBeams
        state.init.eom.min(end+1:state.init.eom.numberOfBeams) = 1; %VI020909A
    end
    if length(state.init.eom.maxPower) < state.init.eom.numberOfBeams
        state.init.eom.maxPower(end+1:state.init.eom.numberOfBeams) = 0; %VI020909A
    end
    if length(state.init.eom.maxLimit) < state.init.eom.numberOfBeams
        state.init.eom.maxLimit(end+1:state.init.eom.numberOfBeams) = 0; %VI020909A
    end
    if length(state.init.eom.changed) < state.init.eom.numberOfBeams
        state.init.eom.changed(end+1:state.init.eom.numberOfBeams) = 1; %VI020909A
    end
    if length(state.init.eom.showBoxArray) < state.init.eom.numberOfBeams
        state.init.eom.showBoxArray(end+1:state.init.eom.numberOfBeams) = 0; %VI020909A
    end
    if length(state.init.eom.boxPowerArray) < state.init.eom.numberOfBeams
        state.init.eom.boxPowerArray(end+1:state.init.eom.numberOfBeams) = 0; %VI020909A
    end
    if length(state.init.eom.startFrameArray) < state.init.eom.numberOfBeams
        state.init.eom.startFrameArray(end+1:state.init.eom.numberOfBeams) = 0; %VI020909A
    end
    if length(state.init.eom.endFrameArray) < state.init.eom.numberOfBeams
        state.init.eom.endFrameArray(end+1:state.init.eom.numberOfBeams) = 0; %VI020909A
    end

    if length(state.init.eom.maxLimit) < state.init.eom.numberOfBeams
        state.init.eom.maxLimit(end+1:state.init.eom.numberOfBeams) = 0; %VI020909A
    end
    
    if ~strcmpi(class(state.init.eom.uncagingMapper.enabled), 'char')
        state.init.eom.uncagingMapper.enabled = zeros(1, state.init.eom.numberOfBeams);
    end
    for i = 1 : size(state.init.eom.uncagingMapper.pixels, 1)
        if state.init.eom.uncagingMapper.pixels(i, 1, :) == 1
            state.init.eom.uncagingMapper.enabled(i) = 0;
        end
    end
    if isempty(state.init.eom.uncagingMapper.pixels)
        state.init.eom.uncagingMapper.enabled(end+1:state.init.eom.numberOfBeams) = 0; %VI020909A
        %state.init.eom.uncagingMapper.enabled(:) = 0; %VI020909A
    end
    if ~isnumeric(state.init.eom.uncagingMapper.pixels)
        if state.init.eom.uncagingMapper.enabled
            fprintf(2, 'Warning: Found possibly corrupted value for state.init.eom.uncagingMapper.pixels.\n');
        end
        state.init.eom.uncagingMapper.pixels = [];
    end
    
    %Match the msPerLine values.
    updateGUIByGlobal('state.init.eom.uncagingPulseImporter.lineConversionFactor', 'Value', state.acq.msPerLine); %VI012109A
%     if state.init.eom.uncagingPulseImporter.lineConversionFactor == 1000 * state.acq.msPerLine
%         %The conversion factor makes sense with the current scan settings.
%         set(gh.uncagingPulseImporter.lineConversionFactorText, 'ForegroundColor', [0 0 0]);%Black    
%     else
%         %This is probably not the value that the user really wants.
%         set(gh.uncagingPulseImporter.lineConversionFactorText, 'ForegroundColor', [1 0 0]);%Red
%         fprintf(2, 'Warning: The ''ms / line'' conversion factor in the UncagingPulseImporter does not match the current scan settings of %s ms/line.\n', ...
%             num2str(1000 * state.acq.msPerLine));
%     end
%     updateGUIByGlobal('state.init.eom.uncagingPulseImporter.pathnameText');
    updateGUIByGlobal('state.init.eom.uncagingPulseImporter.enabled', 'Value', 0, 'Callback', 1);
    %Clear all powerboxes.
    state.init.eom.showBoxArray(:) = 0;
    updateGUIByGlobal('state.init.eom.showBox', 'Value', 0);
    state.init.eom.powerBoxNormCoords = [];
    %state.init.eom.powerBoxNormCoordsString = '[]'; %VI020909B

    for i = 1 : prod(size(state.init.eom.boxHandles))
        try
            if state.init.eom.boxHandles(i) > 0 & ishandle(state.init.eom.boxHandles(i))
                set(state.init.eom.boxHandles(i), 'Visible', 'Off');
                delete(state.init.eom.boxHandles(i));
            end
        catch
            fprintf(2, 'Failed to clear powerbox graphic with handle: %s', num2str(state.init.eom.boxHandles(i)));
        end
    end    
    state.init.eom.boxHandles = [];
    
    ensureEomGuiStates;
catch
    warning(lasterr);
end

try
    if state.init.eom.uncagingMapper.perFrame
        set(gh.uncagingMapper.perFrameRadioButton, 'Enable', 'Inactive');
        set(gh.uncagingMapper.perGrabRadioButton, 'Enable', 'On');
    else
        set(gh.uncagingMapper.perFrameRadioButton, 'Enable', 'On');
        set(gh.uncagingMapper.perGrabRadioButton, 'Enable', 'Inactive');    
    end
catch
    warning(lasterr);
end

try
    feval(state.init.eom.laserFunctionPanel.updateDisplay);
catch
    warning('Failed to execute: %s\n  %s', func2str(state.init.eom.laserFunctionPanel.updateDisplay), lasterr);
end

return;