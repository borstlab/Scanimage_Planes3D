% --------------------------------------------------------------------
% pre - None.
% post - Calibrated.
%        <ensureState>
function eomGuiCalibrate(varargin);
global state;
global gh;

    if ~state.init.eom.pockelsOn %VI011609A
        return;
    end
    
    if strcmp(get(gh.mainControls.focusButton, 'String'), 'Abort')
        fprintf(2, 'Can not calibrate Pockels cell while focusing.\n');
        return;
    elseif strcmp(get(gh.mainControls.startLoopButton, 'String'), 'Abort')
        fprintf(2, 'Can not calibrate Pockels cell while acquiring.\n');
        return;
    elseif strcmp(get(gh.mainControls.grabOneButton, 'String'), 'Abort')
        fprintf(2, 'Can not calibrate Pockels cell while acquiring.\n');
        return;
    end

    %Disable monkeying with this while calibrating.
    enableEomGui(0);
    turnOffMenus;
    set(gh.mainControls.grabOneButton, 'Enable', 'Off');
    set(gh.mainControls.startLoopButton, 'Enable', 'Off');
    set(gh.mainControls.focusButton, 'Enable', 'Off');

    %Run the calibration function.
    try
        calibrateEom(get(gh.powerControl.beamMenu, 'Value'));
    catch
        fprintf(2, '\nError calibrating Pockels Cell: %s\n\n', lasterr);
        
        enableEomGui(1);
        turnOnMenus;
        set(gh.mainControls.grabOneButton, 'Enable', 'On');
        set(gh.mainControls.startLoopButton, 'Enable', 'On');
        set(gh.mainControls.focusButton, 'Enable', 'On');
    end

    %%%VI032311A: Removed%%%
    %     %See if it went okay.
    %     if ~is_calibration_valid
    %         disp(' ');
    %         disp('Calibration Invalid');
    %         disp(debug);
    %         disp(' ');
    %         beep;
    %     end
    %%%%%%%%%%%%%%%%%%%%%%%
    
    turnOnMenus;
    set(gh.mainControls.grabOneButton, 'Enable', 'On');
    set(gh.mainControls.startLoopButton, 'Enable', 'On');
    set(gh.mainControls.focusButton, 'Enable', 'On');
    
    state.init.eom.changed(get(gh.powerControl.beamMenu, 'Value')) = 1;
      
    ensureEomGuiStates;
    
    return;
    
% --------------------------------------------------------------------
% Make sure everything makes sense.
function valid = is_calibration_valid
global state;

valid = (state.init.eom.min(state.init.eom.beamMenu) > 0) & (state.init.eom.min(state.init.eom.beamMenu) < 99);

% --------------------------------------------------------------------
% Generate some useful info...
function msg = debug
global state gh
beam=get(gh.powerControl.beamMenu, 'Value');
msg = sprintf('\nmaxPower_Slider:\n Max=%2.0f\n Min=%2.0f\n Val=%2.0f\nmaxLimit_Slider:\n Max=%2.0f\n Min=%2.0f\n Val=%2.0f\neom.maxPower=%2.0f\neom.min=%2.0f\neom.maxLimit=%2.0f\n', ...
    get(gh.powerControl.maxPower_Slider, 'Max'), get(gh.powerControl.maxPower_Slider, 'Min'), get(gh.powerControl.maxPower_Slider, 'Value'), ...
    get(gh.powerControl.maxLimit_Slider, 'Max'), get(gh.powerControl.maxLimit_Slider, 'Min'), get(gh.powerControl.maxLimit_Slider, 'Value'), ...
    state.init.eom.maxPower(beam), state.init.eom.min(beam), state.init.eom.maxLimit(beam));
