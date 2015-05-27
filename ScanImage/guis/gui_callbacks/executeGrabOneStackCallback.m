function executeGrabOneStackCallback(h)

% executeGrabOneCallback(h).m******
% In Main Controls, This function is executed when the Grab One or Abort button is pressed.
% It will on abort requeu the data appropriate for the configuration.
%
%% CHANGES
%   VI100608A: Defer motor velocity changes to setMotorPosition() -- Vijay Iyer 10/06/08
%   VI101008A: Defer updateRelativeMotorPosition() to setMotorPosition() -- Vijay Iyer 10/10/08
%   VI052809A: Determine Lz value for this stack, if override-Lz option is active -- Vijay Iyer 5/28/09
%   VI112309A: Only require end position if state.motor.stackEndpointsDominate is true -- Vijay Iyer 11/23/09
%   VI010610A: Update power to specified start power at beginning of stack acquisition -- Vijay Iyer 1/8/10
%   VI010810A: Display computed Lz value(s) to command line; abort if value for any beam is < 0 -- Vijay Iyer 1/8/10
%   VI010810B: Handle PowervsZEnable as a scalar now, but only apply for beams whose native Lz value is not Inf.  -- Vijay Iyer 1/8/10
%   VI010910A: Adjust power to match that recorded at stack start point, regardless of whether Override Lz is set or note -- Vijay Iyer 1/9/10
%   VI011210A: Store 'Inf' to state.init.eom.powerLzOverrideArray when value is 'Inf' in the main Lz array (state.init.eom.powerLzStoredArray) -- Vijay Iyer 1/12/10
%   VI032010A: setMotorPosition() renamed to motorSetPosition() -- Vijay Iyer 3/20/10
%   VI052010A: Use new 'verify' display option when moving to start stack position. -- Vijay Iyer 5/20/10
%   VI051211A: Handle secondary z controller XYZ-Z case -- Vijay Iyer 5/12/11
%   VI051911A: Use new stepDelay INI var, to avoid appearance of jitter at start of frame following move prior to stack start, due to motor 'settling' -- Vijay Iyer 5/18/11
%
%% CREDITS
% Written by: Thomas Pologruto and Bernardo Sabatini
% Cold Spring Harbor Labs
% January 26, 2001
%% *********************************

global state gh

if isempty(state.motor.stackStart)
    disp('*** Stack starting position not defined.');
    setStatusString('Need to set start');
    return
end

if isempty(state.motor.stackStop) && state.motor.stackEndpointsDominate %VI112309A
    disp('*** Stack ending position not defined.');
    setStatusString('Need to set end');
    return
end
val=get(gh.mainControls.grabOneButton, 'String');
visible=get(gh.mainControls.grabOneButton, 'Visible');

if strcmp(visible, 'off')
    return
end

if strcmp(val, 'GRAB')
    %%%VI052809A%%%%%%%%%
    if state.init.eom.pockelsOn && state.init.eom.powerVsZEnable %VI010910A %VI010810B
        
        if state.motor.dimensionsXYZZ && state.motor.motorZEnable  %VI051211A
            zPair = [state.motor.stackStart(4) state.motor.stackStop(4)]; %VI051211A
        else
            zPair = [state.motor.stackStart(3) state.motor.stackStop(3)];
        end
        
        for i=1:state.init.eom.numberOfBeams
            %Handle 'Override Lz' case where Lz is determined by powers recorded at start and endpoints
            if ~isinf(state.init.eom.powerLzStoredArray(i)) %VI010910A
                if state.init.eom.powerLzOverride %VI010810B
                    PPair = [state.init.eom.powerVsZStartArray(i) state.init.eom.powerVsZEndArray(i)];
                    LzValue = computePowerLz(zPair,PPair);
                    
                    beamName = state.init.eom.(['beamName' num2str(i)]);
                    
                    %%%VI010810A%%%%%%%
                    if LzValue <=0
                        fprintf(2,['Computed negative Lz value (' num2str(LzValue) ') for Beam ''' beamName ''', which is not allowed. Acquisition aborted.\n']);
                        abortCurrent(false);
                        setStatusString('Negative Lz Value!');
                        return;
                    else
                        disp(['Computed Lz = ' num2str(LzValue) ' um for Beam ''' beamName '''.']);
                        state.init.eom.powerLzOverrideArray(i) = LzValue;
                    end
                    %%%%%%%%%%%%%%%%%%%%
                end
                
                %VI010910: Set power to that recorded at stack start point, whether overriding Lz or not
                state.init.eom.maxPower(i) = state.init.eom.powerVsZStartArray(i); %VI010910A %VI010610A
            else %VI011210A
                state.init.eom.powerLzOverrideArray(i) = inf; %VI011210A
            end
        end
        ensureEomGuiStates(); %VI010610A
    end
    %%%%%%%%%%%%%%%%%%%%%
    
    %MP285SetVelocity(state.motor.velocityFast); %VI100608A
    motorSetPositionAbsolute(state.motor.stackStart,'verify'); %VI052010A %VI032010A
    %updateRelativeMotorPosition; %VI101008A
    pause(state.motor.stepDelay); %VI051911A
    
    executeGrabOneCallback(gh.mainControls.grabOneButton,'motorControlGrab');
    %MP285SetVelocity(state.motor.velocitySlow); %VI100608A
else
    executeGrabOneCallback(gh.mainControls.grabOneButton,'motorControlGrab');
end

