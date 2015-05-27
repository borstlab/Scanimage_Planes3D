function motorGoHome(flag)
%%function motorGoHome(flag,updatePositionDisplay)
%   Returns motor to cached initial position, e.g. following a stack or cycle acquisition
%% SYNTAX
%   flag: <Optional; one of {'cycleAbort' 'cycleHome'}> If supplied, signals this is a cycle abort or time for cycle go-home
%   updatePositionDisplay: (OPTIONAL - Default:true) Logical value indicating, if true, to read and update position display following completion of 
%
%% NOTES
%   Function was rewritten from scratch, and renamed to motorGoHome. To see earlier version, see executeGoHome.MOLD file. -- Vijay Iyer 10/30/09
%
%   This function consolidates return to one of three 'home' positions -- 
%   stack home, cycle home, and cycle iteration home
%
%   In general, it returns to stack home, unless flagged to go to iteration
%   or cycle home. If flagged to go to iteration home, it will do so if one
%   is stored and otherwise will go to the stack home. 
%
%   VVV060512A: At this time, updatePositionDisplay is not supplied by any known caller, i.e. it's always true. Could thus be removed...
%
%% CHANGES
%   VI010810A: If not returning home at end of stack acquisition, then should update current power levels, per-beam, to match those at stack's end
%   VI011210A: Ensure that final power state is rounded, even if last power during stack acquisition was non-integer-valued, in case where returnHome=false -- Vijay Iyer 1/12/10
%   SA031210A: Replace MP285RobustAction() calls with try/catch block for now -- Salvador Aguinaga 03/12/10
%   VI032010A: (Changes to use new LinearStageController class) setMotorPosition() renamed to motorSetPosition(); use motor object's error condition directly -- Vijay Iyer 3/20/10
%   VI040110A: Add updatePositionDisplay option and convert function to be a 'macro' of actions, rather than an action itself -- Vijay Iyer 4/1/10
%   VI040110B: Remove statusString display prior to moves, as they're immediately overwritten by StatusString for motorSetPosition(). Probably not worth slowing down to ensure display . -- Vijay Iyer 4/1/10
%   VI040510A: Do nothing if an existing motor error condition is present. At moment, provide no message -- this is appropriate for all current calls to motorGoHome(). If that changes in future, option coudld be added. -- Vijay Iyer 4/5/10
%   VI091610A: Pass-through if motor is inactive -- Vijay Iyer 9/16/10
%
%% CREDITS
%   Created 10/30/09, by Vijay Iyer
%   Based on version from ScanImage 3.0
%% ************************************************************

global state

%%%VI091610A
if ~state.motor.motorOn
    return;
end

%%%VI040510A
if motorErrorPending()
    return;
end

if nargin < 1 || isempty(flag)
    cycleHome = false;
    cycleAbort = false;
else
    cycleHome = strcmpi(flag,'cycleHome');
    cycleAbort = strcmpi(flag,'cycleAbort');
end

if cycleAbort && state.cycle.autoReset %Return to cycle home on abort if auto-reset is enabled
    cycleHome = true;
end

if nargin < 2 || isempty(updatePositionDisplay)
    updatePositionDisplay = true;
    setPosnMode = 'verify';
else
    setPosnMode = 'none';
end

if cycleHome && state.cycle.cycleOn && state.cycle.returnHomeAtCycleEnd 
    %Go-to cycle home
    if isempty(state.internal.cycleInitialMotorPosition)
        if state.cycle.autoReset %Cycle home is expected
            error('Cannot return to cycle home.  Cycle home not defined!');
        else %This will occur when Cycle is aborted and resumed at iteration where left off -- the Cycle home memory is lost 
            fprintf(2,'WARNING: Cycle home not defined - ');
            if ~isempty(state.internal.iterationHomePosn)
                fprintf(2,'returning to starting position of the final iteration instead.\n');
                gotoHome(state.internal.iterationHomePosn,setPosnMode);
            elseif ~isempty(state.internal.initialMotorPosition) && state.acq.returnHome
                fprintf(2,'returning to final stack home instead.\n');
                gotoHome(state.internal.initialMotorPosition,setPosnMode);
            else
                fprintf(2,'cannot return to home position.\n');
            end
        end
    else
        gotoHome(state.internal.cycleInitialMotorPosition,setPosnMode);
    end
          
elseif cycleAbort && state.cycle.cycleOn && ~isempty(state.internal.iterationHomePosn) 
    %Go-to iteration home
    gotoHome(state.internal.iterationHomePosn,setPosnMode);

elseif ~isempty(state.internal.initialMotorPosition) && state.acq.returnHome 
    %Go-to stack home %VI: Seems that idea is that state.acq.returnHome is a global setting, not per configuration/cycle-iteration. Should think about/revisit. -- 05/18/2011    
    gotoHome(state.internal.initialMotorPosition,setPosnMode);
else
    %No motor operation - just update position, if needed   
    if updatePositionDisplay && ~state.internal.looping
        motorGetPosition();
    end
end

% %%%VI040110A%%%%%%%
% if updatePositionDisplay
%     motorAction(@motorGetPosition,'Read final position following return to Cycle/Stack Home');
% end
% %%%%%%%%%%%%%%%%%%%

%%%VI010810A%%%%
if state.acq.numberOfZSlices > 1 && ~state.acq.returnHome
    for i=1:state.init.eom.numberOfBeams
        state.init.eom.maxPower(i) = round(computeScaledMaxPower(i)); %Updates max power %VI011210A: Ensure that final power value is rounded
        updateMaxPowerDisplay(i);
    end
end
%%%%%%%%%%%%%%%%

function gotoHome(homePosn,setPosnMode)

global state

doMove = false; 

motorGetPosition();

if any(abs(state.motor.lastPositionRead(1:3) - homePosn(1:3)) > state.motor.hMotor.resolutionBest)
    doMove = true;
end

if ~doMove && numel(state.motor.lastPositionRead) > 3
    assert(numel(homePosn) == 4);
    doMove = abs(state.motor.lastPositionRead(4) - homePosn(4)) > min(state.motor.hMotorZ.resolutionBest);
end
    
if doMove
    motorSetPositionAbsolute(homePosn,setPosnMode); %VI040110A %VI032010A
end

    
