function iterateCycle(first)
%% function iterateCycle(first)
%   Start first or subsequent Cycle iteration
%
%% SYNTAX
%   first: <OPTIONAL - LOGICAL> If true, indicates this is being called at start of cycle
%
%% NOTES
%   Each Cycle iteration is a LOOP acquisition consisting of one or more Repeats)
%
%   TODO: Handle state.cycle.timeDelay value, delaying start of first Repeat in next loop to start
%   TODO: Ensure that return to start of stack from last set of Repeats is done if no move done here
%
%% CHANGES
%   VI051911A: Allow for delay following motor moves as required to ensure motor 'settling' following ostensible move completion -- Vijay Iyer 5/19/11
%
%% CREDITS
%   Created 1/4/11, by Vijay Iyer
%% ************************************************************

global state gh

if nargin < 1 || isempty(first)
    first = false;
end

state.internal.iterationHomePosn = []; 

returnCycleHome = false; %Flag indicating whether to move motor to Cycle Home at end of this iteration
returnStackHome = state.acq.numberOfZSlices > 1 && state.acq.returnHome; %Flag indicating whether to move motor to stack Home at end of this iteration

if ~first
    if isinf(state.cycle.iterationsPerLoop) && state.cycle.iteration == state.cycle.cycleLength
        %Finished one cycle -- terminate LOOP or reset to iteration 1
        if finishCycle()
            return;
        else
            returnCycleHome = true;
        end
    else        
        state.cycle.iteration = state.cycle.iteration + 1;
        if state.cycle.iteration > state.cycle.cycleLength
            %Finished one cycle -- terminate LOOP or reset to iteration 1
            if finishCycle()
                return;
            else
                returnCycleHome = true;
            end
        end
        
        updateGUIByGlobal('state.cycle.iteration');
        
        if ~isinf(state.cycle.iterationsPerLoop)
            state.cycle.iterationsPerLoopCounter = state.cycle.iterationsPerLoopCounter + 1;
            
            if state.cycle.iterationsPerLoopCounter == state.cycle.iterationsPerLoop
                endLoopMode(); %Will /not/ return to motor home
                return;
            end
        end
    end
end

resetCounters(1); %Reset all counters now, also before updating # frames, etc.
resetAcqBuffer();

%Load configuration
newConfigPath = getCycleVar('configPath');
newConfigName = getCycleVar('configName');
if  ~isempty(newConfigPath) && ~isempty(newConfigName) && (~strcmp(newConfigPath,state.configPath) || ~strcmp(newConfigName,state.configName));
    cfgFullFile = fullfile(newConfigPath,[newConfigName '.cfg']);
    loadCachedConfiguration(cfgFullFile);
end

% Update acq vars, and GUI elements
cyclableAcqVars = {'repeatPeriod' 'numberOfRepeats' 'numberOfZSlices' 'zStepSize' 'numberOfFrames' 'numAvgFramesSave'};
for acqVar = cyclableAcqVars
    acqVar = acqVar{:};
    val = getCycleVar(acqVar);
    if ~isempty(val)
        % cache if necessary
        if ~state.internal.preCycleCache.isKey(acqVar)
            state.internal.preCycleCache(acqVar) = state.acq.(acqVar);
        end
        state.acq.(acqVar) = val;
        updateGUIByGlobal(['state.acq.' acqVar],'Callback',true);
    elseif ~first && state.internal.preCycleCache.isKey(acqVar)
        state.acq.(acqVar) = state.internal.preCycleCache(acqVar);
        updateGUIByGlobal(['state.acq.' acqVar],'Callback',true);
    end
end

% update any user-defined cycle vars
for var = state.cycle.cycleTableColumnsUserAdded
    var = var{:};
    val = getCycleVar(strrep(var,'.','DOT'));
    if ~isempty(val)
        if isnumeric(val)
            val = num2str(val);
        end
        eval([var ' = ' val ';']);
        updateGUIByGlobal(var,'Callback',true);
    end
end

%Do stage motion, if needed
if ~isempty(getCycleVar('motorActionID'))
    actionID = getCycleVar('motorActionID');
    motorActionType = getCycleVar('motorAction');
    
    storeIterationHome = ~state.cycle.autoReset;
    
    %Store start position for the iteration about to be executed, if needed
    if storeIterationHome && ismember(motorActionType,{'Posn #' 'ROI #'})
        if ~first && ~isempty(state.internal.iterationHomePosnLast)
            state.internal.iterationHomePosn = state.internal.iterationHomePosnLast; %the start position of the /previous/ iteration
        elseif state.cycle.iteration == 2 && ~isempty(state.internal.cycleInitialMotorPosition)
            state.internal.iterationHomePosn = state.internal.cycleInitialMotorPosition; %the start position of iteration # 1            
        elseif ~first && ~isempty(state.internal.initialMotorPosition) && state.acq.returnHome
            state.internal.iterationHomePosn = state.internal.initialMotorPosition; %the stack home of the /previous/ iteration
        else
            state.internal.iterationHomePosn = motorGetPosition();
        end        
    end
    
    switch motorActionType
        case 'Posn #'
            state.hSI.roiGotoPosition(actionID);
        case 'ROI #'
            state.hSI.roiGotoROI(actionID,true);
        case 'Step'
            stepPos = actionID;
            
            if length(stepPos) == 1
                % pad a z-step to a 3 or 4-vector
                if state.motor.motorZOn
                    stepPos = [0 0 stepPos 0];
                else
                    stepPos = [0 0 stepPos];
                end
            end
            
            if ~first && returnCycleHome
                % If this is final iteration of a Cycle iteration-set, combine cycle go-home operation with motor step
                
                combinedMovePosn = state.internal.cycleInitialMotorPosition + stepPos;
                motorSetPositionAbsolute(combinedMovePosn,'assume');
                most.idioms.pauseTight(state.motor.stepDelay); %VI051911A
                
            elseif ~first && ~isempty(state.internal.initialMotorPosition) && state.acq.returnHome
                %If last iteration contained a stack, combine stack go-home operation with motor step
                
                combinedMovePosn = state.internal.initialMotorPosition + stepPos;
                motorSetPositionAbsolute(combinedMovePosn,'assume');
                most.idioms.pauseTight(state.motor.stepDelay); %VI051911A
                
            else
                % otherwise, just do the motor step                
                               
                state.motor.absXPosition = state.motor.absXPosition + stepPos(1);
                state.motor.absYPosition = state.motor.absYPosition + stepPos(2);
                state.motor.absZPosition = state.motor.absZPosition + stepPos(3);
                if numel(stepPos) > 3
                    state.motor.absZZPosition = state.motor.absZZPosition + stepPos(4);
                end
                
                motorSetPositionAbsolute([],'assume');
                most.idioms.pauseTight(state.motor.stepDelay); %VI051911A
                motorUpdatePositionDisplay(); %Need this call; motorSetPositionAbsolute() does not call this when using state var specification, without 'verify' option
            end

            %Store position just set (except if there's a stack without
            %return home mode)--this is starting posn for /next/ iteration
            if storeIterationHome && (state.acq.returnHome || state.acq.numberOfZSlices == 1 )
                state.internal.iterationHomePosnLast = state.motor.lastPositionSet;
            else
                state.internal.iterationHomePosnLast = [];
            end            
    end                       
else
    %Return to cycle or stack home, as appropriate
    
    if returnCycleHome  %Return to cycle home (if stored)
        motorGoHome('cycleHome');
        most.idioms.pauseTight(state.motor.stepDelay); %VI051911A
    elseif ~first && returnStackHome %Return to stack home
        motorGoHome();
        most.idioms.pauseTight(state.motor.stepDelay); %VI051911A
    end
end

% handle power
state.init.eom.stackPowerScaling = ones(state.init.eom.numberOfBeams,1);
powerVal = getCycleVar('power');
if ~isempty(powerVal)
    if length(powerVal) > state.init.eom.numberOfBeams
        disp('Invalid power parameters given: exceeds length of ''state.init.eom.numberOfBeams''');
    else
        for i=1:length(powerVal)
            if isnan(powerVal(i))
                continue;
            end
            state.init.eom.maxPower(i) = powerVal(i);
            updateMaxPowerDisplay(i,powerVal(i));
        end
    end
end

%Update AO signals if needed
if state.internal.updatedZoomOrRot || ~isempty(powerVal) % need to reput the data with the approprite rotation and zoom.    
    linTransformMirrorData(); %VI010809A
    flushAOData;
    state.internal.updatedZoomOrRot=0;
end

%Start next set of loop Repeats
try
    state.cycle.iterationDelay = 0;
    if ~first        
        state.cycle.cycling = 1; %Signal a cycle iteration transition is occurring
        
        state.cycle.iterationDelay = getCycleVar('iterationDelay');
        %%%VI030811A%%%%
        if isempty(state.cycle.iterationDelay)
            state.cycle.iterationDelay = 0;
        end
        %%%%%%%%%%%%%%%%
        %updateCountdownTimer(state.cycle.iterationDelay);
        waitForLoopRepeat(state.cycle.iterationDelay, false); %Don't warn if delay is too short..just start!
    end 
    
    if state.internal.looping && ~state.internal.loopPaused    
        notify(state.hSI,'cycleIterating');
        initializeLoop();
        state.cycle.cycling = 0;
    end       
catch ME
    state.cycle.cycling = 0;
    ME.rethrow();
end



end

function done = finishCycle()
%Helper for the end of a set of cycle iterations -- either end LOOP or prepare to cycle again

global state;

state.cycle.iteration = 1; %Reset cycle index
updateGUIByGlobal('state.cycle.iteration');

state.cycle.cycleCount = state.cycle.cycleCount + 1; %Advance cycle count
setStatusString(['Cycle # ' num2str(state.cycle.cycleCount) ' Done']);
updateGUIByGlobal('state.cycle.cycleCount'); %VI030811A

if state.cycle.cycleCount >= state.cycle.numCycleRepeats
    
    % reset any changed/cached cycleable parameters
    for key = state.internal.preCycleCache.keys()
        state.acq.(key{1}) = state.internal.preCycleCache(key{1});
        updateGUIByGlobal(['state.acq.' key{1}],'Callback',true);
    end
    
    % reset our mainControls counters
    counterNames = {'repeatCounter' 'zSliceCounter' 'frameCounter'};
    for counterName = counterNames
        state.internal.(counterName{1}) = 0;
        updateGUIByGlobal(['state.internal.' counterName{1}]);
    end
    
    pause(0.6); %Allow final 'Cycle # Done' status string to linger a bit
    endLoopMode('cycleHome');   %VI030811A: Signal to go to cycle home, if appropriate
    
    state.cycle.cycleCount = 0;
    updateGUIByGlobal('state.cycle.cycleCount');
    
    state.cycle.iterationsPerLoopCounter = 0;
    
    done = true;
else
    done = false;
end

end

