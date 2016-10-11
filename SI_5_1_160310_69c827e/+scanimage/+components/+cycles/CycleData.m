classdef CycleData < handle & matlab.mixin.Copyable
%% CYCLEDATA Data structure for the relevant "iteration" information for cycle-mode
% 
    properties (SetObservable)
        idx;            % Integer that allows CycleDataGroup to use CRUD operations on the current object
                        % IDs should always be numerically contiguous and integers. The CycleDataGroup
                        % should manage them to be 1-based
                        % This is for all intents and purposes an index in CycleDataGroup
                        % Perhaps we should make this read-only

        % Each property in CycleData can be empty, which "disables" it
        cfgName
        iterDelay
        motorAction
        motorStep
        repeatPeriod
        numRepeats
        numSlices
        zStepPerSlice
        numFrames
        power
        numAvgFrames
        framesPerFile
        lockFramesPerFile
    end
    
    properties(SetAccess = private,SetObservable)
        active;     % Iterations
    end
    
    methods
        function obj = CycleData()
            obj.reset();
        end      

        function reset(obj)
            obj.active = false;
            obj.idx = [];

            obj.cfgName = [];
            obj.iterDelay = [];
            obj.motorAction = [];
            obj.motorStep = [];
            obj.repeatPeriod = [];
            obj.numRepeats = [];
            obj.numSlices = [];
            obj.zStepPerSlice = [];
            obj.numFrames = [];
            obj.power = [];
            obj.numAvgFrames = [];
            obj.framesPerFile = [];
            obj.lockFramesPerFile = false;
        end

        function go(obj, hSI)
        %   Runs the current iteration
        %   NOTE: This is a blocks function. This prevents issues when multiple cycles are being called
        %         from a different script and simplifies usage
        %
            initGoTime = tic; 

            obj.active = true;
            if ~isempty(obj.cfgName)
                hSI.hConfigurationSaver.cfgLoadConfig(obj.cfgName);
            end

            if ~isempty(obj.motorAction) && ~isempty(obj.motorStep)
                if strcmp(obj.motorAction,'Posn #')
                    %1x3 or 1x4 array specifying motor position (in microns), depending on single vs dual motor, and motorDimensionConfiguration.
                    hSI.hMotor.motorPosition = obj.motorStep;
                elseif strcmp(obj.motorAction, 'ID #');
                    hSI.hMotor.gotoUserDefinedPosition(obj.motorStep);
                end
            end

            if ~isempty(obj.repeatPeriod)
                hSI.loopAcqInterval = obj.repeatPeriod;
            end

            if ~isempty(obj.numRepeats)
                hSI.acqsPerLoop = obj.numRepeats;
            end

            if ~isempty(obj.numSlices)
                hSI.hStackManager.numSlices = obj.numSlices;
            end

            if ~isempty(obj.zStepPerSlice)
                hSI.hStackManager.stackZStepSize = obj.zStepPerSlice;
            end

            if ~isempty(obj.numFrames)
                hSI.hStackManager.framesPerSlice = obj.numFrames;
            end

            %+++ 
            if ~isempty(obj.power)
                disp('NOTE: Assuming first beam for now. FIX ME!');
                %hSI.hBeams.powers(1) = obj.power;
            end

            if ~isempty(obj.numAvgFrames)
                hSI.hScan2D.logAverageFactor = obj.numAvgFrames;
            end

            if ~isempty(obj.framesPerFile)
                hSI.hScan2D.logFramesPerFile = obj.framesPerFile;
            end

            % NOTE: Since this is a checkbox we don't have the option to not override the default parameters on an empty value
            hSI.hScan2D.logFramesPerFileLock = obj.lockFramesPerFile;

            % Runs the iteration
            delay = 0.003;  
            %+++ We might want to abort on this case
            while ~strcmpi(hSI.acqState,'idle')
                pause(delay);
            end

            if ~isempty(obj.iterDelay)
                pause(obj.iterDelay - toc(initGoTime));
            end

            hSI.startLoop();

            %TODO
            %+++ Use a programmatic user function?
            %% Use polling for now to block execution
            while hSI.active   
                pause(delay);
            end

            %hSI.abort();    % This seems to be necessary for now, since hSI doesn't clean itself up correctly
                            % In some corner cases. The CycleData testcase will fail if this isn't manually closed
            % The above workaround seems to be related to the known issue SIDEV-267. +++ Must update in SI.m
            %Stop the loop repeat timer.
            stop(hSI.hLoopRepeatTimer);

            % Leave function and unset active flag
            obj.active = false;
        end

        function update(obj, cycleData)
            obj.cfgName           = cycleData.cfgName;
            obj.iterDelay         = cycleData.iterDelay;
            obj.motorAction       = cycleData.motorAction;
            obj.motorStep         = cycleData.motorStep;
            obj.repeatPeriod      = cycleData.repeatPeriod;
            obj.numRepeats        = cycleData.numRepeats;
            obj.numSlices         = cycleData.numSlices;
            obj.zStepPerSlice     = cycleData.zStepPerSlice;
            obj.numFrames         = cycleData.numFrames;
            obj.power             = cycleData.power;
            obj.numAvgFrames      = cycleData.numAvgFrames;
            obj.framesPerFile     = cycleData.framesPerFile;
            obj.lockFramesPerFile = cycleData.lockFramesPerFile;
        end
    end            
end


%--------------------------------------------------------------------------%
% CycleData.m                                                              %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
