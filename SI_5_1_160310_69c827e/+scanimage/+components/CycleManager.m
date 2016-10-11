classdef CycleManager < scanimage.interfaces.Component
% CYCLEMANAGER Model class for cycle mode

    properties (SetObservable)
        %active                      % Logical indicating if cycle mode is currently running
        cycleDataGroup              % Struct array for setting up cycle iteration properties
        enabled                     % Logical for enabling/disabling cycle mode

        cycleIdxTotal               % Integer containing the total number of cycles +++
    end

    properties (SetObservable, SetAccess=private)
        cycleIdxDone                % Integer containing the current cycle index 
        cycleIterIdxDone            % Integer containing the current cycle iterations index
    end

    properties (SetObservable, SetAccess=private, Dependent)
        cycleIterIdxTotal           % Integer containing the total number of cycle iterations
    end

    %%% ABSTRACT PROPERTY REALIZATION (most.Model)
    properties (Hidden, SetAccess=protected)
        mdlPropAttributes = ziniInitPropAttributes();
        mdlHeaderExcludeProps = {};
    end

    %%% ABSTRACT PROPERTY REALIZATION (scanimage.interfaces.Component)
    properties (SetAccess = protected, Hidden)
        numInstances = 0;
    end

    properties (Constant, Hidden)
        COMPONENT_NAME = 'CycleManager';                        % [char array] short name describing functionality of component e.g. 'Beams' or 'FastZ'
        PROP_TRUE_LIVE_UPDATE = {};                         % Cell array of strings specifying properties that can be set while the component is active
        PROP_FOCUS_TRUE_LIVE_UPDATE = {...                  % Cell array of strings specifying properties that can be set while focusing
            };
        DENY_PROP_LIVE_UPDATE = {};                         % Cell array of strings specifying properties for which a live update is denied (during acqState = Focus)
        
        FUNC_TRUE_LIVE_EXECUTION = {};                      % Cell array of strings specifying functions that can be executed while the component is active
        FUNC_FOCUS_TRUE_LIVE_EXECUTION = {};                % Cell array of strings specifying functions that can be executed while focusing
        DENY_FUNC_LIVE_EXECUTION = {};                      % Cell array of strings specifying functions for which a live execution is denied (during acqState = Focus)
    end

    %% LIFECYCLE
    methods
        function obj = CycleManager(hSI)
            obj = obj@scanimage.interfaces.Component(hSI,false,true);
            try
                obj.reset();
                obj.numInstances = 1;
            catch ME
                obj.numInstances = 0;
                most.idioms.warn('CycleManager module initialization failed. Error:\n%s', ME.message);
            end
        end

        %function delete(obj)
            %+++ This should probably be resolved
            %%most.idioms.safeDeleteObj(obj.cycleDataGroup);            
        %end
    end

    methods
        function reset(obj)
            obj.active = false;
            obj.cycleDataGroup = scanimage.components.cycles.CycleDataGroup;
            obj.enabled = false;

            obj.cycleIdxDone = 0;  
            obj.cycleIdxTotal = 1;  
            obj.cycleIterIdxDone = 0;
        end

        function refresh(obj)
            obj.cycleDataGroup.refresh();
            obj.active = obj.active;
            obj.enabled = obj.enabled; 
            obj.cycleIdxTotal = obj.cycleIdxTotal;

            obj.cycleIdxDone = obj.cycleIdxDone;
            obj.cycleIterIdxDone = obj.cycleIterIdxDone;
        end

        function resetCyclesCounter(obj)
            obj.cycleIdxDone = 0;   
        end

        function resetIterationsCounter(obj)
            obj.cycleIterIdxDone = 0;
        end

        function resetCounters(obj)
            obj.resetCyclesCounter();
            obj.resetIterationsCounter();
        end

        function appendNewIteration(obj)
            newRow = scanimage.components.cycles.CycleData;
            obj.cycleDataGroup.add(newRow);
        end

        function removeLastIteration(obj)
            numTotalIters = numel(obj.cycleDataGroup.cycleIters);
            if numTotalIters > 0
                obj.cycleDataGroup.removeByIdx(numTotalIters);
            end
        end

        function [res, numIters, numCycles] = start(obj)
        %   Returns 
        %   res == true on success
        %   numIters, number of successful iterations in latest cycle
        %   numCycles, number of successful cycles
        %
            numCycles = 0;
            numIters = 0;
            res = false;
            if obj.enabled
                obj.active = true;

                % PREPARE CYCLE

                % Save current ScanImage state
                if obj.cycleDataGroup.restoreOriginalCFGEnabled
                    obj.hSI.hConfigurationSaver.cfgSaveConfigAs('sitemp_cyclemode_backup.cfg',true);
                end

                % Run through cycles
                initCycleIdx = obj.cycleIdxDone+1;
                for j =  initCycleIdx : obj.cycleIdxTotal
                    if ~obj.active
                        break;
                    end
                    % Run through iterations
                    initCycleIterIdx = obj.cycleIterIdxDone+1;
                    for i =  initCycleIterIdx : obj.cycleIterIdxTotal
                        if ~obj.active
                            % Cycle mode was aborted
                            break;
                        end
                        if obj.cycleDataGroup.goHomeAtCycleEndEnabled && j == obj.cycleIdxTotal && i == obj.cycleIterIdxTotal
                            % Enable return home for the last iteration of the last cycle
                            obj.hSI.hStackManager.stackReturnHome = obj.cycleDataGroup.goHomeAtCycleEndEnabled;
                        end

                        obj.cycleDataGroup.cycleIters(i).go(obj.hSI);
                        obj.cycleIterIdxDone = i;
                        numIters = i;
                    end
                    obj.cycleIdxDone = j;
                    numCycles = j;

                    % This will reset the iterations-counter 
                    %   if there is more than one cycle
                    %   or if autoResetMode is enabled
                    if obj.cycleDataGroup.autoResetModeEnabled || initCycleIdx ~= obj.cycleIdxTotal
                        obj.resetIterationsCounter();
                    end
                end

                if obj.cycleDataGroup.autoResetModeEnabled
                    obj.resetCounters();
                end

                % CLEAN-UP CYCLE
                % Revert to original state if enabled
                if obj.cycleDataGroup.restoreOriginalCFGEnabled
                    obj.hSI.hConfigurationSaver.cfgLoadConfig('sitemp_cyclemode_backup.cfg');
                    delete('sitemp_cyclemode_backup.cfg');
                end
            else
                disp('Cycle-Mode must be enabled to run');
                return;
            end
            res = true;
        end

        function abort(obj)
            try
                obj.active = false;
            catch ME
                % convert hard error into soft error
                most.idioms.reportError(ME);
            end
        end


        function saveCycle(obj,filename)
        %
            if nargin < 2 || isempty(filename)
                [filename,pathname] = uiputfile('.cyc','Choose filename to save cycle','cycle.cyc');
                if filename==0;return;end
                filename = fullfile(pathname,filename);
            end
            %cycleDataGroup = obj.cycleDataGroup; 
            cycleDataGroup = copy(obj.cycleDataGroup); 
            cycleDataGroup.name = filename; 
            save(filename,'cycleDataGroup','-mat');

            % Update cycle-name field
            obj.cycleDataGroup.name = filename; 
        end


        function loadCycle(obj,filename)
            if nargin < 2 || isempty(filename)
                [filename,pathname] = uigetfile('.cyc','Choose file to load cycle','cycle.cyc');
                if filename==0;return;end
                filename = fullfile(pathname,filename);
            end
            
            cycleDataGroupContainer = load(filename,'-mat','cycleDataGroup');
            cycleDataGroup = cycleDataGroupContainer.cycleDataGroup;

            %obj.cycleDataGroup = copy(cycleDataGroup);
            obj.cycleDataGroup.update(cycleDataGroup);

            %obj.cycleDataGroup.name = filename;    %This should be unnecessary

            %perform a refresh
            obj.refresh();
        end

        function saveCycleCycFmt(obj,cycFilename)
        %   Version of saving cycles that will require proper parsing
        %   NOTE: We'll leave this unused until other features have been properly tested
        %
            fileID = fopen(cycFilename,'w'); 
            % NOTE:
            % In this new implementation cycleLength is not really necessary (for now)
            % but should serve to check if the file is correct
            cycHeader = {   'cycleName',...
                            'cycleLength',...
                            'numCycleRepeats',...
                            'returnHomeAtCycleEnd',...
                            'restoreOriginalConfig'};
            strValHeader = {    obj.cycleDataGroup.name,...
                                num2str(obj.cycleIdxTotal),...
                                num2str(obj.cycleIterIdxTotal),...
                                num2str(obj.cycleDataGroup.goHomeAtCycleEndEnabled),...
                                num2str(obj.cycleDataGroup.restoreOriginalCFGEnabled)...
                                };
            for i = 1:5
                fprintf(fileID,'%s = %s;\n',cycHeader{i},strValHeader{i});
            end

            %+++ It looks like it might be easier if we use the format used in SI TIFF files
            %    so we can do a quick eval
            for i = 1:obj.cycleIterIdxTotal
                currIter = obj.cycleDataGroup.getIterByIdx(i);
                fprintf(fileID,'iter%s =  {%s %s %s %s %s %s %s %s %s %s %s %s %s};\n',...
                    parseIterEntry(currIter.idx),...
                    parseIterEntry(currIter.cfgName),...
                    parseIterEntry(currIter.iterDelay),...
                    parseIterEntry(currIter.motorAction),...
                    parseIterEntry(currIter.motorStep),...
                    parseIterEntry(currIter.repeatPeriod),...
                    parseIterEntry(currIter.numRepeats),...
                    parseIterEntry(currIter.numSlices),...
                    parseIterEntry(currIter.zStepPerSlice),...
                    parseIterEntry(currIter.numFrames),...
                    parseIterEntry(currIter.power),...
                    parseIterEntry(currIter.numAvgFrames),...
                    parseIterEntry(currIter.framesPerFile),...
                    parseIterEntry(currIter.lockFramesPerFile)...
                );
            end
            fclose(fileID);

            %Subsequently there is one line per Cycle iteration, following general format
            %<iteration #> <columnName1> <columnVal1> <columnName2> <columnVal2> ....
            %i.e., one name/value pair, e.g. 'repeatPeriod 3', for each non-empty column for each iteration. Note that column names are specified with lower-case names using camel casing.

            function strVal = parseIterEntry(num)
            % if empty returns quotes
            %
                strVal = '''''';
                if isempty(num)
                    return;
                end

                if isa(num,'char')
                    strVal = ['''' num ''''];
                elseif isa(num,'double')
                    strVal = num2str(num);
                end
            end

        end

        function loadCycleCycFmt(obj,cycFilename)
        %   Version of loading cycles that will require proper parsing
        %   NOTE: We'll leave this unused until other features have been properly tested
        %
            %fid = fopen(cycFilename, 'r');
            %fileID = fopen('nums2.txt','r');

            %formatSpec = '%s';
            %header = textscan(fid,'%s',5,'Delimiter','\n')
            %header
            %header{1}(1)
            %header{1}(2)
            %header{1}(3)
            %header{1}(4)
            %header{1}(5)

            %strIter = textscan(fid,'%s',Inf,'Delimiter','\n');
            %strIter
            %strIter{1}(1)
            %strIter{1}(2)
            %strIter{1}(3)

            %fclose(fid);
            
            %cycHeader = {   'cycleName',...
                            %'cycleLength',...
                            %'numCycleRepeats',...
                            %'returnHomeAtCycleEnd',...
                            %'restoreOriginalConfig'};
            %obj.cycleDataGroup.cycleIters
            %obj.cycleDataGroup.name = getHeaderProperty(, 'cycleName');
            %obj.cycleDataGroup.goHomeAtCycleEndEnabled
            %obj.cycleDataGroup.autoResetModeEnabled
            %obj.cycleDataGroup.restoreOriginalCFGEnabled

            %+++ Make sure cycleLength matches the expected value

            function val = getHeaderProperty(imdescription,propfullname)
                try
                    str = regexpi(imdescription,sprintf('(?<=%s ?= ?).*$',propfullname),'match','once','lineanchors','dotexceptnewline');
                catch
                    str = [''''';'];
                end
                if isempty(str);
                    str = [''''';'];
                end
                val = eval(str);
            end
            %obj.cycleDataGroup = load(cycFilename,'-struct','-mat');
        end
    end

    %% INTERNAL METHODS
    methods (Access = protected)
        function componentStart(obj)
            %obj.refresh();
        end
        
        function componentAbort(obj)
        end
    end
        

    % PROPERTY ACCESS METHODS
    methods
        function val = get.cycleIterIdxTotal(obj)
            val = numel(obj.cycleDataGroup.cycleIters);
            % Side effects
            if obj.cycleIterIdxDone > 1 && obj.cycleIterIdxDone > val
                obj.cycleIterIdxDone = val;
            end
        end
    end

end

%% LOCAL (after classdef)
function s = ziniInitPropAttributes()
s = struct();

end


%--------------------------------------------------------------------------%
% CycleManager.m                                                           %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
