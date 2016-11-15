classdef Control < scanimage.interfaces.Class
    properties (SetAccess = immutable)
        hLinScan;                             % handle of Scan2D, handle gracefully if empty
        hBeams;
    end
    
    properties (SetAccess = private)
        hAO;                                    % handle of AO Task for control of an analog-controlled X/Y scanner pair
        hAOSingleSample;                        % same as hAO - separate Task used for static control of scanner
        hAOSampClk;                             % handle of Ctr Task for sample clock generation for hAO
        
        beamShareGalvoDAQ = false;              % Indicates that beam control channels are on the galvo DAQ. Possible if galvo control DAQ has 4 output channels
        flagOutputNeedsUpdate = false;          % for 'live' acquisition output set to true to indicate that scan parameters (e.g. zoom) changed so that AO needs to be updated
        updatingOutputNow = false;              % protects the 'critical section' in obj.updateAnalogBufferAsync
        waveformLength;                         % length of the output waveform for one frame
        scannerOutputUnTransformed;             % scanner output before transformation (zoom,shift,multiplier)
        bufferUpdatingAsyncRetries = 0;
        samplesDone = 0;
    end
    
    properties (Dependent)
        active;                                 % (logical) true during an active output
        startTrigIn;                            % input terminal of the start trigger (e.g. 'PFI0'); if empty, triggering is disabled
    end
    
    properties
        scanXisFast = true;                     % Fast scanning done on X scanner (identified in MDF). If false, fast scanning done on Y scanner
        
        startTrigOut;                           % output terminal of the start trigger
        
        startTrigEdge = 'rising';               % trigger polarity for the start trigger. one of {'rising','falling'}
    end
    
    properties (Hidden)
        samplesWritten;
        samplesGenerated;
        framesGenerated;
        framesWritten;
        
        genSampClk = false;
    end
    
    %% Lifecycle
    methods
        function obj = Control(hLinScan)
            obj.hLinScan = hLinScan;
            obj.hBeams = hLinScan.hSI.hBeams;
            
            obj.ziniPrepareTasks();
        end
        
        function delete(obj)            
            most.idioms.safeDeleteObj(obj.hAO);
            most.idioms.safeDeleteObj(obj.hAOSingleSample);
            most.idioms.safeDeleteObj(obj.hAOSampClk);
        end
    end
    
    % Public Methods
    methods
        function start(obj)
            obj.assertNotActive('method:start');
            obj.hAO.abort(); % to prevent DAQmx error -200288
            if obj.genSampClk
                obj.hAOSampClk.abort();
            end
            
            % calculate output buffer
            if obj.beamShareGalvoDAQ
                obj.waveformLength = obj.hBeams.configureStreaming(obj.hAO.sampClkRate);
                obj.samplesGenerated = 0;
                obj.framesGenerated = 0;
                obj.samplesWritten = obj.hBeams.streamingBufferSamples;
                obj.framesWritten = obj.hBeams.streamingBufferFrames;
                waveformOutput = obj.calcJointBuffer(1,obj.hBeams.streamingBufferFrames);
                if obj.hBeams.streamingBuffer
                    obj.hAO.registerEveryNSamplesEvent(@obj.streamingBufferNSampCB,obj.hBeams.nSampCbN,false);
                    obj.hAO.set('writeRegenMode','DAQmx_Val_DoNotAllowRegen');
                else
                    obj.hAO.registerEveryNSamplesEvent([],[],false);
                    obj.hAO.set('writeRegenMode','DAQmx_Val_AllowRegen');
                end
            else
                waveformOutput  = obj.hLinScan.hSI.scannerAO.ao_volts.G; %%AS: Thats where the output is output
                obj.hAO.set('writeRegenMode','DAQmx_Val_AllowRegen');
                obj.waveformLength = size(waveformOutput,1);
            end
            assert(obj.waveformLength > 0, 'AO generation error. Scanner control waveform length is zero.');
            
            % configure sample mode
            if obj.hLinScan.framesPerAcq <= 0 || isinf(obj.hLinScan.framesPerAcq) || obj.hLinScan.trigNextStopEnable
                obj.hAO.sampQuantSampMode = 'DAQmx_Val_ContSamps';
            else
                obj.hAO.sampQuantSampMode = 'DAQmx_Val_FiniteSamps';
                obj.hAO.sampQuantSampPerChan = obj.waveformLength * obj.hLinScan.framesPerAcq;
            end
            
            % update output buffer
            obj.hAO.cfgOutputBuffer(obj.waveformLength);
            obj.hAO.writeRelativeTo = 'DAQmx_Val_FirstSample';
            obj.hAO.writeOffset = 0;
            obj.hAO.writeAnalogData(waveformOutput); %%AS: AO is written here 
            obj.hAO.start();
            if obj.genSampClk
                obj.hAOSampClk.start();
            end
        end
        
        function restart(obj)
            obj.assertNotActive('method:restart');
            obj.hAO.abort();
            if obj.genSampClk
                obj.hAOSampClk.abort();
            end
            
            if obj.flagOutputNeedsUpdate
                % cannot simply restart, obj.start instead to update AO buffer
                obj.start()
                return;
            end
            
            try
                obj.hAO.start();
                if obj.genSampClk
                    obj.hAOSampClk.start();
                end
            catch ME
                if ~isempty(strfind(ME.message, '200462'))
                    warning('Output buffer is empty. Cannot restart. Starting a new generation instead');
                    obj.start();
                else     
                    rethrow(ME);
                end
            end
        end
        
        function abort(obj)  
            try
                obj.hAO.abort();
                if obj.genSampClk
                    obj.hAOSampClk.abort();
                end
                obj.updatingOutputNow = false;
            catch ME
               most.idioms.reportError(ME);
            end
        end
        
       function parkOrPointLaser(obj,xyz)
            %   ParkOrPointLaser(): parks laser at mdf defined park location (vars state.acq.parkAngleX & state.acq.parkAngleY); closes shutter and turns off beam with Pockels Cell
            %   ParkOrPointLaser(xy): parks laser at user defined location xy, a 2 element vector of optical degree values
            obj.assertNotActive('parkOrPointLaser');
            
            if nargin < 2 || isempty(xyz)
                xyz = [obj.hLinScan.mdfData.scanParkAngleX obj.hLinScan.mdfData.scanParkAngleY obj.hLinScan.mdfData.scanParkAngleZ];
            else
                validateattributes(xyz,{'numeric'},{'vector','numel',3});
            end
            
            xyz = [xyz(:,1)*obj.hLinScan.mdfData.voltsPerOpticalDegreeX xyz(:,2)*obj.hLinScan.mdfData.voltsPerOpticalDegreeY xyz(:,3)*obj.hLinScan.mdfData.voltsPerMicronZ];

            obj.hAO.control('DAQmx_Val_Task_Unreserve');
            obj.hAOSingleSample.writeAnalogData(xyz, 0.2, true);
       end
       
       function centerScanner(obj)
           obj.hAO.control('DAQmx_Val_Task_Unreserve');
           obj.hAOSingleSample.writeAnalogData([0 0 0], 0.2, true);
       end
    end
    
    % Getter / Setter Methods for properties
    methods  
        function val = get.active(obj)
            val = ~obj.hAO.isTaskDoneQuiet();
        end
        
        function val = get.startTrigIn(obj)
            startTrigType = get(obj.hAO,'startTrigType');
            
            switch startTrigType
                case 'DAQmx_Val_None';
                    val = '';
                case 'DAQmx_Val_DigEdge';
                    val = get(obj.hAO,'digEdgeStartTrigSrc');
            end
        end
        
        function set.startTrigIn(obj,val)
            obj.assertNotActive('startTrigIn');
            
            switch obj.startTrigEdge
                case 'rising'
                    edge = 'DAQmx_Val_Rising';
                case 'falling'
                    edge = 'DAQmx_Val_Falling';
                otherwise
                    assert(false);
            end
            
            if isempty(val)
                obj.hAO.disableStartTrig();
            else
                obj.hAO.cfgDigEdgeStartTrig(val,edge);
            end
        end
        
        function set.startTrigEdge(obj,val)
            obj.assertNotActive('startTrigEdge');
            assert(ismember(val,{'rising','falling'}));
            obj.startTrigEdge = val;
            obj.startTrigIn = obj.startTrigIn;    
        end
        
        function set.startTrigOut(obj,val)
            obj.assertNotActive('startTrigOut');
            if ~isempty(obj.startTrigOut)
                % disconnect old output terminal
                hDaqSystem = dabs.ni.daqmx.System();
                hDaqSystem.tristateOutputTerm(obj.startTrigOut);
            end
            
            if ~isempty(val)
                % set the new route
                val = qualifyTerminal(val);
                obj.hAO.exportSignal('DAQmx_Val_StartTrigger',val);
            end
            
            obj.startTrigOut = val;
            
            function name = qualifyTerminal(name)
                if ~isempty(name) && isempty(strfind(name,'/'))
                   name = sprintf('/%s/%s',obj.hLinScan.mdfData.deviceNameGalvo,name); 
                end
            end
        end
    end
    
    methods (Hidden)
        function streamingBufferNSampCB(obj,~,~)
            obj.samplesGenerated = obj.samplesGenerated + obj.hBeams.nSampCbN;
            obj.framesGenerated = obj.samplesGenerated / obj.hBeams.frameSamps;
            obj.updateAnalogBufferAsync();
        end
        
        function ao = calcJointBuffer(obj, bufStartFrm, nFrames)
            if obj.hBeams.streamingBuffer
                bao = obj.hBeams.calcStreamingBuffer(bufStartFrm, nFrames);
                ao = [repmat(obj.hLinScan.hSI.scannerAO.ao_volts.G, nFrames, 1) bao];
            else
                if obj.hBeams.enablePowerBox && obj.hBeams.hasPowerBoxes
                    ao = [obj.hLinScan.hSI.scannerAO.ao_volts.G obj.hLinScan.hSI.scannerAO.ao_volts.Bpb];
                else
                    ao = [obj.hLinScan.hSI.scannerAO.ao_volts.G obj.hLinScan.hSI.scannerAO.ao_volts.B];
                end
            end
        end
        
        function updateAnalogBufferAsync(obj)
            if obj.updatingOutputNow
                obj.flagOutputNeedsUpdate = true;
                return
            end
            
            if obj.beamShareGalvoDAQ
                if obj.hBeams.streamingBuffer
                    framesToWrite = obj.hBeams.streamingBufferFrames + obj.framesGenerated - obj.framesWritten;
                    startFrame = obj.framesWritten + 1;
                    
                    obj.hAO.writeRelativeTo = 'DAQmx_Val_CurrWritePos';
                    obj.hAO.writeOffset = 0;
                    if framesToWrite > 0
                        waveformOutput = obj.calcJointBuffer(startFrame, framesToWrite);
                    end
                else
                    framesToWrite = 1;
                    waveformOutput = obj.calcJointBuffer(1, 1);
                    obj.hAO.writeRelativeTo = 'DAQmx_Val_FirstSample';
                    obj.hAO.writeOffset = 0;
                end
            else
                framesToWrite = 1;
                obj.hAO.writeRelativeTo = 'DAQmx_Val_FirstSample';
                obj.hAO.writeOffset = 0;
                waveformOutput  = obj.hLinScan.hSI.scannerAO.ao_volts.G;
                waveformLength_ = size(waveformOutput,1);
                assert(obj.waveformLength == waveformLength_, 'AO generation error. Size of waveforms have changed.');
            end
            
            obj.flagOutputNeedsUpdate = false;
            if framesToWrite > 0
                obj.hAO.writeAnalogDataAsync(waveformOutput,2,[],[],@obj.updateAnalogBufferAsyncCb); % task.writeAnalogData(writeData, timeout, autoStart, numSampsPerChan)
                obj.updatingOutputNow = true;
            end
        end
        
        function updateAnalogBufferAsyncCb(obj,~,evt)
            obj.updatingOutputNow = false; % this needs to be the first call in the function in case there are errors below

            if obj.beamShareGalvoDAQ && obj.hBeams.streamingBuffer
                obj.samplesWritten = obj.samplesWritten + evt.sampsWritten;
                obj.framesWritten = obj.samplesWritten / obj.hBeams.frameSamps;
            end
            
            if evt.status ~= 0 && evt.status ~= 200015 && obj.hLinScan.active
                fprintf(2,'Error updating scanner buffer: %s\n%s\n',evt.errorString,evt.extendedErrorInfo);
                
                if obj.bufferUpdatingAsyncRetries < 3 || obj.flagOutputNeedsUpdate
                    obj.bufferUpdatingAsyncRetries = obj.bufferUpdatingAsyncRetries + 1;
                    fprintf(2,'Scanimage will retry update...\n');
                    obj.updateAnalogBufferAsync();
                else
                    obj.bufferUpdatingAsyncRetries = 0;
                end
            else
                obj.bufferUpdatingAsyncRetries = 0;

                if obj.flagOutputNeedsUpdate
                    obj.updateAnalogBufferAsync();
                end
            end
        end
    end
    
    % Helper functions
    methods (Access = private)        
        function ziniPrepareTasks(obj)
            beamMDF = obj.hBeams.mdfData;
            linScanMDF = obj.hLinScan.mdfData;
            
            % initialize hAO & hAI tasks
            obj.hAO = most.util.safeCreateTask([obj.hLinScan.name '-ScannerOut']);
            obj.hAO.createAOVoltageChan(linScanMDF.deviceNameGalvo, linScanMDF.XMirrorChannelID, 'XMirrorChannel');
            obj.hAO.createAOVoltageChan(linScanMDF.deviceNameGalvo, linScanMDF.YMirrorChannelID, 'YMirrorChannel');
            obj.hAO.createAOVoltageChan(linScanMDF.deviceNameGalvo, linScanMDF.ZMirrorChannelID, 'ZMirrorChannel');
            
            % initialize extra AO channels for beams if they are on the same DAQ
            if ~isempty(linScanMDF.beamDaqID) && strcmp(beamMDF.beamDaqDevices{linScanMDF.beamDaqID}, linScanMDF.deviceNameGalvo)
                obj.beamShareGalvoDAQ = true;
                for i = 1:obj.hBeams.daqNumBeams(linScanMDF.beamDaqID)
                    obj.hAO.createAOVoltageChan(linScanMDF.deviceNameGalvo,beamMDF.beamDaqs(linScanMDF.beamDaqID).chanIDs(i),obj.hBeams.displayNames{obj.hBeams.daqBeamIDs{linScanMDF.beamDaqID}(i)});
                end
            end
            
            %create sample clock task if acq and ctrl are on same board but not aux board
            if strcmp(linScanMDF.deviceNameAcq,linScanMDF.deviceNameGalvo) && ~strcmp(linScanMDF.deviceNameGalvo,linScanMDF.deviceNameAux)
                obj.hAOSampClk = most.util.safeCreateTask([obj.hLinScan.name '-AOSampClk']);
                obj.hAOSampClk.createCOPulseChanFreq(linScanMDF.deviceNameGalvo, 1, [obj.hLinScan.name '-AOSampClkChan'], 500e3);
                obj.hAOSampClk.cfgImplicitTiming('DAQmx_Val_ContSamps');
                set(obj.hAOSampClk.channels(1),'ctrTimebaseSrc','ai/SampleClock');
            end
            
            % preliminary sample rate
            obj.hAO.cfgSampClkTiming(obj.hAO.get('sampClkMaxRate'), 'DAQmx_Val_FiniteSamps');
            get(obj.hAO, 'writeRelativeTo');
            get(obj.hAO, 'writeOffset');
            
            % unbuffered task
            obj.hAOSingleSample = most.util.safeCreateTask([obj.hLinScan.name '-ScannerOutSingleSample']);
            obj.hAOSingleSample.createAOVoltageChan(obj.hLinScan.mdfData.deviceNameGalvo, obj.hLinScan.mdfData.XMirrorChannelID,'XMirrorChannel');
            obj.hAOSingleSample.createAOVoltageChan(obj.hLinScan.mdfData.deviceNameGalvo, obj.hLinScan.mdfData.YMirrorChannelID, 'YMirrorChannel');
            obj.hAOSingleSample.createAOVoltageChan(obj.hLinScan.mdfData.deviceNameGalvo, obj.hLinScan.mdfData.ZMirrorChannelID, 'ZMirrorChannel');
        end
        

        function assertNotActive(obj,propName)
            assert(~obj.active,'Cannot access %s during an active acquisition',propName);
        end
    end
end


%--------------------------------------------------------------------------%
% Control.m                                                                %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
