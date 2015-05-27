function [eom_max, eom_min, avg_dev] = calibrateEom(beams)
%% function [eom_max, eom_min, avg_dev] = calibrateEom(beams);
%  Construct an array of output voltages that correspond to attenuations in laser intensity via a Pockels cell.
%
%% SYNTAX
%  The return values are:
%    eom_max - the maximum voltage input measured.
%    eom_min - the minimum voltage input measured.
%    avg_dev - the average standard deviation over all measurements.
%
%% NOTES
%  The array is stored in state.eom.lut. The index into the array reflects the percentage of the maximum possible intensity. The resolution is 1%.
%
%  This version was rewritten from scratch (again) as part of adopting new DAQmx interface. -- Vijay Iyer 9/9/09
%
%% CHANGES
%   VI091309A: Handle case where some or all of the photodiode input channels are on the 'primary' board (the one with PMT inputs) -- Vijay Iyer 9/13/09
%   VI122909A: Unreserve state.init.eom.hAO Task following calibration to allow power level to be set (to either max or min, depending on directMode and other status) following calibration -- Vijay Iyer 12/29/09
%   VI062410A: Photodiode analog input channels for each beam are now separate Tasks -- Vijay Iyer 6/24/10
%   VI062410B: The trigger source for photodiode input channels/tasks is now pre-configured in setupAIObjects_Common(), so remove from here -- Vijay Iyer 6/24/10
%   VI062510A: Bugfixes in section handling multiple beams -- Vijay Iyer 6/25/10
%   VI111010A: Add delay between calibration sweeps (allows for case where slow decay in transmission is seen after reaching high voltages) -- Vijay Iyer 11/10/10
%   VI111010B: Open shutter if (new) state.shuter.shutterBeforeEOM is true -- Vijay Iyer 11/10/10
%   VI112410A: Wait for completion of photodiode Task, not Pockels Task, before trying to reading photodiode data  -- Vijay Iyer 11/24/10
%   VI112410B: Ensure that AO buffer length is even, to avoid DAQmx error -200692 with some devices (e.g. AO series) -- Vijay Iyer 11/24/10
%   VI032311A: Handle case where photodiode voltage signal is inverted -- Vijay Iyer 3/23/11
%   VI040511A: React to changes in openShutter() behavior -- ensure that shutter is physically open before triggering acquisition -- Vijay Iyer 4/15/11
%   VI081011A: Incorporate changes submitted by Dana Hod to fix handling of rejected-light case -- Vijay Iyer 8/10/11
%   
%% CREDITS
%  Created 9/9/09, by Vijay Iyer
%  Based very heavily on original version by Tim O'Connor, 5/13/03
%% ************************************************************

global state;

if nargin < 1
    beams = 1:state.init.eom.numberOfBeams;
end    

%Calibrate each beam individually
for i=1:length(beams)
    calibrateBeam(beams(i));
    
    %Don't calibrate others when calibration is cancelled.
    if state.init.eom.cancel
        return;
    end
end

%Ready beams for actual use afterwards
stopCalibration();

return;


    function calibrateBeam(beam)
        %Get pockels voltage range for this beam
        pockelsVoltageRange = state.init.eom.(['pockelsVoltageRange' num2str(beam)]);
        
        %Handle cases where naive/non calibration is required
        if dummyEOMCalibrate(beam)
            markCalibration(beam); %mark it as calibrated, though it's not really
            
            eom_min = state.init.eom.min(beam);
            eom_max = state.init.eom.maxPhotodiodeVoltage(beam);
            avg_dev = 0;
            return;
        end
              
        
        %Proceed with actual calibration
        wb = waitbar(0, sprintf('Calibrating Pockels Cell #%s...', num2str(beam)), 'Name', 'Calibrating...', ...
            'createCancelBtn', 'global state; state.init.eom.cancel = 1; delete(gcf);');
        state.init.eom.cancel = 0;
        
        %Create array of modulation voltages, sampling more densely at lower end of range
        modulation_voltage = [0:(state.internal.eom.calibration_interval/10):(pockelsVoltageRange/10) (pockelsVoltageRange/10):state.internal.eom.calibration_interval:pockelsVoltageRange 0]'; %VI111010A
        %%%VI112410B
        if rem(length(modulation_voltage),2)
            modulation_voltage(end+1) = modulation_voltage(end);
        end
            
        photodiode_voltage = zeros(length(modulation_voltage), 1);
        state.init.eom.lut(beam, :) = zeros(100, 1);
        
        %Prepare AO & AI Tasks (Beam-Independent)
        state.init.eom.hAO.cfgSampClkTiming( state.internal.eom.calibrationSampleRate, 'DAQmx_Val_FiniteSamps',length(modulation_voltage));  %No repeats
        state.init.hAIPhotodiode{beam}.set('sampQuantSampPerChan',length(modulation_voltage)); %VI062410A
        setTriggerSource(state.init.eom.hAO,true); %VI062410B %Forces internal triggering
        
        %Prepare AO & AI Tasks (Beam-Dependent)
        outputDataBuf = zeros(length(modulation_voltage), state.init.eom.numberOfBeams);
        for beamIdx=1:state.init.eom.numberOfBeams %VI062510A
            if beamIdx == beam
                outputDataBuf(:,beamIdx) = modulation_voltage;
            else
                %%%VI062510A%%%
                if size(state.init.eom.lut,1) >= beamIdx 
                    outputDataBuf(:,beamIdx) = state.init.eom.lut(beamIdx, state.init.eom.min(beamIdx)); 
                else
                    outputDataBuf(:,beamIdx) = 0; %If not yet calibrated, output 0
                end
                %%%%%%%%%%%%%%%%
            end
        end
        state.init.eom.hAO.writeAnalogData(outputDataBuf);
        
        %VI062410A: Removed
        %state.init.hAIPhotodiode.set('readChannelsToRead',state.init.hAIPhotodiode.channels(beam).chanName); %Channels are arranged in array in order of beam index 
        
        %%%VI091309A%%%%%%%%%%
        if strcmpi(state.init.hAIPhotodiode{beam}.channels.deviceName, state.init.acquisitionBoardID) %VI062410A
            state.init.hAI.control('DAQmx_Val_Task_Unreserve');
        end
        %%%%%%%%%%%%%%%%%%%%%%                  
        
        %Start the calibration loop
        data = zeros(length(modulation_voltage),state.internal.eom.calibrationPasses);
        calibrationPassTime = length(modulation_voltage) / state.internal.eom.calibrationSampleRate;
        
        %%%VI111010B: Open shutter, if needed
        if isfield(state.shutter,'shutterBeforeEOM') &&  state.shutter.shutterBeforeEOM
            openShutter(true);  %VI040511A
        end
        
        try         
            for i=1:state.internal.eom.calibrationPasses
                if state.init.eom.cancel
                    stopCalibration();
                    return;
                end
                
                %Start acquisition
                start([state.init.eom.hAO state.init.hAIPhotodiode{beam}]); %VI062410A
                dioTrigger;
                
                %Wait for completion
                %             pause(1.0*calibrationPassTime)
                %             if ~state.init.eom.hAO.isTaskDone()
                %                 fprintf(2,'WARNING: Calibration did not occur within time expected. Calibration aborted.\n');
                %                 state.init.eom.cancel = 1;
                %                 stopCalibration();
                %                 return;
                %             end
                %
                state.init.hAIPhotodiode{beam}.waitUntilTaskDone(1.02*calibrationPassTime); %VI112410A %VI111010A
                
                %Read acquired data
                data(:,i) = state.init.hAIPhotodiode{beam}.readAnalogData();
                
                %Stop Tasks
                stop([state.init.hAIPhotodiode{beam} state.init.eom.hAO]);
                
                %Update waitbar
                waitbar(i / state.internal.eom.calibrationPasses, wb);
                
                %Apply inter-calibration delay
                pause(state.internal.eom.interCalibrationDelay);
            end
        catch ME
            closeShutter(); %VI111010B
            rethrow(ME);
        end
        
        %%%VI111010B: Close shutter, if it's been opened here
        if isfield(state.shutter,'shutterBeforeEOM') && state.shutter.shutterBeforeEOM
            closeShutter(); 
        end

        if state.init.eom.cancel
            stopCalibration();
            return;
        else %VI122909A
            state.init.eom.hAO.control('DAQmx_Val_Task_Unreserve'); %VI122909A %Allows beam to now be parked 
        end
        
        %Process data        
        if state.init.eom.photodiodeInvert(beam) %VI032311A
            data = -data; %VI032311A
        end
        photodiode_voltage = mean(data, 2);
        photodiode_voltage_stdDev = std(data, 1, 2); %Normalize by the number of calibration passes
        
        %Subtract off any offset in the detector electronics.
        photodiode_voltage = photodiode_voltage - state.init.eom.(['photodiodeOffset' num2str(beam)]);
        
        %Gather up the return variables.
        [eom_min min_p] = min(photodiode_voltage);
        [eom_max max_p] = max(photodiode_voltage); %VI081011A
        avg_dev = mean(photodiode_voltage_stdDev ./ eom_max);
        state.init.eom.maxPhotodiodeVoltage(beam) = eom_max;
        
        if (avg_dev > .35) || ((eom_min / eom_max) > .15)%Bad data? -- Too noisy or not enough attenuation.
            fprintf(2, '\nWARNING: Pockels cell calibration data seems bad.\n');
            beep;
            if (eom_min / eom_max) > .15
                fprintf(2, '  Pockels cell minimum power not less than 15%% of maximum power. Min: %s%%\n', num2str( 100 * eom_min / eom_max));
            else
                fprintf(2, '  Pockels cell calibration seems excessively noisy.\n  Typical standard deviation per sample: %s%%\n', num2str(100 * avg_dev));
            end
            
            f = figure('NumberTitle', 'off', 'DoubleBuffer', 'On','Name', 'Pockels Cell Calibration Curve', 'Color', 'White');
            a = axes('Parent', f, 'FontSize', 12, 'FontWeight', 'Bold');
            plot(modulation_voltage, photodiode_voltage, 'Parent', a, 'Color', [0 0 0], 'LineWidth', 2);
            %TO1604 - Added the beam number to the plot.
            t = sprintf('Pockels Cell Calibration Curve (beam: %s)', num2str(beam));
            title(t, 'Parent', a, 'FontWeight', 'bold');
            xlabel('Modulation Voltage (From DAQ Board) [V]', 'Parent', a, 'FontWeight', 'bold');
            ylabel('Photodiode Voltage [V]','Parent', a, 'FontWeight', 'bold');
            state.internal.figHandles = [f state.internal.figHandles]; %VI110708A
        end
        
        % Normalize.
        photodiode_voltage = photodiode_voltage / eom_max;
        
        %Take measurement from rejected light, if necessary.
        %Note: The return values are still in absolute form.
        rejected = state.init.eom.(['rejected_light' num2str(beam)]);
        if rejected
            photodiode_voltage = 1 - photodiode_voltage;
        end
        
        %Round off to the nearest %.
        photodiode_voltage = round(100 * photodiode_voltage);
        photodiode_voltage(photodiode_voltage < 0) = 0;
        
        %Identify minimum power percentage
        p = ceil(100 * eom_min / eom_max);
        
        if isnan(p) %dud data
            fprintf(2,'WARNING: Photodiode data is flat--possibly disconnected. Using naive linear calibration instead\n');
            naiveEOMCalibrate(beam);
            p=1;
        else
            if p < 1
                p = 1;
            elseif p > 100   %TO1604 - This seemed to happen when the laser was acting funny (not mode-locked?).
                p = 100;
                fprintf(2, 'WARNING: Pockels cell calibration appears saturated or noisy\n');
            end
            
            if rejected %VI081011A
                state.init.eom.lut(beam, 1:p) = modulation_voltage(max_p); %VI081011A
            else
                state.init.eom.lut(beam, 1:p) = modulation_voltage(min_p);
            end
        end
        %TO1604 - Look into this some more...
        state.init.eom.min(beam) = p;
        
        %Set the real values.
        for i = (p+1):100
            
            if rejected %VI081011A
                pos = max_p+find(photodiode_voltage(max_p+1:min_p-1) == i); %VI081011A
            else
                pos = find(photodiode_voltage == i); %Locate a modulation voltage that gives the desired transmittance.
            end
            
            if length(pos) > 0
                %Take the one closest to the last voltage.
                pos_diff = abs(modulation_voltage(pos) - state.init.eom.lut(beam, i - 1));
                [val loc] = min(pos_diff);
                state.init.eom.lut(beam, i) = modulation_voltage(pos(loc));
            else %if length(pos) > 0
                if i == 100 %Just keep the last one.
                    state.init.eom.lut(beam, i) = state.init.eom.lut(beam, i - 1);
                elseif i == state.init.eom.min(beam)
                    state.init.eom.lut(beam, i) = modulation_voltage(min_p);
                elseif i > state.init.eom.min(beam)
                    %Assume local linearity.
                    %This can result in something very ugly...  but, it's still a reasonable method.
                    if i > 2
                        step = state.init.eom.lut(beam, i - 1) - state.init.eom.lut(beam, i - 2);
                        if abs(step) < (.2 * pockelsVoltageRange)
                            state.init.eom.lut(beam, i) = state.init.eom.lut(beam, i - 1) + step;%Project outwards.
                        else
                            state.init.eom.lut(beam, i) = state.init.eom.lut(beam, i - 1);%Give up, and just use the last value.
                        end
                    else %if i > 2
                        state.init.eom.lut(beam, i) = state.init.eom.lut(beam, i - 1);
                    end %if i > 2
                end %if i == 100
            end %if length(pos) > 0
        end  %for i = p + 1:100
        
        
        %Provide warnings if modulation voltages determined are outside of range
        over = find(state.init.eom.lut > pockelsVoltageRange);
        if over
            if find((state.init.eom.lut(over) - pockelsVoltageRange) >= .005)
                fprintf(2, ['Warning: Illegal entries found in the Pockels cell voltage lookup table (exceeded maximum value of ' num2str(pockelsVoltageRange) 'V specified in INI file). Resetting them into legal range.' sprintf('\n')]); %VI101608A, VI110208A
            end
            state.init.eom.lut(over) = pockelsVoltageRange;
        end
        under0 = find(state.init.eom.lut < 0);
        if under0
            if find(state.init.eom.lut(under0) <= -.005)
                fprintf(2, 'Warning: Illegal entries found in the Pockels cell voltage lookup table (under 0V), resetting them into legal range.\n');
            end
            state.init.eom.lut(under0) = 0;
        end
        
        if size(state.init.eom.lut(beam, :), 2) ~= 100
            error(sprintf('Pockels cell %s lookup table size out of bounds: %s', num2str(beam), num2str(size(state.init.eom.lut(beam, :), 2))));
        end
        
        %TO12204a - Tim O'Connor 1/24/04: Somehow the min wasn't a cardinal value when one laser was configured but turned off.
        state.init.eom.min(beam) = ceil(state.init.eom.min(beam));
        
        %% Clean up tasks
        markCalibration(beam); %VI041008B
        delete(wb);
        
        %Mark a beam as calibrated
        function markCalibration(beam)
            
            state.init.eom.changed(beam) = 1;
            ensureEomGuiStates(beam);%TO22604g
            
            %Flag that it's been calibrated (at least once). TO22604e
            state.init.eom.calibrated(beam) = 1;
            
        end       
        
    end

    %Clean up calibration AO/AI Tasks
    function stopCalibration
        stop(state.init.eom.hAO);        
        %%%%VI062410A%%%%
        for i=1:length(beams)
            hTask = state.init.hAIPhotodiode{beams(i)};
            if ~isempty(hTask)
                stop(hTask)
            end
        end
        %%%%%%%%%%%%%%%%%
        scim_parkLaser(); %Sets beam powers to newly calculated minimum values
        flushAOData(); %Prepares beam for scanning again        
    end

end



