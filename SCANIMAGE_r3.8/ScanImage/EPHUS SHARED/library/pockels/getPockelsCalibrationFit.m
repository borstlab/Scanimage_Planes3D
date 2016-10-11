% getPockelsCalibrationFit - Measure a Pockels cell calibration curve and return the coeffecients of a 3rd order polynomial fit.
%
% SYNTAX
%  [coeffs, maxV, minV, offset] = getPockelsCalibrationFit(pockelsCellDevice, photodiodeDevice, triggerOrigin, pockelsCellTriggerDestination, photodiodeTriggerDestination, shutterDevice)
%  [coeffs, maxV, minV, offset] = getPockelsCalibrationFit(pockelsCellDevice, photodiodeDevice, triggerOrigin, pockelsCellTriggerDestination, photodiodeTriggerDestination, shutterDevice, sampleClockTaskOrJob, sampleClockSource)
%   pockelsCellDevice - The NIMEX device name for the analog output connected to the pockels cell under test.
%   photodiodeDevice - The NIMEX device name for the analog output connected to the photodiode under test.
%   triggerOrigin - The NIMEX device name for the digital line that will send a trigger.
%   pockelsCellTriggerDestination - The NIMEX terminal name for the pockels cell task to accept a trigger.
%   photodiodeTriggerDestination - The NIMEX terminal name for the photodiode task to accept a trigger.
%   shutterDevice - The NIMEX device name for the shutter control.
%   sampleClockTaskOrJob - A preconfigured NIMEX task to be used as the master sample clock source, or a @daqjob which contains such a task
%   sampleClockDestination - The NIMEX terminal for the sample clock destination. If a daqjob was entered for the sampleClockTaskOrJob, do not use this argument.
%   coeffs - The coefficients of a 3rd order fit, as per the Matlab `\` operator.
%   maxV - The maximum voltage measured on the photodiode [V].
%   minV - The minimum voltage measured on the photodiode [V].
%   offset - The photodiode's voltage offset (due to ambient light and/or electronics) [V].
%
% NOTES
%
% CHANGES
% JL102307A   modified the code to @nimex
% TO012408E - Add a title to the plot. -- Tim O'Connor 1/24/08
% TO080108E - Allow external sample clocks. Output digital shutter signals, if necessary. -- Tim O'Connor 8/1/08
% VI102608A - Allow a job containing a sample clock task to be supplied, rather than the task directly -- Vijay Iyer 10/26/08
% TO021510E - Make the modulation voltage range configurable. -- Tim O'Connor 2/15/10
% 2011-06-11, Ben Suter -- new implementation to fix bug wherein the fit
% was not performed over the full [0 ... 1] photodiode range, because the
% normalization did not first subtract the minimum measured value. I think
% the original implementation was perhaps confused about the two "offsets"
% involved, namely the "offset" due to a dark current from the photodiode,
% and the "offset" due to the Pockels still passing some non-zero power at
% minimal transmission. This second offset was not subtracted before
% normalizing, and as a result the "normalized intensity" of the photodiode
% could range from, say, [0.4 ... 1.0]. As a result, requests for e.g. "0"
% power were projected to Pockels command voltages that fall outside the
% defined modulation range, resulting in UNDER_VOLTAGE (or OVER_VOLTAGE)
% errors. I kept the old implementation as a renamed sub-function.
% 2011-06-11, Ben Suter -- tried to fit with sine squared sin^2(ax+b) and
% this works very well, but I don't know how to invert it. So instead, I am
% returning a lookup table.
% 2011-06-11, Ben Suter -- BUGFIX: one-off sample bug: there is a
% 1-sample lag between the photodiode reading and the command voltage. The
% second photodiode sample reflects the first command sample, and so on. I
% fixed this by shifting the trains of samples by one, rather than by
% changing the parts of the code that talk to the hardware (not sure how to
% do that).
%
% Created 10/22/07 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2007
function [coeffs, maxV, minV, offset, pockels_lookup] = getPockelsCalibrationFit(pockelsCellDevice, photodiodeDevice, triggerOrigin, pockelsCellTriggerDestination, photodiodeTriggerDestination, shutterDevice, minModulationVoltage, maxModulationVoltage, varargin)

samplingRate = 1000; % was 10000 previously
modulationVoltageIncrement = 0.01;
modulation_voltage = (minModulationVoltage:modulationVoltageIncrement:maxModulationVoltage-modulationVoltageIncrement)';

% Use this pattern to test for any errors in matching photodiode_voltage
% samples to modulation_voltage samples. I caught a one-off bug this way
% and fixed it below. 2011-06-11, Ben Suter
% modulation_voltage = [ -1 -0.5 0 0.5 1 0.5 0 -0.5 -1 0 0 0 0 0 0 0 0 0 0 1 -1 1 -1 0 0 0 0 0 ];

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Ramp the Pockels modulation voltage across the defined range 
% while recording the resulting photodiode voltage, so that 
% we can calculate a fit between them later.
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

pockelsCellTask = nimex;
nimex_addAnalogOutput(pockelsCellTask, pockelsCellDevice);
photodiodeTask = nimex;
nimex_addAnalogInput(photodiodeTask, photodiodeDevice);
shutterTask = nimex;
nimex_addAnalogOutput(shutterTask, shutterDevice);

%TO080108E
sampleClockDestination = '';
masterSampleClockTask = [];
if ~isempty(varargin)
    %%%VI102608A%%%%%%%%%%%%%%%
    if isa(varargin{1},'nimex')
        masterSampleClockTask = varargin{1};
        sampleClockDestination = varargin{2};
    elseif isa(varargin{1},'daqjob') && (length(varargin{1}) == 1)
        job = varargin{1}; 
        masterSampleClockTask = getMasterSampleClock(job);
        sampleClockDestination = getSampleClockDestination(job);
                
        %Set the job's trigger destination to be the default in its list
        [triggerDests, cachedTriggerDest] = get(job,'triggerDestinations', 'triggerDestination');
        set(job,'triggerDestination', triggerDests{1});        
    else
        error('If digital stimulator channels are in use, either a valid NIMEX task and sample clock destination terminal OR a @daqjob object must be supplied');
    end   
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

% fprintf(1, 'Test mode, changing control signal to a sin^2 curve\n');
% testModulation_voltage = sin(0 : (pi/2) / 100 : (pi/2) - (pi/2)/100).^2;
% modulation_voltage = testModulation_voltage;

% JL102307A   modified the code to @nimex
nimex_setTaskProperty(pockelsCellTask, 'samplingRate', samplingRate, 'triggerSource', pockelsCellTriggerDestination, 'sampsPerChanToAcquire', length(modulation_voltage), 'clockSource', sampleClockDestination);%TO080108E,BS061111
nimex_setTaskProperty(photodiodeTask, 'samplingRate', samplingRate, 'triggerSource', photodiodeTriggerDestination, 'sampsPerChanToAcquire', length(modulation_voltage), 'clockSource', sampleClockDestination);%TO080108E,BS061111

%TO080108E
if ~isempty(masterSampleClockTask)
    nimex_startTask(masterSampleClockTask);
end

% BS061111 offset not needed anymore, but we keep this here because it is
% an output argument of the function (even though not used)
%Find photodiode offset.
nimex_startTask(photodiodeTask);
nimex_sendTrigger(photodiodeTask, triggerOrigin);
offsetdata = nimex_readAnalogF64(photodiodeTask, length(modulation_voltage));
nimex_stopTask(photodiodeTask);
offset = mean(offsetdata);

%Output the Pockels cell control signal (a ramp).
nimex_writeAnalogF64(pockelsCellTask, pockelsCellDevice, modulation_voltage, length(modulation_voltage));
%TO080108E - Handle digital shutter lines cleanly.
if isempty(strfind(shutterDevice, 'port'))
    nimex_putSample(shutterTask, shutterDevice, 5);
else
    nimex_putSample(shutterTask, shutterDevice, uint32(255));
end

nimex_startTask(photodiodeTask);
nimex_startTask(pockelsCellTask);
nimex_sendTrigger(photodiodeTask, triggerOrigin);

%Read in the photodiode voltages corresponding to the Pockels cell control signal.
photodiode_voltage = nimex_readAnalogF64(photodiodeTask, length(modulation_voltage));
%TO080108E - Use proper digital signals.
if isempty(strfind(shutterDevice, 'port'))
    nimex_putSample(shutterTask, shutterDevice, 0);
else
    nimex_putSample(shutterTask, shutterDevice, uint32(0));
end

%TO080108E
if ~isempty(masterSampleClockTask)
    nimex_stopTask(masterSampleClockTask);
    %%%VI102608A: Restore trigger destination to pre-existing value
    if exist('job')
        set(job, 'triggerDestination', cachedTriggerDest);
    end
    %%%%%%%%%%%%%%%%%%%%%%
end

nimex_stopTask(photodiodeTask);
nimex_stopTask(pockelsCellTask);


% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find minimal and maximal power transmission, select the corresponding
% Pockels modulation voltage range between these points, along 
% with the corresponding photodiode voltage readings. 
% Normalize the photodiode voltage to range between 0 (min tranmission)
% and 1 (max transmission), and calculate a fit between 
% modulation command and photodiode reading (e.g. transmitted power).
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% There is a 1-sample lag in the photodiode readings, relative to the
% command samples, and these two lines fix that problem.
% 2011-06-11, Ben Suter
photodiode_voltage = photodiode_voltage(2:end);
modulation_voltage = modulation_voltage(1:end-1);
% 2011-10-03, Ben Suter and Tim O'Connor, the underlying issue may be due to rising or falling edge triggering


pdv = photodiode_voltage;

% 1. Find PD voltage at min transmission
[mmn mmni] = min(pdv);
[unused idx] = min(abs(mmn)); % closest to 0 V command
mn = mmn(idx);
mni = mmni(idx);

% 2. Find PD voltage at max tranmission
[mmx mmxi] = max(pdv);
[unused idx] = min(abs(mmx-mn));
mx = mmx(idx);
mxi = mmxi(idx);

% 3. Select PD voltage and Pockels command voltage range between min and
% max transmission
if mxi > mni
    pdv = pdv(mni:mxi);
    pov = modulation_voltage(mni:mxi);
else
    pdv = pdv(mni:-1:mxi);
    pov = modulation_voltage(mni:-1:mxi);
end

% 4. Normalize PD voltage between 0 and 1, so that we can easily interpret
% this as percent (or fraction) of Pockels transmission range, and after
% fitting easily map from Pockels pulse amplitude to this range.
pdvn = (pdv - pdv(1)) ./ (pdv(end) - pdv(1));

% 5. Calculate parameters of fit to equation

% The polynomial fit is not good like this. Instead of improving it, I
% implemented a lookup table approach.
coeffs = [];
% coeffs = [ones(size(pdvn)) pdvn pdvn.^2 pdvn.^3] \ pov; % 3rd order polynomial

% The sine squared fit is very good, but isn't useful because I don't know
% how to invert it (which is what we want to do: given a desired power,
% what command voltage do we need). We want to know Pockels command voltage
% as a function of transmitted power.
% [estimates] = fitSineSquared(pov, pdvn); 

pockels_lookup.transmitted_power = pdvn;
pockels_lookup.command_voltage = pov;

% 6. Return some measurements for backwards compatibility: minV, maxV, offset
minV = mn;
maxV = mx;
fprintf(1, '\n%s - Pockels cell calibration -\n ambient light/amplifier offset: %g [V]\n photodiode min: %g [V]\n photodiode max: %g [V]\n\n\n', datestr(now), offset, minV, maxV);

% 7. Plot the raw data and resulting fit
h = figure;
scrsz = get(0,'ScreenSize');
set(h, 'Position', [1 1 scrsz(3:4) * 0.7]);

subplot(2, 2, 1);
plot(offsetdata, 'o:');
xlabel('sample #'); ylabel('photodiode voltage (V)');
title('Photodiode dark current (shutter closed), i.e. offset');

subplot(2, 2, 3);
plot(modulation_voltage, photodiode_voltage, 'ko');
xlabel('Pockels modulation voltage (V)');
ylabel('photodiode voltage (V)');
title('Photodiode response to Pockels ramp, raw values');

subplot(2, 2, [2 4]);
hold on;
plot(pov, pdvn, 'go');

% fitx = [ones(size(pdvn)) pdvn pdvn.^2 pdvn.^3] * coeffs; % fit Pockels command voltage for percent transmission
% plot(fitx, pdvn, 'b');

% fit2 = sin(estimates(1)*pov + estimates(2)).^2;
% plot(pov, fit2, 'r');

xlabel('Pockels modulation voltage (V)');
ylabel('photodiode response, normalized');
title(sprintf('Pockels transmission (%4.2f to %4.2f V) vs. modulation voltage (%2.1f to %2.1f V)', minV, maxV, minModulationVoltage, maxModulationVoltage));
legend({'Data after normalization'});
% legend({'Data after normalization', 'Fit to data: a+bx+cx^2+dx^3'});
% legend({'Data after normalization', 'Fit to data: a+bx+cx^2+dx^3', 'Fit to data: sin(a*x+b)^2'});

end

% function [estimates, model] = fitSineSquared(xdata, ydata)
% % Call fminsearch with a random starting point.
% start_point = rand(1, 2);
% model = @expfun;
% estimates = fminsearch(model, start_point);
% % expfun accepts curve parameters as inputs, and outputs sse,
% % the sum of squares error for sin(phase*xdata + period).^2 - ydata,
% % and the FittedCurve. FMINSEARCH only needs sse, but we want
% % to plot the FittedCurve at the end.
%     function [sse, FittedCurve] = expfun(params)
%         period = params(1);
%         phase = params(2);
%         FittedCurve = sin(period*xdata + phase).^2;
%         ErrorVector = FittedCurve - ydata;
%         sse = sum(ErrorVector .^ 2);
%     end
% end

function [coeffs, maxV, minV, offset] = getPockelsCalibrationFit_OldWithBug(pockelsCellDevice, photodiodeDevice, triggerOrigin, pockelsCellTriggerDestination, photodiodeTriggerDestination, shutterDevice, minModulationVoltage, maxModulationVoltage, varargin)

pockelsCellTask = nimex;
nimex_addAnalogOutput(pockelsCellTask, pockelsCellDevice);
photodiodeTask = nimex;
nimex_addAnalogInput(photodiodeTask, photodiodeDevice);
shutterTask = nimex;
nimex_addAnalogOutput(shutterTask, shutterDevice);

%TO080108E
sampleClockDestination = '';
masterSampleClockTask = [];
if ~isempty(varargin)
    %%%VI102608A%%%%%%%%%%%%%%%
    if isa(varargin{1},'nimex')
        masterSampleClockTask = varargin{1};
        sampleClockDestination = varargin{2};
    elseif isa(varargin{1},'daqjob') && (length(varargin{1}) == 1)
        job = varargin{1}; 
        masterSampleClockTask = getMasterSampleClock(job);
        sampleClockDestination = getSampleClockDestination(job);
        
        
        %Set the job's trigger destination to be the default in its list
        [triggerDests, cachedTriggerDest] = get(job,'triggerDestinations', 'triggerDestination');
        set(job,'triggerDestination', triggerDests{1});        
    else
        error('If digital stimulator channels are in use, either a valid NIMEX task and sample clock destination terminal OR a @daqjob object must be supplied');
    end   
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

modulation_voltage = (minModulationVoltage:0.01:maxModulationVoltage-0.01)';%TO021510E

% fprintf(1, 'Test mode, changing control signal to a sin^2 curve\n');
% testModulation_voltage = sin(0 : (pi/2) / 100 : (pi/2) - (pi/2)/100).^2;
% modulation_voltage = testModulation_voltage;

% JL102307A   modified the code to @nimex

nimex_setTaskProperty(pockelsCellTask, 'samplingRate', 10000, 'triggerSource', pockelsCellTriggerDestination, 'sampsPerChanToAcquire', length(modulation_voltage), 'clockSource', sampleClockDestination);%TO080108E
nimex_setTaskProperty(photodiodeTask, 'samplingRate', 10000, 'triggerSource', photodiodeTriggerDestination, 'sampsPerChanToAcquire', length(modulation_voltage), 'clockSource', sampleClockDestination);%TO080108E

%TO080108E
if ~isempty(masterSampleClockTask)
    nimex_startTask(masterSampleClockTask);
end

%Find photodiode offset.
nimex_startTask(photodiodeTask);
nimex_sendTrigger(photodiodeTask, triggerOrigin);
offsetdata = nimex_readAnalogF64(photodiodeTask, length(modulation_voltage));
nimex_stopTask(photodiodeTask);
figure, plot(offsetdata, 'o:');
offset = mean(offsetdata);

%Output the Pockels cell control signal (a ramp).
nimex_writeAnalogF64(pockelsCellTask, pockelsCellDevice, modulation_voltage, length(modulation_voltage));
% %TO100511A - In NI hardware, the per sample analog input operation may complete before the per sample analog output operation completes.
% %This is a function of the voltage step and the slew rate, so tweaking board parameters is not an ideal solution, since it depends on the signal content.
% %See http://forums.ni.com/t5/Multifunction-DAQ/Strange-problem-with-simultaneous-analog-output-input/td-p/1502470 for a small discussion of the topic.
% %Essentially, the result is a sub-sample length, variable (waveform dependent) phase shift between the analog input and analog output, regardless of clock synchronization.
% %If using this little hack, remove Ben's correction(s) below.
% putSample(pockelsCellTask, pockelsCellDevice, modulation_voltage(1));%This will pre-set the first sample.

%TO080108E - Handle digital shutter lines cleanly.
if isempty(strfind(shutterDevice, 'port'))
    nimex_putSample(shutterTask, shutterDevice, 5);
else
    nimex_putSample(shutterTask, shutterDevice, uint32(255));
end

nimex_startTask(photodiodeTask);
nimex_startTask(pockelsCellTask);

nimex_sendTrigger(photodiodeTask, triggerOrigin);

%Read in the photodiode voltages corresponding to the Pockels cell control signal.
photodiode_voltage = nimex_readAnalogF64(photodiodeTask, length(modulation_voltage));
maxV = max(photodiode_voltage);
[minV, mni] = min(photodiode_voltage);
%TO080108E - Use proper digital signals.
if isempty(strfind(shutterDevice, 'port'))
    nimex_putSample(shutterTask, shutterDevice, 0);
else
    nimex_putSample(shutterTask, shutterDevice, uint32(0));
end

% 2011-06-10, Ben Suter
% Plots for debugging UNDER_VOLTAGE and OVER_VOLTAGE errors
figure; 
subplot(2, 2, 1); plot(photodiode_voltage); title('photodiode_voltage');
subplot(2, 2, 2); plot(photodiode_voltage - offset); title(sprintf('photodiode_voltage - offset, offset: %g', offset));
subplot(2, 2, 3); plot(modulation_voltage, photodiode_voltage); xlabel('modulation_voltage'); ylabel('photodiode_voltage');
title(sprintf('maxV: %g, minV: %g, mni: %g', maxV, minV, mni));
subplot(2, 2, 4); plot(modulation_voltage, photodiode_voltage-offset); xlabel('modulation_voltage'); ylabel('photodiode_voltage-offset');
title(sprintf('modulation_voltage(mni): %g', modulation_voltage(mni)));

photodiode_voltage = photodiode_voltage - offset;
fprintf(1, '\n%s - Pockels cell calibration -\n ambient light/amplifier offset: %s [V]\n photodiode min: %s [V]\n photodiode max: %s [V]\n\n\n', datestr(now), num2str(offset), ...
    num2str(min(photodiode_voltage)), num2str(max(photodiode_voltage)));


%TO080108E
if ~isempty(masterSampleClockTask)
    nimex_stopTask(masterSampleClockTask);
    %%%VI102608A: Restore trigger destination to pre-existing value
    if exist('job')
        set(job, 'triggerDestination', cachedTriggerDest);
    end
    %%%%%%%%%%%%%%%%%%%%%%
end

nimex_stopTask(photodiodeTask);
nimex_stopTask(pockelsCellTask);

eom_max = maxV - offset;

%Normalize
photodiode_voltage = photodiode_voltage / eom_max;

%Check that the calibration is valid.
mn = minV - offset;
if length(mni) > 1 && length(mni) < 3
    mni = mni(1);
elseif length(mni) >= 3
    warning('Too many minima');
    mni = mni(1);
end
[mx mxi] = max(photodiode_voltage);
if length(mxi) > 1 && length(mxi) < 3
    mxi = mxi(1);
elseif length(mxi) >= 3
    warning('Too many maxima');
    mxi = mxi(1);
end
if mxi > mni
    photodiode_voltage = photodiode_voltage(mni:mxi);
    modulation_voltage = modulation_voltage(mni:mxi);
elseif mni > mxi
    photodiode_voltage = photodiode_voltage(mxi:mni);
    modulation_voltage = modulation_voltage(mxi:mni);
end

%Calculate a fit of the photodiode voltage to the Pockels cell control signal.
coeffs = [ones(size(photodiode_voltage)) photodiode_voltage photodiode_voltage.^2 photodiode_voltage.^3] \ modulation_voltage;

%Display the fit vs the real measurements.
T = (0:.01:mx)';
Y = [ones(size(T)) T T.^2 T.^3] * coeffs;
indices = find(Y >= min(modulation_voltage));
Y = Y(indices);
T = T(indices);
figure, plot(T, Y,'.-', photodiode_voltage, modulation_voltage, 'o-');
xlabel('Photodiode Intensity (Offset Subtracted, Normalized)');
ylabel('Modulation Voltage');
title('Pockels Cell Calibration');
legend('Fit', 'RawData');

return;
end
