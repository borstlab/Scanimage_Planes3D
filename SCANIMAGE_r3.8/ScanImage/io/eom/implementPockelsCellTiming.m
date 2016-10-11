function data = implementPockelsCellTiming(beam)
%% function data = implementPockelsCellTiming(beam)
%  This function implements Pockels pattern for 'special features' like the Power BOX, Uncaging Mapper, etc.
%
%% SYNTAX
%   data = implementPockelsCellTiming(beam)
%       beam: The index of the beam # to compute the Pockels data for
%       data: Row vector of samples for /all/ frames
%
%% NOTES
%   This version was rewritten from scratch. To see earlier versions of this function, see implementPockelsCellTiming.mold -- Vijay Iyer 2/4/09s
%
%   Data output by this function pertains to /all/ frames. RepeatOutput is not used when any of the special EOM features are active.
%
%   Only Power Box feature is supported at this time -- Vijay Iyer 2/4/09
%
%   TODO: For PowerBox, if flyback blanking is enabled, then use minimum power outside of acquisition window -- this may be different than the state.init.eom.boxPowerOffArray value -- Vijay Iyer 6/7/10
%
%% CHANGES
%   VI102309A: Handle slow dimension flyback options -- Vijay Iyer 10/23/09
%   VI060710A: Actually use the state.init.eom.boxPowerOffArray value; for now, this applies to all time outside the box, including outside the acquisition window. -- Vijay Iyer 6/7/10
%
%% CREDITS
%  Created 2/4/09, by Vijay Iyer
%  Based heavily on earlier version by Tom Pologruto and Tim O'Connor
%% ********************************************************
global state gh;

%Preallocate data in framewise fashion
data = state.init.eom.lut(beam, state.init.eom.min(beam)) + zeros(state.internal.lengthOfXData * state.internal.storedLinesPerFrame,state.acq.numberOfFrames); %VI102309A: Use storedLinesPerFrame instead of scanned linesPerFrame, as this is what was used to select/specify the PowerBox

rateFactor = state.acq.inputRate /state.acq.outputRate;

%Determine start/end of line
[lineAISamples(1) lineAISamples(2)] = determineAcqColumns();
lineAOSamples = round(lineAISamples/rateFactor);
lineSampleError = lineAISamples(1) - lineAOSamples(1)*rateFactor;

%Handle PowerBox feature
if length(state.init.eom.showBoxArray) < beam
    state.init.eom.showBoxArray(beam) = 0;
end

%Determine if focusing at present; won't actually recompute if so; shouldn't happen!
focusingNow = strcmpi(get(gh.mainControls.focusButton, 'String'), 'ABORT');


%Helper functions
    function [boxAOSamples, boxAOSamplesInv] = computeBoxAOSamples
        %Helper function
        convertToAO = @(AISamples)(AISamples-1)/rateFactor + 1;

        %Determine start & end pixel to target
        boxPixels = round([state.init.eom.powerBoxNormCoords(beam,1) sum(state.init.eom.powerBoxNormCoords(beam, [1 3]))] * state.acq.pixelsPerLine); %round is redundant, but ensures no miniscule error

        %Convert start/end pixels into samples
        boxAISamples = lineAISamples(1) + (boxPixels - 1) * state.acq.binFactor;
        boxAOSamples = round(convertToAO(boxAISamples));
        boxSampleErrors = round(convertToAO(boxAISamples)) - convertToAO(boxAISamples);

        %Compute inverted line for bidi scan, if applicable
        if state.acq.bidirectionalScan
            boxAISamplesInv = lineAISamples(1) + (state.acq.pixelsPerLine - boxPixels(end)) * state.acq.binFactor + [0 diff(boxAISamples)];
            boxAOSamplesInv = round(convertToAO(boxAISamplesInv)) - 1; %Do not understand why the -1 is useful, but on average this leads to better results
            boxSampleErrorsInv = round(convertToAO(boxAISamplesInv)) - convertToAO(boxAISamplesInv);
        else
            boxAOSamplesInv = [];
        end

        %Adjust commands for Pockels response
        boxAOSamples = boxAOSamples - 1;
        if state.acq.bidirectionalScan
            boxAOSamplesInv = boxAOSamplesInv - 1;
        end
    end

    function [normLines, invertedLines] = identifyLines()
        if state.acq.bidirectionalScan
            normLines = lines(mod(lines,2) > 0);
            invertedLines = setdiff(lines,normLines);
        else
            normLines = lines;
            invertedLines = [];
        end
    end

if state.init.eom.showBoxArray(beam) && ~isempty(state.init.eom.powerBoxNormCoords) && ~focusingNow ...
        && ~state.init.eom.uncagingPulseImporter.enabled %left this in for posterity
    
    
    %%%VI060710A: For powerBox feature, use boxPowerOffArray value everywhere outside of the box
    [data(:)] = state.init.eom.lut(beam, state.init.eom.boxPowerOffArray(beam));

    %Compute Box AO Samples
    [boxAOSamples, boxAOSamplesInv] = computeBoxAOSamples();

    %12/16/03 Tim O'Connor - Modified to behave differently during linescan. (Not sure why at the moment -- Vijay Iyer 2/19/09)
    if state.acq.linescan == 1
        %Reformat data into a single column array, up front
        data = reshape(data,[state.internal.lengthOfXData * state.acq.linesPerFrame * state.acq.numberOfFrames 1]);

        %The different behavior during linescans is intended for use during uncaging experiments.
        if state.init.eom.endFrameArray(beam) < state.init.eom.startFrameArray(beam)
            beep;
            fprintf(2, 'WARNING: ''Start Line'' must come before ''End Line'' in powerbox settings. The powerbox has not been applied to the scan.\n');
        elseif state.init.eom.endFrameArray(beam) > (state.internal.storedLinesPerFrame * state.acq.numberOfFrames) %VI102309A
            beep;
            fprintf(2, 'WARNING: ''End Line'' must fall within the time bounds of the scan. The powerbox has not been applied to the scan.\n');
        else

            %Determine which 'frames' (lines) are inverted
            lines = state.init.eom.startFrameArray(beam) : state.init.eom.endFrameArray(beam);
            [normLines, invertedLines] = identifyLines();

            if state.init.eom.startFrameArray(beam)> 0 && state.init.eom.endFrameArray(beam) > 0 && round(state.init.eom.boxPowerArray(beam)) > 0
                for lineCounter = normLines
                    data((lineCounter-1)*state.internal.lengthOfXData + (boxAOSamples(1):boxAOSamples(2)), 1) = ...
                        state.init.eom.lut(beam,  round(state.init.eom.boxPowerArray(beam)));
                end

                %Handle inverted lines for bidi scan, if applicable
                if state.acq.bidirectionalScan
                    for lineCounter = invertedLines
                        data((lineCounter-1)*state.internal.lengthOfXData + (boxAOSamplesInv(1):boxAOSamplesInv(2)), 1) = ...
                            state.init.eom.lut(beam,  round(state.init.eom.boxPowerArray(beam)));
                    end
                end
            end
        end

    else
        %Process start/endFrame...this validation should already be done, ideally
        startFrame = round(state.init.eom.startFrameArray(beam));
        endFrame = round(min(state.init.eom.endFrameArray(beam), state.acq.numberOfFrames));

        min([startFrame endFrame], 1);

        if endFrame < startFrame
            beep;
            fprintf(2, 'WARNING: ''Start Frame'' must come before ''End Frame'' in powerbox settings.\n');
        end
        
        
        %Identify array of lines within the power box
        lines = floor([state.init.eom.powerBoxNormCoords(beam, 2) sum(state.init.eom.powerBoxNormCoords(beam, [2 4]))] .* state.internal.storedLinesPerFrame); %VI102309A

        %Ensure that one line is selected, if so desired (per Tim O'Connor 6/2/04 TO060204a)
        if state.init.eom.powerBoxUncagingConstraint
            lines(2) = lines(1);
            lines = lines(1:2);
        end

        %Determine which lines are 'normal' and which are inverted due to bidi scanning
        lines = lines(1):lines(2);
        [normLines, invertedLines] = identifyLines();

        if endFrame && startFrame && round(state.init.eom.boxPowerArray(beam)) > 0

            for frameCounter = startFrame : endFrame
                for lineCounter = normLines
                    data((lineCounter-1)*state.internal.lengthOfXData + (boxAOSamples(1):boxAOSamples(2)), frameCounter) = ...
                        state.init.eom.lut(beam,  round(state.init.eom.boxPowerArray(beam)));
                end

                %Handle inverted lines for bidi scan, if applicable
                if state.acq.bidirectionalScan
                    for lineCounter = invertedLines
                        data((lineCounter-1)*state.internal.lengthOfXData + (boxAOSamplesInv(1):boxAOSamplesInv(2)), frameCounter) = ...
                            state.init.eom.lut(beam,  round(state.init.eom.boxPowerArray(beam)));
                    end
                end
            end

        end
    end
end

%%%VI102309A%%%%%%%
%Append extra line of scanned data at minimum power
if state.acq.slowDimDiscardFlybackLine
    data(end+1:end+state.internal.lengthOfXData,:) = state.init.eom.lut(beam, state.init.eom.min(beam));    
end
%%%%%%%%%%%%%%%%%%%

%reshape data and output it...
data = reshape(data,[state.internal.lengthOfXData * state.acq.linesPerFrame * state.acq.numberOfFrames 1]);



end