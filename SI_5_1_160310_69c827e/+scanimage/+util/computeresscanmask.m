function [mask, samplesPerPixel, samplesToSkip] = computeresscanmask(scanFreq, sampleRate, fillFractionSpatial, pixelsPerLine, bidirectional)
%COMPUTERESSCANMASK Computes the line mask for resonant scanning
%   The mask indicates the number of samples acquired per pixel in a line
%   based on the given parameters

    sampleTimes = linspace(0,1/scanFreq,sampleRate/scanFreq);

    tFun = @(theta) acos(-2*theta) / (2*pi*scanFreq);
    pixelThetas = linspace(-(1/2),1/2,pixelsPerLine) * fillFractionSpatial;
    pixelTimes = tFun(pixelThetas);

    %% Sample to pixel assignments
    endPoints = 1.5*pixelTimes(1) - 0.5*pixelTimes(2);
    endPoints(2:numel(pixelTimes)) = .5*pixelTimes(1:end-1) + .5*pixelTimes(2:end);
    endPoints(end+1) = 1.5*pixelTimes(end) - .5*pixelTimes(end-1);

    is = find((sampleTimes - endPoints(1))>0,1);
    ip = 1;
    samplesPerPixel = zeros(pixelsPerLine,1);
    while ip <= pixelsPerLine
        if sampleTimes(is) < endPoints(ip+1)
            samplesPerPixel(ip) = samplesPerPixel(ip) + 1;
            is = is+1;
        else
            ip = ip+1;
        end
    end
    samplesPerPixel = cast(samplesPerPixel,'int16');

    %% Determine bidirectional 'sample mask'
    samplesToSkip = ((sampleRate/scanFreq) - 2*sum(samplesPerPixel))/2;
    if ~(floor(samplesToSkip)==samplesToSkip)
        %fprintf('WARNING: Samples to skip not evenly divisible. This may prevent perfect bidirectional alignment, i.e. a half-pixel error.\n');
        samplesToSkip = round(samplesToSkip);
    end
    samplesToSkip = cast(samplesToSkip,'int16');

    assert(all(samplesPerPixel),'Mask contains zero values, which will result in incorrect FPGA behavior');

    if bidirectional
        mask = [samplesPerPixel;-samplesToSkip;flipud(samplesPerPixel)];
        assert(-samplesToSkip < 0,'Mask entry ''samplesToSkip'' must be negative. Current value: %d',-samplesToSkip);
    else
        mask = samplesPerPixel;
    end
end



%--------------------------------------------------------------------------%
% computeresscanmask.m                                                     %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
