function smoothed = smoothextrema(signal, varargin)
% SMOOTHEXTREMA(signal) - Smooths out peaks around the extrema of a signal.
%
% SMOOTHEXTREMA(signal, PropertyName, PropertyValue) - Same as above, using the specified
% properties.
%
%
% Created: Timothy O'Connor 5/17/04
% Copyright: Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2004

%Only work on vectors, no matrices.
if length(size(signal)) > 2 | min(size(signal)) ~= 1
    error('Matlab:badopt', 'Input signal must be a single row/column vector.');
end

plotOn = 0;
plotPointersOn = 0;
windowSize = 1;
polarity = 1;
invariantAmplitude = 0;
for i = 1 : 2 : length(varargin)
    if strcmpi(varargin{i}, 'Plot')
        plotOn = varargin{i + 1};
    elseif strcmpi(varargin{i}, 'PlotPointers')
        plotPointersOn = varargin{i + 1};
    elseif strcmpi(varargin{i}, 'WindowSize')
        windowSize = varargin{i + 1};
        if windowSize < 1
            error('Matlab:badopt', 'Invalid window size: %s', num2str(windowSize));
        end
    elseif strcmpi(varargin{i}, 'Polarity')
        if strcmpi(varargin{i + 1}, 'bidirectional')
            polarity = 1;
        elseif strcmpi(varargin{i + 1}, 'forward')
            polarity = 2;
        elseif strcmpi(varargin{i + 1}, 'backward')
            polarity = 3;
        else
            error('Matlab:badopt', 'Invalid polarity specification: ''%s''.', varargin{i + 1});
        end
    elseif strcmpi(varargin{i}, 'InvariantAmplitude')
        invariantAmplitude = varargin{i + 1};
    end
end

filtered = [];

smn = min(signal);
smx = max(signal);
minima = find(signal == min(signal));
maxima = find(signal == max(signal));

forwardAveraged = [];
if polarity == 1 | polarity == 2
    for i = windowSize : -1 : 1
        for j = 1 : windowSize - i + 1
            forwardAveraged = [forwardAveraged (minima - i)' (maxima - i)'];
        end
    end
    forwardAveraged = forwardAveraged(find(forwardAveraged > 0));
end

backwardAveraged = [];
if polarity == 1 | polarity == 3
    for i = 1 : windowSize
        for j = 1 : windowSize - i + 1
            backwardAveraged = [backwardAveraged (minima + i)' (maxima + i)'];
        end
    end
    backwardAveraged = backwardAveraged(find(backwardAveraged <= length(signal)));
end

%It's useful to look at this for diagnostic reasons.
if plotPointersOn
    figure;
    plot(signal);
    
    hold on;

    plot(minima, ones(length(minima), 1) * smn, 'or');
    plot(maxima, ones(length(maxima), 1) * smx, 'og');

    plot(forwardAveraged, signal(forwardAveraged), '>k');
    plot(backwardAveraged, signal(backwardAveraged), '<k');
    
    xlabel('Samples');
    ylabel('Amplitude');
    title(sprintf('smoothextrema: Input (windowsize: %s)', num2str(windowSize)));
    legend('Signal', 'Minima', 'Maxima', 'ForwardAveraged', 'BackwardAveraged');
    
    hold off;
end

smoothed = signal;
%Do the forward averaging.
for i = 1 : length(forwardAveraged)
    smoothed(forwardAveraged(i)) = (smoothed(forwardAveraged(i)) + smoothed(forwardAveraged(i) + 1)) / 2;
end
% smoothed(forwardAveraged) = (smoothed(forwardAveraged) + smoothed(forwardAveraged + 1)) / 2;
%Do the backward averaging.
% smoothed(backwardAveraged) = (smoothed(backwardAveraged) + smoothed(backwardAveraged - 1)) / 2;
for i = 1 : length(backwardAveraged)
    smoothed(backwardAveraged(i)) = (smoothed(backwardAveraged(i)) + smoothed(backwardAveraged(i) - 1)) / 2;
end

%Move the extrema, if allowed.
% if ~invariantAmplitude
%     extrema = [minima' maxima'];
% 
%     first = extrema(find(extrema <= 1));
%     if ~isempty(first) & polarity == 1 | polarity == 2
%         smoothed(first) = (smoothed(first) + smoothed(first + 1)) / 2;
%     end
%     last = extrema(find(extrema >= length(smoothed)));
%     if ~isempty(last) & polarity == 1 | polarity == 3ward
%         smoothed(last) = (smoothed(last) + smoothed(last - 1)) / 2;
%     end
%     
%     extrema = extrema([find(extrema > 1) find(extrema < length(smoothed))]);
%     
%     forward = extrema(find(extrema <= 1));
%     backward = extrema(find(extrema > length(smoothed)));
% 
%     if polarity == 1
%         %Bidirectional
%     elseif polarity == 2
%         %Forward
%     elseif polarity == 3
%         %Backward
%     end
%     
% %     extrema = extrema(find(extrema ~= forward & extrema ~= backward));
% 
%     forward = [forward' find(abs(smoothed(extrema) - smoothed(extrema + 1)) > ...
%             abs(smoothed(extrema) - smoothed(extrema - 1)))'];
%     backward = [backward' find(abs(smoothed(extrema) - smoothed(extrema - 1)) > ...
%             abs(smoothed(extrema) - smoothed(extrema + 1)))'];
% 
%     smoothed(forward) = (smoothed(forward) + smoothed(forward + 1)) / 2;
%     smoothed(backward) = (smoothed(backward) + smoothed(backward - 1)) / 2;
% 
% %     smoothed(extrema) = (forwardWeight .* (smoothed(extrema) + smoothed(extrema + 1)) / 2 + ...
% %          backWeight .* (smoothed(extrema) + smoothed(extrema - 1)) / 2) / 2;
% end

if plotOn
    figure;
    plot(smoothed);
    
    hold on;

    plot(minima, ones(length(minima), 1) * smn, 'or');
    plot(maxima, ones(length(maxima), 1) * smx, 'og');

    plot(forwardAveraged, smoothed(forwardAveraged), '>k');
    plot(backwardAveraged, smoothed(backwardAveraged), '<k');
    
    xlabel('Samples');
    ylabel('Amplitude');
    title(sprintf('smoothextrema: Output (windowsize: %s)', num2str(windowSize)));
    legend('Smoothed-Signal', 'Minima', 'Maxima', 'ForwardAveraged', 'BackwardAveraged');
    
    hold off;
    
    figure;
    
    ddtRaw = mean(diff(diff(signal)));
    ddtSmoothed = mean(diff(diff(smoothed)));
    plot(1:length(signal), signal, '.:', 1:length(smoothed), smoothed, '.:');
%     set(get(gca, 'Children'), 'MarkerSize', 3);
    legend('Raw Signal', 'Smoothed Signal');
    xlabel('Samples');
    ylabel('Amplitude');
    title(sprintf('SmoothExtrema (windowsize: %s)\n     Change in \\delta^2/\\deltat^2: %s%%', ...
        num2str(windowSize), num2str((ddtRaw - ddtSmoothed) / ddtRaw)));
end


return;