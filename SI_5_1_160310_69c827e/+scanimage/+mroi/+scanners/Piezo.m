classdef Piezo < handle
    % The following are taken from the LSC Pure Analog Controller, but seem
    % like reasonable things for piezo. Add more from FastZ as necessary.
    properties
        sampleRateHz;
        commandVoltsPerMicron; % Conversion factor for command signal to analog linear stage controller
        sensorVoltsPerMicron;  % Conversion signal for sensor signal from analog linear stage controller
        commandVoltsOffset;    % Offset value, in volts, for command signal to analog linear stage controller
        sensorVoltsOffset;     % Offset value, in volts, for sensor signal from analog linear stage controller
    end
    
    methods(Static)
        function obj = default
            obj=scanimage.mroi.scanners.Piezo(1,1,0,0);
        end
    end
    
    methods
        function obj=Piezo(commandVoltsPerMicron,sensorVoltsPerMicron,commandVoltsOffset,sensorVoltsOffset,sampleRateHz)
            obj.commandVoltsPerMicron = commandVoltsPerMicron;
            obj.sensorVoltsPerMicron   = sensorVoltsPerMicron;
            obj.commandVoltsOffset = commandVoltsOffset;
            obj.sensorVoltsOffset  = sensorVoltsOffset;
            obj.sampleRateHz  = sampleRateHz;
        end
    end
end


%--------------------------------------------------------------------------%
% Piezo.m                                                                  %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
