classdef Beams < handle
    properties
        beamIDs;
        sampleRateHz;
        
        powers;
        powerLimits;
        flybackBlanking;
        
        pzAdjust;
        Lzs;
        
        interlaceDecimation;
        interlaceOffset;
        
        powerFracToVoltageFunc;
        
        linePhase;          % [seconds]
        beamClockDelay;     % [us]
        beamClockExtend;    % [us]
        
        powerBoxes = [];
    end

    methods(Static)
        function obj = default
            obj=scanimage.mroi.scanners.Beams(1,200000,0,100,true,false,Inf,1,0,[],0,0,0);
        end
    end

    methods
        function obj=Beams(...
                beamIDs,...
                sampleRateHz,...
                powers,...
                powerLimits,...
                flybackBlanking,...
                pzAdjust,...
                Lzs,...
                interlaceDecimation,...
                interlaceOffset,...
                powerFracToVoltageFunc,...
                linePhase,...
                beamClockDelay,...
                beamClockExtend,...
                powerBoxes)
            
            obj.beamIDs     = beamIDs;
            obj.sampleRateHz = sampleRateHz;
            obj.powers = powers;
            obj.powerLimits = powerLimits;
            obj.flybackBlanking = flybackBlanking;
            obj.pzAdjust = pzAdjust;
            obj.Lzs = Lzs;
            obj.interlaceDecimation = interlaceDecimation;
            obj.interlaceOffset = interlaceOffset;
            obj.powerFracToVoltageFunc = powerFracToVoltageFunc;
            obj.linePhase = linePhase;
            obj.beamClockDelay = beamClockDelay;
            obj.beamClockExtend = beamClockExtend;
            
            if nargin > 13
                obj.powerBoxes = powerBoxes;
            end
        end
    end
end


%--------------------------------------------------------------------------%
% Beams.m                                                                  %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
