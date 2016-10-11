function setAITriggerType(aiobj,trig_type )
%SETATRIGGERTYPE Sets trigger type of AI object in a manner compatible with DAQmx and traditional NI-DAQ
% DESCRIPTION 
%   In DAQmx, a "TriggerCondition" property is exposed when the TriggerType is "HWDigital". This should generally be set to "PostiveEdge"
%   but defaults instead to "NegativeEdge". This function sets the TriggerCondition property correctly whenever TriggerType is set to
%   "HWDigital" in DAQmx.


set(aiobj,'TriggerType',trig_type);

if strcmpi(trig_type,'HWDigital')
    if ~isempty(strfind(daqhwinfo('nidaq','AdaptorDllName'),'mwnidaqmx.dll'))
%        set(aiobj,'TriggerCondition','PositiveEdge');
    end
end

        
