classdef EventData < event.EventData
    %EVENTDATA ScanImage EventData class -- contains additional property with scimData variable supplied by ScanImage event notifiers
    %
    % NOTES
    %   scimData can be a structure, e.g. for cases where event notification includes 2 or more values to pass to listeners
    
    properties
        scimData; %ScanImage-supplied event data, using the scanimage.EventManager.notifySmart() method
    end
    
    methods
        function obj = EventData(scimData) 
            if nargin
                obj.scimData = scimData;
            end
        end        
    end
   
end

