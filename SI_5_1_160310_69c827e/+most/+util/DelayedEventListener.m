classdef DelayedEventListener < handle    
    properties
        delay;
    end
    
    properties (Access = private)
       hDelayTimer;
       functionHandle;
       hListener;
    end
    
    methods
        function obj = DelayedEventListener(delay,varargin)            
            obj.hDelayTimer = timer(...
                'TimerFcn',@obj.executeFunctionHandle,...
                'BusyMode','drop',...
                'ExecutionMode','singleShot',...
                'StartDelay',delay,...
                'ObjectVisibility','off');
            
            if ischar(varargin{end}) && strcmpi(varargin{end},'weak')
                obj.hListener = most.idioms.addweaklistener(varargin{1:end-1});
            else
                obj.hListener = addlistener(varargin{:});
            end
            obj.functionHandle = obj.hListener.Callback;
            obj.hListener.Callback = @(varargin)obj.delayFunction(varargin);
            
            listenerSourceNames = strjoin(cellfun(@(src)class(src),obj.hListener.Source,'UniformOutput',false));
            set(obj.hDelayTimer,'Name',sprintf('Delayed Event Listener Timer %s:%s',listenerSourceNames,obj.hListener.EventName));
            
            obj.delay = delay;
        end
        
        function delete(obj)
           most.idioms.safeDeleteObj(obj.hListener);
           if most.idioms.isValidObj(obj.hDelayTimer)
               stop(obj.hDelayTimer);
               most.idioms.safeDeleteObj(obj.hDelayTimer);
           end
        end
    end
    
    methods
        function delayFunction(obj,varargin)
            % restart timer
            stop(obj.hDelayTimer);
            start(obj.hDelayTimer);
        end
        
        function executeFunctionHandle(obj,varargin)
            obj.functionHandle(varargin);
        end
    end
    
    methods
        function set.delay(obj,val)
            wasRunning = strcmp(get(obj.hDelayTimer,'Running'),'on');
            stop(obj.hDelayTimer);
            set(obj.hDelayTimer,'StartDelay',val);
            obj.delay = val;
            
            if wasRunning
                start(obj.hDelayTimer);
            end
        end
    end
end


%--------------------------------------------------------------------------%
% DelayedEventListener.m                                                   %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
