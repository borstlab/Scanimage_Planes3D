classdef triggerRouteRegistry < handle
    % Helper class to manage DAQmx trigger routes
    % This class is used to track routes that are connected via an objects
    % lifetime - on deletion of this object all registered routes are
    % disconnected
    properties
        verbose = false;
    end
    
    properties (SetAccess = private)
        routes;
    end
    
    properties (Access = private)
        daqSys;
    end
    
    %% Lifecycle
    methods
        function obj = triggerRouteRegistry()
            obj.daqSys = dabs.ni.daqmx.System();
            obj.routes= {};
        end
        
        function delete(obj)
            if most.idioms.isValidObj(obj.daqSys)
                obj.clearRoutes();
            end
        end
    end
    
    %% User methods
    methods
        function connectTerms(obj,src,dest)
            if ~strcmpi(src,dest)
                obj.daqSys.connectTerms(src,dest);
                obj.fprintf('Connecting Terms: %s -> %s\n',src,dest);
                obj.addRoute(src,dest);
            end
        end
        
        function disconnectTerms(obj,src,dest)
            if ~strcmpi(src,dest)
                obj.daqSys.disconnectTerms(src,dest)
                if strfind(dest,'PFI')
                    obj.daqSys.tristateOutputTerm(dest);
                end
                obj.fprintf('Disonnecting Terms: %s -> %s\n',src,dest);
                obj.removeRoute(src,dest);
            end
        end
        
        function reinitRoutes(obj)
            routes_ = obj.routes;
            for idx = 1:size(routes_,1)
                try
                    src = routes_{idx,1};
                    dest = routes_{idx,2};
                    obj.connectTerms(src,dest);
                catch ME
                    most.idioms.reportError(ME);
                end
            end            
        end
        
        function deinitRoutes(obj)
            routes_ = obj.routes;
            for idx = 1:size(routes_,1)
                try
                    src = routes_{idx,1};
                    dest = routes_{idx,2};
                    obj.daqSys.disconnectTerms(src,dest);
                    if strfind(dest,'PFI')
                        obj.daqSys.tristateOutputTerm(dest);
                    end
                    obj.fprintf('Deinit Terms: %s -> %s\n',src,dest);
                catch ME
                    most.idioms.reportError(ME);
                end
            end            
        end
        
        function clearRoutes(obj)
            for idx = 1:size(obj.routes,1)
                src = obj.routes{idx,1};
                dest = obj.routes{idx,2};
                obj.daqSys.disconnectTerms(src,dest);
            end
            
            obj.routes = {};
        end
    end
    
    %% Private methods
    methods (Access = private)        
        function idx = findRouteIdx(obj,src,dest)
            [~,idxs] = ismember(src,obj.routes);
            if idxs > 0
                [~,idx] = ismember(dest,obj.routes(idxs,2));
            end
            
            if idxs < 1 || idx < 1
                idx = [];
            else
                idx = idxs(idx);
            end
        end
        
        function addRoute(obj,src,dest)
            if isempty(obj.findRouteIdx(src,dest))
                obj.routes(end+1,:) =  {src,dest};
            end
        end
        
        function removeRoute(obj,src,dest)
            idx = obj.findRouteIdx(src,dest);
            if ~isempty(idx)
                obj.routes(idx,:) =  [];
            end
        end
        
        function fprintf(obj,varargin)
            if obj.verbose
                fprintf(varargin{:});
            end
        end
    end
end
    


%--------------------------------------------------------------------------%
% triggerRouteRegistry.m                                                   %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
