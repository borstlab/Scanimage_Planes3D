classdef scanfieldDisplay < scanimage.interfaces.Class
    %% USER PROPS
    properties
        windowTitle = 'Scanfield Display';
        visible = true;
        scanfields = [];
    end
    
    %% INTERNAL PROPS
    properties (Hidden, SetAccess=?scanimage.interfaces.Class)
        hDisplay;
        hFig;
        hAxes;
        
        frameNums;
    end
    
    %% PROP ACCESS
    methods
        function set.windowTitle(obj,v)
            obj.windowTitle = v;
            set(obj.hFig, 'Name', obj.windowTitle);
        end
        
        function set.visible(obj,v)
            obj.visible = v;
            if v
                obj.resetDisplay();
            else
                set(obj.hFig,'Visible','off');
            end
        end
        
        function set.scanfields(obj,v)
            obj.scanfields = v;
            if obj.visible
                obj.resetDisplay();
            end
        end
    end
    
    %% LIFECYCLE
    methods
        function obj = scanfieldDisplay(hDisplay)
            obj.hDisplay = hDisplay;
            
            obj.hFig = most.idioms.figureSquare('Name',obj.windowTitle,'Visible','off',...
                    'ColorMap',gray(255),'NumberTitle','off','Menubar','none',...
                    'CloseRequestFcn',@obj.lclDisplayFigCloseEventHandler);
            
            if ~isempty(obj.hDisplay.hController)
                ctrler = obj.hController{1};
                ctrler.registerGUI(obj.hFigs);
            end
        end
        
        function delete(obj)
            delete(obj.hFig);
        end
    end
    
    %% USER METHODS
    methods
        function resetDisplay(obj)
            numTiles = numel(obj.scanfields);
            obj.hAxes = {};
            if numTiles
                
                clf(obj.hFig);
                set(obj.hFig,'HandleVisibility','callback');
                set(0, 'CurrentFigure', obj.hFig);
                
                if numTiles > 1
%                     hGrid = uigridcontainer('v0','Parent',hFig,'Units','norm','Position',[0 0 1 1],'Margin',2);
%                     set(hGrid,'GridSize',[round(sqrt(numTiles)) ceil(numTiles/columns)]);
%                     
%                     for tileIdx=1:numTiles
%                         obj.hAxes{tileIdx} = scanimage.mroi.RoiDisplay(hGrid);
%                         
%                         if obj.hSI.hRoiManager.mroiEnable
%                             if obj.displayTiling
%                                 obj.hAxes{tileIdx}.zs = zs(obj.displayFrameBatchSelection(tileIdx));
%                                 obj.hAxes{tileIdx}.initialize(obj.hSI,zs(obj.displayFrameBatchSelection(tileIdx)),displayMode,...
%                                     obj.roiDisplayEdgeColor,obj.roiDisplayEdgeAlpha,obj.roiProjectionDisplayEdgeColor,obj.roiProjectionDisplayEdgeAlpha);
%                             end
%                         else
%                             obj.hAxes{tileIdx}.zs = zs(obj.displayFrameBatchSelection(tileIdx));
%                             obj.hAxes{tileIdx}.initialize(obj.hSI,zs(obj.displayFrameBatchSelection(tileIdx)),displayMode,...
%                                 obj.roiDisplayEdgeColor,obj.roiDisplayEdgeAlpha,obj.roiProjectionDisplayEdgeColor,obj.roiProjectionDisplayEdgeAlpha);
%                         end
%                     end
                else
                    obj.hAxes{1} = scanimage.mroi.RoiDisplay(obj.hFig);
                    obj.hAxes{1}.initialize(obj.hDisplay.hSI,obj.scanfields(1).z,'no_transform');
                end
            else
                clf(obj.hFig);
            end
            
            if obj.visible
                set(obj.hFig,'Visible','on');
            end
        end
        
        function updateDisplay(obj)
            for i = 1:numel(obj.scanfields)
                try
                    sf = obj.scanfields(i);
                    obj.hAxes{i}.drawRoiData(obj.hDisplay.rollingStripeDataBuffer{obj.frameNums(i)}{1}.roiData{sf.roiId},sf.channel);
                catch ME
%                     ME.message
                end
            end
        end
    end
    
    %% INTERNAL METHODS
    methods (Hidden)
        function lclDisplayFigCloseEventHandler(obj,src,evnt) 
            set(src,'Visible','off');
            obj.visible = false;
        end
        
        function frames = cacheData(obj)
            for i = 1:numel(obj.scanfields)
                obj.frameNums(i) = find(obj.hDisplay.displayZs == obj.scanfields(i).z);
            end
            
            frames = unique(obj.frameNums);
        end
    end
end



%--------------------------------------------------------------------------%
% scanfieldDisplay.m                                                       %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
