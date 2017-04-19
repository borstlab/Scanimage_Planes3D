classdef RoiDisplay < handle
    properties
        zs   = [0];             % [numeric] array of zs that are viewed in this object     
        chan = 1;               % [numeric] scalar channel being displayed in this Axes object
        showCrosshair = false;  % [boolean] show/hide crosshair in display
        debugEnabled = false;   % [boolean] show/hide roiData debug in the ROI label.
        transposeImage = true;  % roiData.imageData stores images transposed
        parent;                 % parent graphic handle
        dataMultiplier = 1;
        CLim = [0,100];
    end
    

    properties (SetAccess = private, Hidden)
        hSI;
        hAxes;
        hSurfs;
        hMainSurfs;
        roiMap;
        labelMap;
        hCrossHair;
        dragdata;
        hUicMain;
        hUicFlowMain;
        hSurfContextMenu;
        hAxesContextMenu;
        hCursor;
        hOnTopGroup;
    end
    
    properties (SetAccess = private, Hidden, Dependent)
        hFig;
    end
    
    properties (Constant, Hidden)
        graphics2014b = most.idioms.graphics2014b(); % cache this for performance
    end
    
    
    methods
        function obj = RoiDisplay(parent)
            if nargin < 1 || isempty(parent)
                parent = figure();
            end
            
            obj.hUicMain = uicontainer('Parent',parent,'DeleteFcn',@(src,evt)most.idioms.safeDeleteObj(obj));
            obj.parent = parent;
            
            obj.hSurfContextMenu = uicontextmenu('Parent',obj.hFig);
                uimenu('Parent',obj.hSurfContextMenu,'Label','Reset View','Callback',@(src,evt)obj.surfContextMenuCallback(@obj.resetview));
                uimenu('Parent',obj.hSurfContextMenu,'Separator','on','Label','Hide Cursor','Callback',@(src,evt)obj.surfContextMenuCallback(@obj.hideCursor));
                uimenu('Parent',obj.hSurfContextMenu,'Label','Show Crosshair','Callback',@(src,evt)obj.surfContextMenuCallback(@obj.toggleCrosshair));
                uimenu('Parent',obj.hSurfContextMenu,'Separator','on','Label','Histogram','Callback',@(src,evt)obj.surfContextMenuCallback(@obj.showSurfHistogram));
                uimenu('Parent',obj.hSurfContextMenu,'Label','Pixel Value','Callback',@(src,evt)obj.surfContextMenuCallback(@obj.pixelValue));
                uimenu('Parent',obj.hSurfContextMenu,'Label','Image Stats','Callback',@(src,evt)obj.surfContextMenuCallback(@obj.surfImageStats));
                uimenu('Parent',obj.hSurfContextMenu,'Label','Assign image in base','Callback',@(src,evt)obj.surfContextMenuCallback(@obj.imageAssigninBase));
                uimenu('Parent',obj.hSurfContextMenu,'Label','Save to Tiff','Callback',@(src,evt)obj.surfContextMenuCallback(@obj.saveSurfToTiff));
                
            obj.hAxesContextMenu = uicontextmenu('Parent',obj.hFig);
                uimenu('Parent',obj.hAxesContextMenu,'Label','Reset View','Callback',@obj.resetview);
                uimenu('Parent',obj.hAxesContextMenu,'Label','Top View','Callback',@(src,evt)obj.resetview('top'));
                uimenu('Parent',obj.hAxesContextMenu,'Separator','on','Label','Hide Cursor','Callback',@obj.hideCursor);
                uimenu('Parent',obj.hAxesContextMenu,'Label','Show Crosshair','Callback',@obj.toggleCrosshair);
                uimenu('Parent',obj.hAxesContextMenu,'Separator','on','Label','Show Volume Histogram','Callback',@(src,evt)obj.showVolumeHistogram());
                uimenu('Parent',obj.hAxesContextMenu,'Label','Volume Stats','Callback',@(src,evt)obj.volumeImageStats());
                
            obj.hAxes = obj.prepareAxes('Parent',obj.hUicMain,'ButtonDownFcn',@obj.selectdrag,'UIContextMenu',obj.hAxesContextMenu);
            
            obj.hCursor = line('Parent',obj.hAxes,'Visible','off','LineStyle','none',...
                'Marker','+','MarkerSize',10,'MarkerEdgeColor','r','LineWidth',1,'HitTest','off');
            
            obj.hOnTopGroup = hgtransform('Parent',obj.hAxes);
            
            obj.hCrossHair = hggroup('Parent',obj.hOnTopGroup,'Visible','off','HitTest','off');
            line('XData',[0,1],'YData',[0.5,0.5],...
                'Parent',obj.hCrossHair,'Color','white','LineWidth',1);
            line('XData',[0.5,0.5],'YData',[0,1],...
                'Parent',obj.hCrossHair,'Color','white','LineWidth',1);
            obj.showCrosshair = obj.showCrosshair; % Set Visibility of cross hair according to obj.showCrosshair
            
            obj.axesSelectedByUser(obj.hAxes);
            obj.CLim = obj.CLim;
        end
        
        function delete(obj)
            most.idioms.safeDeleteObj(obj.hSurfContextMenu);
            most.idioms.safeDeleteObj(obj.hAxesContextMenu);
            most.idioms.safeDeleteObj(obj.hUicMain);
            most.idioms.safeDeleteObj(obj.hAxes);
            most.idioms.safeDeleteObj(obj.roiMap);
            most.idioms.safeDeleteObj(obj.labelMap);
            most.idioms.safeDeleteObj(obj.hCrossHair);
            most.idioms.safeDeleteObj(obj.hUicMain);
            most.idioms.safeDeleteObj(obj.hUicFlowMain);
        end
        
        function initialize(obj,hSI,zs,displayMode,surfEdgeColor,surfEdgeAlpha,surfPEdgeColor,surfPEdgeAlpha)
            if nargin < 3 || isempty(zs)
                zs = obj.zs;
            end
            
            if nargin < 4 || isempty(displayMode)
                %IFNDEF#FREE
                displayMode = '3d';
                %#ELSE
%                 displayMode = 'transform';
                %#ENDIF
            else
                displayMode = lower(displayMode);
            end
            
            if nargin < 5 || isempty(surfEdgeColor)
                surfEdgeColor = hSI.hDisplay.roiDisplayEdgeColor;
            end
            
            if nargin < 6 || isempty(surfEdgeAlpha)
                surfEdgeAlpha = hSI.hDisplay.roiDisplayEdgeAlpha;
            end
            
            if nargin < 7 || isempty(surfPEdgeColor)
                surfPEdgeColor = hSI.hDisplay.roiProjectionDisplayEdgeColor;
            end
            
            if nargin < 8 || isempty(surfPEdgeAlpha)
                surfPEdgeAlpha = hSI.hDisplay.roiProjectionDisplayEdgeAlpha;
            end
            
            obj.hSI = hSI;
            
            
            if strcmpi('no_transform',displayMode);
                if ~hSI.hRoiManager.mroiEnable && hSI.hRoiManager.forceSquarePixels && hSI.hRoiManager.scanAngleMultiplierSlow ~= 0
                    yRatio = abs(hSI.hRoiManager.linesPerFrame / hSI.hRoiManager.pixelsPerLine);
                    disp('in ROIDisplay, initialize')
                    disp(yRatio)
                else
                    yRatio = 1;
                end
            else
                yRatio = 1;
            end
            set(obj.hAxes,'PlotBoxAspectRatio',[1 yRatio 1],'DataAspectRatioMode','auto');
            
            
            obj.zs = zs;
            
            if isempty(obj.zs) || any(isnan(obj.zs))
                obj.setCrossHairZ(0-1e-6);
            else
                obj.setCrossHairZ(min(obj.zs)-1e-6);
            end
            
            delete(obj.hSurfs); % clear all existing surfaces in axis

            if obj.graphics2014b
                obj.hSurfs = gobjects(1,0);
                obj.hMainSurfs = gobjects(1,0);
            else
                obj.hSurfs = [];
                obj.hMainSurfs = [];
            end
            obj.roiMap = containers.Map('KeyType','double','ValueType','any');
            obj.labelMap = containers.Map('KeyType','double','ValueType','any');
            
            roi = hSI.hRoiManager.currentRoiGroup.rois;
            scanField = {roi.scanfields};
            if ~isempty(roi) && ~isempty(scanField)
                roiId = roi.id;
                zSurfMap = containers.Map('KeyType','double','ValueType','any');
                zLabelMap = containers.Map('KeyType','double','ValueType','any');
                
                tmpzs = obj.zs;
                if any(isnan(tmpzs))
                    assert(numel(tmpzs) == 1, 'A roi display with an indeterminate z plane can only have one z level.')
                    tmpzs = 0;
                end
                
                
                %Surface handles for Roi
                for idx = 1:length(tmpzs)
                    z = tmpzs(idx);
                    
                    [imcoordsX,imcoordsY,imcoordsZ] = meshgrid(0:1,0:1,z);

                    if obj.transposeImage
                        imcoordsX = imcoordsX';
                        imcoordsY = imcoordsY';
                    end

                    switch lower(displayMode)
                        case {'transform','3d'}
                            [imcoordsX,imcoordsY] = scanField.transform(imcoordsX,imcoordsY);
                        case 'no_transform'
                            %No Transform
                            surfEdgeColor = 'none';
                        otherwise
                            %TODO
                    end

                    imData = 0; %Unused if mroi disabled

                    surfs = struct();
                    [surfs.hSurf,centerpoint] = prepareSurface(imcoordsX,imcoordsY,imcoordsZ,imData,'Parent',obj.hAxes,'EdgeColor',surfEdgeColor,'EdgeAlpha',surfEdgeAlpha);
                    set(surfs.hSurf,'HitTest','on','ButtonDownFcn',@(src,evt)obj.clickedsurf(centerpoint,src,evt));
                    
                    
                    obj.hSurfs(end+1) = surfs.hSurf;
                    obj.hMainSurfs(end+1) = surfs.hSurf;
                    

                    zSurfMap(z) = surfs;

                    %Roi Names
%                         hLabel = text(imcoordsX(1)+0.01,imcoordsY(1),imcoordsZ(1),roi.name,...
%                             'Parent',obj.hAxes,...
%                             'FontWeight','normal',...
%                             'Color','Yellow',...
%                             'FontSize',7,...
%                             'HorizontalAlignment','Left',...
%                             'VerticalAlignment','Top');
%                         zLabelMap(z) = hLabel;

                end
                obj.roiMap(roiId) = zSurfMap;
                obj.labelMap(roiId) = zLabelMap;
            end
            
            obj.resetview(true);
            obj.CLim = obj.CLim;
            
            % nested function
            function [hSurf,centerpoint] = prepareSurface(imcoordsX,imcoordsY,imcoordsZ,imData,varargin)
                hSurf = surface(imcoordsX,imcoordsY,imcoordsZ,imData,...
                    'FaceColor','texturemap',...
                    'CDataMapping','scaled',...
                    'FaceLighting','none',...
                    'UIContextMenu',obj.hSurfContextMenu,...
                    varargin{:});
                
                centerpoint = [(imcoordsX(1,1) + imcoordsX(2,2))/2,...
                              (imcoordsY(1,1) + imcoordsY(2,2))/2,...
                              (imcoordsZ(1,1) + imcoordsZ(2,2))/2];
            end
        end
        
        function resetview(obj,varargin)
            if obj.hSI.hDisplay.needReset && (isempty(varargin) || ~isa(varargin{1},'logical') || ~varargin{1})
                obj.hSI.hDisplay.resetActiveDisplayFigs();
            else
                set(obj.hAxes,'CameraViewAngleMode','auto');
                if isempty(obj.zs) || length(obj.zs) == 1 || any(isnan(obj.zs))
                    if isempty(obj.zs) || any(isnan(obj.zs))
                        z = 0;
                    else
                        z = obj.zs(1);
                    end
                    
                    camtarget(obj.hAxes,[0.5,0.5,z]);
                    set(obj.hOnTopGroup,'Matrix',makehgtform('translate',[0 0 z-1e-6]));
                    set(obj.hAxes,'ZLim',[z-1e-6 z]);
                    
                    if obj.graphics2014b
                        view(obj.hAxes,0,90);
                    else
                        % GJ: weird workaround
                        % In tiled mode and large number of tiles, for some
                        % reason the surfaces are sometimes not visible when
                        % elevation is set to 90 degrees
                        view(obj.hAxes,0,89.998);
                    end
                else
                    if ~isempty(varargin) && ischar(varargin{1}) && strcmpi(varargin{1},'top')
                        camtarget(obj.hAxes,[0.5,0.5,min(obj.zs)]);
                        view(obj.hAxes,0,90);
                    else
                        camtarget(obj.hAxes,[0.5,0.5,(max(obj.zs)-min(obj.zs))/2]);
                        view(obj.hAxes,45,45);
                    end
                    set(obj.hOnTopGroup,'Matrix',makehgtform('translate',[0 0 min(obj.zs)-1e-6]));
                    set(obj.hAxes,'ZLim',[min(obj.zs)-1e-6 max(obj.zs)]);
                end
            end
        end
        
        function drawRoiData(obj,roiDatas,chan)
            if nargin < 3 || isempty(chan)
                chan = obj.chan;
            end
            obj.chan = chan;
            
            if isempty(roiDatas)
                return
            end
                        
            if ~iscell(roiDatas)
                roiDatas = {roiDatas};
            end
            
            for i = 1:numel(roiDatas)
                roiData = roiDatas{i};
                if roiData.hRoi.display && any(roiData.channels == obj.chan)
                    try % this try catch statement is faster than using obj.roiMap.isKey
                        zSurfMap = obj.roiMap(roiData.hRoi.id);
                    catch
                        most.mimics.warning('roiMap value for this roi ID is not valid.');
                        continue
                    end
                    
                    %zLabelMap = obj.labelMap(roiData.hRoi.id);
                    for zIdx = 1:numel(roiData.zs)
                        if isnan(obj.zs)
                            z = 0;
                        else
                            z = roiData.zs(zIdx);
                        end
                        
                        try % this try catch statement is faster than using obj.roiMap.isKey
                            surfs = zSurfMap(z); % get the right surface handle
                        catch
                            most.mimics.warning('roiData has an encoded z value of %d, but this is not a display key in RoiDisplay. Roidata ID: %d\n', z, roiData.hRoi.id);
                            continue
                        end
                        
%                             hLabel = zLabelMap(z); % get the handle to the ROI label.
                        
                        imData = roiData.imageData{roiData.channels == obj.chan}{zIdx};
                        
                        
                        surfSetCdata(surfs.hSurf,imData)
                        

                        if obj.debugEnabled
                            %display debug information on ROI
%                             labelString = [ num2str(roiData.zs) ' ' ...
%                                             num2str(roiData.frameTimestamp) ' ' ...
%                                             num2str(roiData.frameNumberAcq) ' ' ...
%                                             num2str(roiData.frameNumberAcqMode) ];
%                                 set(hLabel,'String',labelString);
                        end
                    end
                end
            end
            
            function surfSetCdata(hSurf,cData)
                if obj.graphics2014b
                    hSurf.CData = cData;
                else
                    if isa(cData,'uint8')
                        set(hSurf,'CData',cData);
                    else
                        cDataDbl = double(cData);
                        set(hSurf,'CData',cDataDbl);
                    end
                end
            end
            
            function surfSetAlphaData(hSurf,alphaData)
                if size(alphaData,3) > 1
                    % RGB merge display cannot be used with transparency
                    % deactivate FaceAlpha by seting surf to opaque
                    set(hSurf,'FaceAlpha',1);
                    return
                end
                
                if obj.graphics2014b
                    hSurf.AlphaData = alphaData;
                else
                    if isa(alphaData,'uint8')
                        % todo: handle merge!
                        set(hSurf,'AlphaData',alphaData);
                    else
                        cDataDbl = double(alphaData);
                        set(hSurf,'AlphaData',cDataDbl);
                    end
                end
            end
        end
        
        
        function hAx = prepareAxes(obj,varargin)
            hAx = axes(...
                'Box','off',...
                'NextPlot','add',...
                'XLimMode','manual',...
                'YLimMode','manual',...
                'ZLimMode','manual',...
                'DataAspectRatio',[1 1 1],...
                'XLim',[0 1],...
                'YLim',[0 1],...
                'ZLim',[-Inf Inf],...
                'Color','black',...
                'Position',[0 0 1 1],...
                'YDir','reverse',...
                'ZDir','reverse',...
                'XTick',[],'YTick',[],'ZTick',[],...
                'XTickLabelMode','manual','YTickLabelMode','manual','ZTickLabelMode','manual',...
                'XTickLabel',[],'YTickLabel',[],'ZTickLabel',[],...
                'CLim',[0 1],...
                varargin{:});
        end
    end
    
    methods        
        function val = get.hFig(obj)
            val = ancestor(obj.hUicMain,'figure');
        end
        
        function set.parent(obj,val)
           set(obj.hUicMain,'Parent',val);
        end
        

        function resetScanFields(obj)
            % sets all scanFields back to black            
            for hSurf = obj.hSurfs
                set(hSurf,'AlphaData',0,'CData',0);
            end
        end
        
        function setCrossHairZ(obj,z)
           hLines = get(obj.hCrossHair,'Children');
           for hLine = hLines(:)'
               zData = get(hLine,'ZData');
               set(hLine,'ZData',ones(size(zData)).*z);
           end
        end
    end
    
    methods
        
        function set.dataMultiplier(obj,val)
            obj.dataMultiplier = double(val);
            obj.CLim = obj.CLim;
        end
        
        function set.CLim(obj,val) 
            correctedVal = double(val) .* obj.dataMultiplier;
            set(obj.hAxes,'CLim',correctedVal);
            
            
            obj.CLim = val;
        end
        
        function set.showCrosshair(obj,val)
            if val
                visibleOnOff = 'on';
            else
                visibleOnOff = 'off';
            end
            
            if ~isempty(obj.hCrossHair) && ishandle(obj.hCrossHair)
                set(obj.hCrossHair,'Visible',visibleOnOff);
				if ~obj.graphics2014b
                    % workaround for Matlab<2014b to hide crosshair
                    set(get(obj.hCrossHair,'Children'),'Visible',visibleOnOff);
                end
                
                % check / uncheck menu item
                mnu = findall(obj.hSurfContextMenu,'Label','Show Crosshair');
                mnu = [mnu findall(obj.hAxesContextMenu,'Label','Show Crosshair')];
                set(mnu,'Checked',visibleOnOff);
            end
            
            obj.showCrosshair = val;
        end
    end
    
    %% 3d mouse navigation functions 
    methods
        function axesSelectedByUser(obj,hAx)
            set(obj.hFig,'WindowScrollWheelFcn',@(src,evt)obj.scrollWheelFcn(hAx,src,evt));
        end
        
        function scrollWheelFcn(obj,hAx,~,evt)
            zoomSpeedFactor = 1.1;
            cAngle = get(hAx,'CameraViewAngle');
            scroll = zoomSpeedFactor ^ double(evt.VerticalScrollCount);
            cAngle = cAngle * scroll;
            set(hAx,'CameraViewAngle',cAngle);
        end
        
        function pt = getPoint(obj)
            pt = hgconvertunits(obj.hFig,[0 0 get(obj.hFig,'CurrentPoint')],...
				get(obj.hFig,'Units'),'pixels',0);
            pt = pt(3:4);
        end
        
        function clickedsurf(obj,surfcenter,src,evt)
            hAx = ancestor(src,'axes');
            switch get(obj.hFig,'SelectionType');
                case 'open'   % double click
                    obj.resetview();
                otherwise
                    obj.selectdrag(src,evt);
            end
            obj.axesSelectedByUser(hAx);
        end
        
        function selectdrag(obj,src,evt)
           switch get(obj.hFig,'SelectionType');
               case 'normal' % left click
                   obj.startdrag(@obj.dolly);
               case 'alt'    % right click
                   % reserved for context menu
               case 'open'   % double click
               case 'extend' % scroll wheel click
           end
        end
        
        function startdrag(obj,dragtype)
            pt = obj.getPoint();
            obj.dragdata = struct(...
                'figStartPoint',pt,...
                'figLastPoint',pt);
            set(obj.hFig,...
                'WindowButtonMotionFcn',@(src,evt)obj.motion(dragtype,src,evt),...
                'WindowButtonUpFcn',@obj.stopdrag);
            waitfor(obj.hFig,'WindowButtonMotionFcn',[]);
        end
        
        function motion(obj,dragtype,src,evt)
            pt = obj.getPoint();
            deltaPix = pt - obj.dragdata.figLastPoint;
            obj.dragdata.figLastPoint = pt;            
            dragtype(deltaPix);
        end
        
        function stopdrag(obj,src,~)
            set(src,...
                'WindowButtonMotionFcn',[],...
                'WindowButtonUpFcn'    ,[]);
            obj.dragdata = [];
        end
        
        function pan(obj,deltaPix)
            panxy = -deltaPix*camva(obj.hAxes)/500;
            campan(obj.hAxes,panxy(1),panxy(2),'camera',[0 0 1]);
        end
        
        
        function dolly(obj,deltaPix)
            set(obj.hAxes,'CameraViewAngleMode','manual');
            camdolly(obj.hAxes,-deltaPix(1),-deltaPix(2),0,'movetarget','pixels');
        end        
    end
    
    %% Surf UI Context Menu Callbacks
    methods
        function surfContextMenuCallback(obj,fcn)
            hSurf = gco(obj.hFig);
            if ~isempty(hSurf) && strcmpi(get(hSurf,'type'),'surface')
                fcn(hSurf);
            end
            
            if isvalid(obj) % when axes is reset by fcn, obj might get deleted
                obj.axesSelectedByUser(obj.hAxes);
            end
        end
        
        function showSurfHistogram(obj,hSurf)
            data = obj.getSurfCData(hSurf);
            obj.showHistogram(data);
        end
        
        function showVolumeHistogram(obj)
            data = obj.getVolumeData();
            obj.showHistogram(data);
        end
        
        function showHistogram(obj,data)
            if ~isempty(data)
                hFig = figure('DoubleBuffer','on','color','w',...
                    'NumberTitle','off','Name','Pixel Histogram',...
                    'PaperPositionMode','auto','PaperOrientation','landscape', 'HandleVisibility', 'callback');
                hAx = axes('Parent',hFig);
                hist(hAx,double(data(:)),256);
                set(get(hAx,'XLabel'),'String','Pixel Intensity','FontWeight','bold','FontSize',12);
                set(get(hAx,'YLabel'),'String','Number of Pixels','FontWeight','bold','FontSize',12);
            end
        end
        
        function pixelValue(obj,hSurf)            
            [point,pixel,pixelVal] = obj.getClickedSurfPixel(hSurf);

            if ~isempty(point) && ~isempty(pixel) && ~isempty(pixelVal)      
                s = struct();
                s.index = pixel;
                s.position = point(:)';
                s.value = pixelVal(:)';
            
                assignin('base','Pixel',s);
                evalin('base','Pixel');
                
                set(obj.hCursor,'Parent',ancestor(hSurf,'axes'),...
                    'XData',point(1),'YData',point(2),'ZData',point(3)-1e-6,'Visible','on');
            end
        end
        
        function surfImageStats(obj,hSurf)
            data = obj.getSurfCData(hSurf);
            obj.imageStats(data);
        end
        
        function volumeImageStats(obj)
            data = obj.getVolumeData();
            obj.imageStats(data);
        end
        
        function imageStats(obj,data)
            if isempty(data)
                return
            end
            
            data = double(data); % std requires floating point type
            
            s = struct();
            s.mean = mean(data(:));
            s.std = double(std(data(:)));
            s.max = max(data(:));
            s.min = min(data(:));
            s.pixels = numel(data);
            
            assignin('base','ImageStats',s);
            evalin('base','ImageStats');
        end
        
        function hideCursor(obj,varargin)
            set(obj.hCursor,'Visible','off');
        end
        
        function imageAssigninBase(obj,hSurf)            
            assignin('base','ImageData',obj.getSurfCData(hSurf));
            fprintf('Assigned ImageData in base\n');
        end
        
        function saveSurfToTiff(obj,hSurf,filename)
            imgdata = obj.getSurfCData(hSurf);
            
            if nargin < 3 || isempty(filename)
                [filename,pathname] = uiputfile('.tif','Choose path to save tif','image.tif');
                if filename==0;return;end
                filename = fullfile(pathname,filename);
            end
            
            if isa(imgdata,'uint8') && size(imgdata,3)==3
                photometric = Tiff.Photometric.RGB;
                sampleFormat = Tiff.SampleFormat.UInt;
                samplesPerPixel = 3;
                bitsPerSample = 8;
            else
                imgdata = int16(imgdata);
                photometric = Tiff.Photometric.MinIsBlack;
                sampleFormat = Tiff.SampleFormat.Int;
                samplesPerPixel = 1;
                bitsPerSample = 16;
            end
            
            hTif = Tiff(filename,'w');
            try
                tagstruct.ImageLength = size(imgdata,1);
                tagstruct.ImageWidth = size(imgdata,2);
                tagstruct.Photometric = photometric;
                tagstruct.BitsPerSample = bitsPerSample;
                tagstruct.SamplesPerPixel = samplesPerPixel;
                tagstruct.SampleFormat = sampleFormat;
                tagstruct.RowsPerStrip = 16;
                tagstruct.PlanarConfiguration = Tiff.PlanarConfiguration.Chunky;
                tagstruct.Software = 'ScanImage';
                hTif.setTag(tagstruct);

                hTif.write(imgdata);
            catch ME
                most.idioms.reportError(ME);
                hTif.close();
            end
            hTif.close();
        end
        
        function toggleCrosshair(obj,varargin)
            obj.showCrosshair = ~obj.showCrosshair;
        end
        
        function CData = getSurfCData(obj,hSurf)
            CData = get(hSurf,'CData');
            CData = CData ./ cast(obj.dataMultiplier,'like',CData);
            if obj.transposeImage
                CData = permute(CData,[2,1,3]);
            end
        end
        
        function data = getVolumeData(obj)
           CDatas = arrayfun(@(hSurf)obj.getSurfCData(hSurf),obj.hMainSurfs,'UniformOutput',false);
           CDatas = cellfun(@(CData)CData(:),CDatas,'UniformOutput',false);
           data = vertcat(CDatas{:});
        end
        
        function [point,pixel,pixelVal] = getClickedSurfPixel(obj,hSurf)
            point = [];
            pixel = [];
            pixelVal = [];
            
            hAx = ancestor(hSurf,'axes');
            r = get(hAx,'CurrentPoint')';
            
            xx = get(hSurf,'XData');
            yy = get(hSurf,'YData');
            zz = get(hSurf,'ZData');
            
            if obj.transposeImage
                xx = xx';
                yy = yy';
                zz = zz';
            end
            
            pp = [xx(1,1);yy(1,1);zz(1,1)];
            v1 = [xx(1,1)-xx(2,1);yy(1,1)-yy(2,1);zz(1,1)-zz(2,1)];
            v2 = [xx(1,1)-xx(1,2);yy(1,1)-yy(1,2);zz(1,1)-zz(1,2)];
            n = -cross(v1,v2);
            
            pl = r(:,1);
            l = r(:,1) - r(:,2);
            
            if dot(l,n) ~= 0
                d = dot(pp-pl,n)/dot(l,n);
                point = d*l+pl;
            else
                return % surface and view plane are perpendicular
            end
            
            p = point - pp;            
            vv1 = dot(p,v1./norm(v1));
            vv2 = dot(p,v2./norm(v2));
            
            data = obj.getSurfCData(hSurf);
            if ~isempty(data)
                pixel(1) = min(size(data,1),max(1,round(norm(vv1)/norm(v1) * size(data,1))));
                pixel(2) = min(size(data,2),max(1,round(norm(vv2)/norm(v2) * size(data,2))));
                pixelVal = data(pixel(1),pixel(2),:);
            end
        end 
    end
end


%--------------------------------------------------------------------------%
% RoiDisplay.m                                                             %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
