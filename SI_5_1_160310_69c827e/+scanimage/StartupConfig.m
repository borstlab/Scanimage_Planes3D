classdef StartupConfig < most.HasClassDataFile
    properties
        hFig;
        mdfPath;
        usrPath;
        runSI = false;
    end
    
    %control handles
    properties
        etMdfPath;
        etUsrPath;
        
        lastCSusr;
    end
    
    methods
        function obj = StartupConfig(mdfPath, usrPath)
            
            obj.loadLastCSdata();
            obj.ensureClassDataFile(struct('lastUsrFile',obj.lastCSusr));
            if isempty(usrPath)
                usrPath = obj.getClassDataVar('lastUsrFile');
            end
            if strcmp(usrPath, '.usr')
                usrPath = '';
            end
            
            %figure out center
            p = most.gui.centeredScreenPos([80, 15.3846]);
            
            obj.hFig = figure(...
                'Units','characters',...
                'Color',[0.941176470588235 0.941176470588235 0.941176470588235],...
                'MenuBar','none',...
                'Name','ScanImage Configuration',...
                'NumberTitle','off',...
                'Position',p,...
                'Resize','off',...
                'Visible','off',...
                'KeyPressFcn', @obj.keyFcn);
            
            mdfPanel = uipanel(...
                'Parent',obj.hFig,...
                'Units','characters',...
                'Title','Machine Data File',...
                'Clipping','on',...
                'Position',[1.6 9.3846153846154 76 5.61538461538462],...
                'Tag','uipanel1');
            
            uicontrol(...
                'Parent',mdfPanel,...
                'Units','characters',...
                'Callback',@(varargin)obj.pbBrowseMdf(varargin{1}),...
                'Position',[1.6 0.538461538461542 13.8 1.69230769230769],...
                'String','Browse...',...
                'TooltipString','Select an existing machine data file from disk.',...
                'Tag','pbBrowseMdf');
            
            uicontrol(...
                'Parent',mdfPanel,...
                'Units','characters',...
                'Callback',@(varargin)obj.pbCreateMdf(varargin{1}),...
                'Position',[15.8 0.538461538461542 13.8 1.69230769230769],...
                'String','New...',...
                'TooltipString','Create a new machine data file.',...
                'Tag','pbCreateMdf');
            
            uicontrol(...
                'Parent',mdfPanel,...
                'Units','characters',...
                'Callback',@(varargin)obj.pbEditMdf(varargin{1}),...
                'Position',[30 0.538461538461542 13.8 1.69230769230769],...
                'String','Modify...',...
                'TooltipString','Modify the selected machine data file.',...
                'Tag','pbEditMdf');
            
            obj.etMdfPath = uicontrol(...
                'Parent',mdfPanel,...
                'Units','characters',...
                'BackgroundColor',[1 1 1],...
                'HorizontalAlignment','left',...
                'Position',[1.6 2.38461538461539 71.2 1.69230769230769],...
                'String',mdfPath,...
                'Style','edit',...
                'Tag','etMdfPath',...
                'KeyPressFcn', @(varargin)obj.clearFocus(varargin{1}));
            
            usrPanel = uipanel(...
                'Parent',obj.hFig,...
                'Units','characters',...
                'Title','User Settings File',...
                'Clipping','on',...
                'Position',[1.6 3.07692307692308 76.2 5.61538461538462],...
                'Tag','uipanel2');
            
            uicontrol(...
                'Parent',usrPanel,...
                'Units','characters',...
                'Callback',@(varargin)obj.pbBrowseUsr(varargin{1}),...
                'Position',[1.6 0.538461538461542 13.8 1.69230769230769],...
                'String','Browse...',...
                'TooltipString','Select a user settings file from disk.',...
                'Tag','pbBrowseUsr');
            
            obj.etUsrPath = uicontrol(...
                'Parent',usrPanel,...
                'Units','characters',...
                'BackgroundColor',[1 1 1],...
                'HorizontalAlignment','left',...
                'Position',[1.6 2.38461538461539 71.2 1.69230769230769],...
                'String',usrPath,...
                'Style','edit',...
                'Tag','etUsrPath',...
                'KeyPressFcn', @(varargin)obj.clearFocus(varargin{1}));
            
            uicontrol(...
                'Parent',obj.hFig,...
                'Units','characters',...
                'Callback',@(varargin)obj.pbLoadSI(),...
                'FontWeight','bold',...
                'Position',[1.8 0.769230769230769 13.8 1.69230769230769],...
                'String','Continue',...
                'TooltipString','Load scanimage with the selected configuration.',...
                'Tag','pbLoadSI');
            
            uicontrol(...
                'Parent',obj.hFig,...
                'Units','characters',...
                'Callback',@(varargin)obj.pbAbortSI(),...
                'Position',[16.2 0.76923076923077 13.8 1.69230769230769],...
                'String','Cancel',...
                'TooltipString','Abort loading of scanimage application.',...
                'Tag','pbAbortSI');
            
            set(obj.hFig,'Visible','on');
        end
        
        function pbLoadSI(obj)
            obj.mdfPath = get(obj.etMdfPath, 'String');
            obj.usrPath = get(obj.etUsrPath, 'String');
            
            if ~logical(exist(obj.mdfPath,'file'))
                errordlg('Specified machine data file not found.','ScanImage');
                return
            end
            
            if isempty(obj.usrPath)
                obj.usrPath = '.usr';
            else
                if ~logical(exist(obj.usrPath,'file'))
                    errordlg('Specified user file not found.','ScanImage');
                    return
                end
            end
            
            obj.runSI = true;
            delete(obj.hFig);
            drawnow
        end
        
        function pbAbortSI(obj)
            delete(obj.hFig);
        end
        
        function pbBrowseMdf(obj, uiObj)
            mdfpath = get(obj.etMdfPath, 'String');
            
            if isempty(mdfpath)
                mdfpath = '*.m';
            end
            
            [mdffile, mdfpath] = uigetfile(mdfpath,'Select machine data file...');
            if ~isequal(mdffile,0) && ~isequal(mdfpath,0)
                set(obj.etMdfPath, 'String', fullfile(mdfpath,mdffile));
            end
            
            obj.clearFocus(uiObj);
        end
        
        function pbCreateMdf(obj, uiObj)
            [mdffile, mdfpath] = uiputfile('*.m','Save new machine data file...');
            if ~isequal(mdffile,0) && ~isequal(mdfpath,0)
                mdfPth = fullfile(mdfpath,mdffile);
                fid = fopen(mdfPth,'a');
                most.MachineDataFile.writeHeader(fid);
                fclose(fid);
                set(obj.etMdfPath, 'String', mdfPth);
            end
            
            obj.clearFocus(uiObj);
        end
        
        function pbEditMdf(obj, uiObj)
            obj.mdfPath = get(obj.etMdfPath, 'String');
            if ~logical(exist(obj.mdfPath,'file'))
                errordlg('Specified machine data file not found.','ScanImage');
            else
                edit(obj.mdfPath)
            end
            
            obj.clearFocus(uiObj);
        end
        
        function pbBrowseUsr(obj, uiObj)
            pth = get(obj.etUsrPath, 'String');
            if isempty(pth)
                pth = obj.lastCSusr;
            end
            if isempty(pth)
                pth = '*.usr';
            end
            
            [usrfile, usrpath] = uigetfile(pth,'Select machine data file...');
            if ~isequal(usrfile,0) && ~isequal(usrpath,0)
                set(obj.etUsrPath, 'String', fullfile(usrpath,usrfile));
            end
            
            obj.clearFocus(uiObj);
        end
        
        function keyFcn(obj,~,evt)
            switch evt.Key
                case 'return'
                    obj.pbLoadSI();
                    
                case 'escape'
                    obj.pbAbortSI();
            end
        end
        
        function clearFocus(~,uiObj)
            set(uiObj, 'Enable', 'off');
            drawnow update;
            set(uiObj, 'Enable', 'on');
        end
    end
    
    methods (Hidden)
        function loadLastCSdata(obj)
            classPrivatePath = most.util.className('scanimage.components.ConfigurationSaver','classPrivatePath');
            classNameShort   = most.util.className('scanimage.components.ConfigurationSaver','classNameShort');
            classDataFileName = fullfile(classPrivatePath, [classNameShort '_classData.mat']);
            if exist(classDataFileName, 'file')
                e = load(classDataFileName);
                try
                    obj.lastCSusr = e.lastUsrFile;
                catch
                    obj.lastCSusr = '';
                end
            end
        end
    end
    
    methods (Static)
        function [mdfPath, usrPath, runSI] = doModalConfigPrompt(mdfPath, usrPath)
            
            if nargin < 1 || isempty(mdfPath)
                mdfPath = '';
                runSI = false;
                
                %find last mdf
                classPrivatePath = most.util.className('most.HasMachineDataFile','classPrivatePath');
                classNameShort   = most.util.className('most.HasMachineDataFile','classNameShort');
                classDataFileName = fullfile(classPrivatePath, [classNameShort '_classData.mat']);
                if exist(classDataFileName, 'file')
                    e = load(classDataFileName);
                    try
                        mdfPath = e.lastMachineDataFilePath;
                    catch
                    end
                end
                
                if ~exist(mdfPath, 'file')
                    a = questdlg(['A previously loaded machine data file was not found. If this is the first time running '...
                        'ScanImage, a machine data file must be created. Select "create" to do this now or select "browse"'...
                        ' to locate an existing machine data file.'],'ScanImage','Create...','Browse...','Cancel','Create...');
                    
                    switch a
                        case 'Create...'
                            [mdffile, mdfpath] = uiputfile('Machine_Data_File.m','Save new machine data file...');
                            if isequal(mdffile,0) || isequal(mdfpath,0)
                                return;
                            end
                            mdfPath = fullfile(mdfpath,mdffile);
                            fid = fopen(mdfPath,'a');
                            most.MachineDataFile.writeHeader(fid);
                            fclose(fid);
                            usrPath = '.usr';
                            runSI = true;
                            uiwait(msgbox(['ScanImage will now run and generate a machine data file one section at a time. For each section, ScanImage will prompt you to modify the file and continue. '...
                                'Be sure to save the file after modifying each time, then select "OK" at the prompt to continue loading ScanImage.'], 'ScanImage'));
                            return;
                            
                        case 'Browse...'
                            [mdffile, mdfpath] = uigetfile('*.m','Select machine data file...');
                            if isequal(mdffile,0) || isequal(mdfpath,0)
                                return;
                            end
                            mdfPath = fullfile(mdfpath,mdffile);
                            
                        case 'Cancel'
                            return;
                    end
                end
            end
            
            if nargin < 2 || isempty(usrPath)
                usrPath = '';
            end
            
            obj = scanimage.StartupConfig(mdfPath, usrPath);
            
            waitfor(obj.hFig);
            
            mdfPath = obj.mdfPath;
            usrPath = obj.usrPath;
            runSI = obj.runSI;
            
            if runSI
                obj.setClassDataVar('lastUsrFile',usrPath);
            end
            
            delete(obj);
        end
    end
end



%--------------------------------------------------------------------------%
% StartupConfig.m                                                          %
% Copyright � 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
