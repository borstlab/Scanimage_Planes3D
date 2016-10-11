classdef ConfigurationSaver < scanimage.interfaces.Component & most.HasClassDataFile
    %%% Functionality to store/loat SI setting to/from configuration and user files
    %% USER PROPS
    properties (SetObservable)
        fastCfgCfgFilenames = repmat({''},scanimage.components.ConfigurationSaver.FAST_CONFIG_NUM_CONFIGS,1);
        fastCfgAutoStartTf = false(scanimage.components.ConfigurationSaver.FAST_CONFIG_NUM_CONFIGS,1);
        fastCfgAutoStartType = cell(scanimage.components.ConfigurationSaver.FAST_CONFIG_NUM_CONFIGS,1);
    end
    
    properties (SetAccess=?most.Model,Dependent)
        usrCfgFileVarName; % varName stored in a USR file for cfg file associated with that USR file
    end
    
    properties (SetObservable,Transient)
        usrPropList = scanimage.components.ConfigurationSaver.USR_PROP_LIST_DEFAULT;
    end
    
    properties (SetObservable,Transient,SetAccess=protected)
        usrLoadingInProgress = false;
        cfgLoadingInProgress = false;
    end
    
    properties (SetAccess=protected,SetObservable)
        usrFilename = '';
        cfgFilename = '';
    end
    
    %% INTERNAL PROPS
    properties (Constant, Hidden)
        FAST_CONFIG_NUM_CONFIGS = 6;
        
        % List of props that may/must be included in USR file
        % usrAvailableUsrPropList = most.Model.mdlGetDefaultConfigProps('scanimage.SI');
        VERSION_PROP_NAMES =  {'VERSION_MAJOR'; 'VERSION_MINOR'}; %These props included in USR and CFG files, as well as file header data
        
        % List of props that are included in USR file by default
        USR_PROP_LIST_DEFAULT = {...
            'focusDuration';
            'hConfigurationSaver.fastCfgCfgFilenames';'hConfigurationSaver.fastCfgAutoStartTf';'hConfigurationSaver.fastCfgAutoStartType';
            'hBeams.stackUserOverrideLz';'hUserFunctions.userFunctionsUsr';'hDisplay.renderer';
            'hChannels.imagingSystemSettings';'hDisplay.channelsMergeEnable';'hDisplay.channelsMergeFocusOnly';
            'hMotors.motorSecondMotorZEnable';
            };
    end
    
    %%% Absract prop realization (most.Model)
    properties (Hidden, SetAccess=protected)
        mdlPropAttributes = zlclInitPropAttributes();
        mdlHeaderExcludeProps = {'usrPropList'};
    end
    
    %%% Abstract prop realization (scanimage.interfaces.Component)
    properties (Hidden,SetAccess = protected)
        numInstances = 1;
    end
    
    properties (Constant, Hidden)
        COMPONENT_NAME = 'ConfigFiles';      % [char array] short name describing functionality of component e.g. 'Beams' or 'FastZ'
        PROP_TRUE_LIVE_UPDATE = {};          % Cell array of strings specifying properties that can be set while the component is active
        PROP_FOCUS_TRUE_LIVE_UPDATE = {};    % Cell array of strings specifying properties that can be set while focusing
        DENY_PROP_LIVE_UPDATE = {...         % Cell array of strings specifying properties for which a live update is denied (during acqState = Focus)
		    'fastCfgCfgFilenames','fastCfgAutoStartTf','fastCfgAutoStartType'};
        
        FUNC_TRUE_LIVE_EXECUTION = {};       % Cell array of strings specifying functions that can be executed while the component is active
        FUNC_FOCUS_TRUE_LIVE_EXECUTION = {}; % Cell array of strings specifying functions that can be executed while focusing
        DENY_FUNC_LIVE_EXECUTION = {    'usrSaveUsr', ...
            'usrSaveUsrAs', ...
            'usrLoadUsr', ...
            'cfgSaveConfig', ...
            'cfgSaveConfigAs', ...
            'cfgLoadConfig', ...
            'fastCfgSetConfigFile', ...
            'fastCfgLoadConfig', ...
            'fastCfgClearConfigFile'};       % Cell array of strings specifying functions for which a live execution is denied (during acqState = Focus)
    end
    
    %% LIFECYCLE
    methods (Hidden)
        function obj = ConfigurationSaver(hSI)
            obj = obj@scanimage.interfaces.Component(hSI,[]);
            %Initialize class data file (ensure props exist in file)
            obj.zprvEnsureClassDataFileProps();
        end
    end
    
    %% PROP ACCESS
    methods
        function val = get.usrCfgFileVarName(obj)
            val = regexprep(sprintf('%s__configFileName',class(obj)),'\.','_');
        end
        
        function set.usrPropList(obj,val)
            obj.usrPropList = val;
        end
        
        function set.fastCfgCfgFilenames(obj,val)
            obj.assertSINotActive('fastCfgCfgFilenames');
            obj.validatePropArg('fastCfgCfgFilenames',val);
            obj.fastCfgCfgFilenames = val;
            obj.fastCfgAutoStartTf = obj.fastCfgAutoStartTf; %#ok<MCSUP>
        end
        
        function set.fastCfgAutoStartTf(obj,val)
            obj.assertSINotActive('fastCfgAutoStartTf');
            obj.validatePropArg('fastCfgAutoStartTf',val);
            
            tfEmpty = cellfun(@isempty,obj.fastCfgCfgFilenames); %#ok<MCSUP>
            val(tfEmpty) = false;
            
            obj.fastCfgAutoStartTf = val;
            tfEmptyType = cellfun(@isempty,obj.fastCfgAutoStartType); %#ok<MCSUP>
            tfAutoStartOnButEmptyType = val & tfEmptyType;
            obj.fastCfgAutoStartType(tfAutoStartOnButEmptyType) = {'grab'}; %#ok<MCSUP> % default to grab
        end
        
        function set.fastCfgAutoStartType(obj,val)
            obj.assertSINotActive('fastCfgAutoStartType');
            
            obj.validatePropArg('fastCfgAutoStartType',val);
            obj.fastCfgAutoStartType = val;
        end
    end
    
    %% USER METHODS
    methods
        function usrSaveUsr(obj)
            % Save 1) current values of USR property subset, 2) current GUI
            % layout, and 3) currently loaded CFG file (if any) to
            % currently specified usrFilename
            obj.usrSaveUsrAs(obj.usrFilename);
        end
        
        function usrSaveUsrAs(obj,fname,cfgfname,transientSave)
            % Save 1) current values of USR property subset, 2) current GUI
            % layout, and 3) currently loaded CFG file (if any) to
            % specified or selected USR filename
            % fname (optional): usr filename. If unspecified or empty, uiputfile is run.
            % cfgfname (optional): cfg filename to be associated with specified usr file. If empty or not specified, obj.cfgFilename is used.
            % transientSave (optional): If true, do not replace lastConfigFilePath in the class data file. Useful for internal calls
            if nargin < 2
                fname = [];
            end
            if nargin < 3 || isempty(cfgfname)
                cfgfname = obj.cfgFilename;
            end
            if nargin < 4
                transientSave = false;
            end
            
            obj.assertSINotActive('usrSaveUsrAs');
            
            % Handle cross caching with cfg file path
            lastPath = obj.getClassDataVar('lastUsrFile');
            if isempty(lastPath)
                lastPath = obj.getClassDataVar('lastConfigFilePath');
                
                if isempty(lastPath)
                    lastPath = most.idioms.startPath;
                end
            end
            
            usrFileName = obj.zprvUserCfgFileHelper(fname,...
                @()uiputfile('%.usr','Save Usr As...',lastPath),...
                @(path,file,fullfile)assert(exist(path,'dir')==7,'Specified directory does not exist.'));
            if isempty(usrFileName) % usr cancelled
                return;
            end
            
            if ~transientSave
                obj.setClassDataVar('lastUsrFile',usrFileName);
            end
            
            % make hchannels cache the imaging settings
            % HACK: a cleaner way would be to fire an event that says usr is about to save
            obj.hSI.hChannels.saveCurrentImagingSettings();
            
            % save usr subset
            obj.hSI.mdlSavePropSetFromList([obj.usrPropList; obj.VERSION_PROP_NAMES; {'hConfigurationSaver.usrPropList'}],usrFileName);
            
            % save layout
            if ~isempty(obj.hSI.hController)
                assert(isscalar(obj.hSI.hController));
                obj.hSI.hController{1}.ctlrSaveGUILayout(usrFileName);
            end
            
            % save associated cfgfile
            cfgfileVarname = obj.usrCfgFileVarName;
            tmp.(cfgfileVarname) = cfgfname; %#ok<STRNU>
            save(usrFileName,'-struct','tmp','-mat','-append');
            
            if ~transientSave
                obj.usrFilename = usrFileName;
            end
        end
        
        function tf = usrLoadUsr(obj,fname)
            % Load contents of specifed or selected USR file, updating 1)
            % values of USR property subset, 2) GUI layout, and 3)
            % currently loaded CFG file
            if nargin < 2
                fname = [];
            end
            
            tf = false;

            if strcmp(fname, '.usr')
                % If only the extension was given, we do not load a usr file
                % The reason for this is to allow easier scanimage scriptability
                return;
            end
            
            obj.assertSINotActive('usrLoadUsr');
            
            try
                % Handle cross caching with cfg file path
                lastPath = obj.getClassDataVar('lastUsrFile');
                if isempty(lastPath)
                    lastPath = obj.getClassDataVar('lastConfigFilePath');
                    
                    if isempty(lastPath)
                        lastPath = most.idioms.startPath;
                    end
                end
                
                usrFileName = obj.zprvUserCfgFileHelper(fname,...
                    @()uigetfile('%.usr','Load Usr File...',lastPath),...
                    @(path,file,fullfile)assert(exist(fullfile,'file')==2,'Specified file does not exist.'));
                if isempty(usrFileName) % usr cancelled
                    return;
                end
                obj.setClassDataVar('lastUsrFile',usrFileName);
                
                obj.usrLoadingInProgress = true;
                wb = waitbar(0,'Loading User Settings ...');
                
                % load usr propset
                usrPropSetFull = obj.hSI.mdlLoadPropSetToStruct(usrFileName);
                usrPropSetApply = rmfield(usrPropSetFull,intersect(fieldnames(usrPropSetFull),obj.VERSION_PROP_NAMES));
                
                % set usr* state
                obj.usrFilename = usrFileName;
                
                % load associated cfgfilename
                usrSpecifiedCfgFilename = [];
                s = load(usrFileName,'-mat');
                if isfield(s,obj.usrCfgFileVarName)
                    usrSpecifiedCfgFilename = s.(obj.usrCfgFileVarName);
                end
                
                waitbar(0.25,wb);
                
                % cfgFile handling
                % * If the usrFile specifies a cfgFile and it exists/loads
                % properly, that cfgfile will be used.
                % * If the usrFile specifies a cfgFile and it either doesn't
                % exist or doesn't load, no cfgFile will be used.
                % * If the usrFile doesn't specify a cfgFile (or specifies an
                % empty cfgFile), the current cfgFile will be used.
                if ~isempty(usrSpecifiedCfgFilename)
                    if exist(usrSpecifiedCfgFilename,'file')==2
                        cfgfilename = usrSpecifiedCfgFilename;
                    else
                        warning('ConfigurationSaver:fileNotFound',...
                            'Config file ''%s'' specified in usr file ''%s'' was not found.',usrSpecifiedCfgFilename,usrFileName);
                        cfgfilename = '';
                    end
                elseif ~isempty(obj.cfgFilename) && exist(obj.cfgFilename,'file')==2
                    cfgfilename = obj.cfgFilename;
                else
                    % no cfg file associated with USR file; no cfg file currently loaded
                    cfgfilename = '';
                end
                
                waitbar(0.5,wb);
                
                % apply usr/cfg state
                obj.hSI.mdlApplyPropSet(usrPropSetApply);
                if ~isempty(cfgfilename)
                    try
                        obj.cfgLoadConfig(cfgfilename);
                    catch %#ok<CTCH>
                        warning('ConfigurationSaver:errLoadingConfig',...
                            'Error loading config file ''%s''.',cfgfilename);
                        %this must be done to update imaging settings
                        %only do here if cfg load failed cfg will do this
                        obj.hSI.hChannels.registerChannels();
                    end
                else
                    %this must be done to update imaging settings. Only
                    %do here if not loading a cfg because cfg will do %this
                    obj.hSI.hChannels.registerChannels();
                end
                
                waitbar(0.75,wb);
                
                % update layout
                if ~isempty(obj.hSI.hController)
                    assert(isscalar(obj.hSI.hController));
                    obj.hSI.hController{1}.ctlrLoadGUILayout(usrFileName);
                end
                waitbar(1,wb);
                tf = true;
            catch ME
                most.idioms.reportError(ME);
            end
            delete(wb);
            obj.usrLoadingInProgress = false;
        end
        
        function cfgSaveConfig(obj)
            %Save values of (most) publicly settable properties of this class to currently loaded CFG file
            obj.cfgSaveConfigAs(obj.cfgFilename);
        end
        
        function cfgSaveConfigAs(obj,fname,transientSave)
            %Save values of (most) publicly settable properties of this class to specified or selected CFG file
            % transientSave (optional): If true, do not replace lastConfigFilePath in the class data file. Useful for internal calls
            
            % Save configuration to file and update .cfgFilename.
            % * If fname is not specified, uiputfile is called to get a file.
            % * If fname exists, config info is appended/overwritten to fname.
            % * If fname does not exist, it is created.
            if nargin < 2
                fname = [];
            end
            if nargin < 3
                transientSave = false;
            end
            
            obj.assertSINotActive('cfgSaveConfigAs');
            
            % Handle cross caching with usr file path
            lastPath = obj.getClassDataVar('lastConfigFilePath');
            if isempty(lastPath)
                [lastPath,~,~] = fileparts(obj.getClassDataVar('lastUsrFile'));
                
                if isempty(lastPath)
                    lastPath = most.idioms.startPath;
                end
            end
            
            cfgfilename = obj.zprvUserCfgFileHelper(fname,...
                @()uiputfile('*.cfg','Save Config As...',lastPath),...
                @(path,file,fullfile)assert(exist(path,'dir')==7,'Specified directory does not exist.'));
            if isempty(cfgfilename) % user cancelled
                return;
            end
            
            if ~transientSave
                obj.setClassDataVar('lastConfigFilePath',fileparts(cfgfilename));
            end
            
            % make hchannels cache the imaging settings
            % HACK: a cleaner way would be to fire an event that says usr is about to save
            obj.hSI.hChannels.saveCurrentImagingSettings();
            
            % save it
            obj.hSI.mdlSavePropSetFromList(setdiff([obj.hSI.mdlGetConfigurableProps;obj.VERSION_PROP_NAMES],obj.usrPropList), cfgfilename);
            
            if ~transientSave
                obj.cfgFilename = cfgfilename;
            end
        end
        
        function cfgLoadConfig(obj,fname)
            % Load contents of specifed or selected CFG file, updating
            % values of most publicly settable properties of this class.
            
            % * If fname is not specified, uigetfile is called to get a file.
            % * Config info is appended/overwritten to fname.
            
            if nargin < 2
                fname = [];
            end
            
            obj.assertSINotActive('cfgLoadConfig');
            
            try
                % Handle cross caching with usr file path
                % obj.ensureClassDataFile(struct('lastConfigFilePath',most.idioms.startPath));
                lastPath = obj.getClassDataVar('lastConfigFilePath');
                if isempty(lastPath)
                    [lastPath,~,~] = fileparts(obj.getClassDataVar('lastUsrFile'));
                    
                    if isempty(lastPath)
                        lastPath = most.idioms.startPath;
                    end
                end
                
                cfgfilename = obj.zprvUserCfgFileHelper(fname,...
                    @()uigetfile('*.cfg','Load Config...',lastPath),...
                    @(path,file,fullfile)assert(exist(fullfile,'file')==2,'Specified file does not exist.'));
                if isempty(cfgfilename)
                    return;
                end
                obj.setClassDataVar('lastConfigFilePath',fileparts(cfgfilename));
                
                obj.cfgLoadingInProgress = true;
                wb = waitbar(0,'Loading Configuration ...');
                % At the moment, this just loads the cfg, ignoring possible
                % need to reload the USR, or parts of the USR.
                cfgPropSet = obj.hSI.mdlLoadPropSetToStruct(cfgfilename);
                waitbar(0.25,wb);
                cfgPropSetApply = rmfield(cfgPropSet,intersect(fieldnames(cfgPropSet),obj.VERSION_PROP_NAMES));
                waitbar(0.5,wb);
                obj.hSI.mdlApplyPropSet(cfgPropSetApply);
                waitbar(0.75,wb);
                obj.cfgFilename = cfgfilename;
                waitbar(1,wb);
            catch ME
                most.idioms.reportError(ME);
            end
            obj.hSI.hChannels.registerChannels();
            obj.cfgLoadingInProgress = false;
            delete(wb);
        end
        
        function fastCfgSetConfigFile(obj,idx,fname)
            % Specify/select a CFG file to a numbered FastCFG,
            % for subsequent rapid (cached) loading with fastCfgLoadConfig()
            validateattributes(idx,{'numeric'},{'scalar' 'nonnegative' 'integer' '<=' obj.FAST_CONFIG_NUM_CONFIGS});
            
            % this statement is a workaround for a bug:
            % if fastCfgSetConfigFile is executed, the file dialog opens
            % when the file dialog opens, the current working directory is
            % changed to the start path of the file dialog
            % this is a problem if SI is not on the path, and callbacks
            % are being executed (as it is the case during an active
            % acquisition) the callbacks try to access classes that are not
            % found in the current working directory.
            obj.assertSINotActive('fastCfgSetConfigFile');
            
            if nargin < 3
                fname = [];
            end
            
            % Handle cross caching with cfg and usr file path
            lastPath = obj.getClassDataVar('lastFastConfigFilePath');
            if isempty(lastPath)
                lastPath = obj.getClassDataVar('lastConfigFilePath');
                
                if isempty(lastPath)
                    [lastPath,~,~] = fileparts(obj.getClassDataVar('lastUsrFile'));
                    
                    if isempty(lastPath)
                        lastPath = most.idioms.startPath;
                    end
                end
            end
            
            cfgfilename = obj.zprvUserCfgFileHelper(fname,...
                @()uigetfile('*.cfg','Select Config File',lastPath),...
                @(path,file,fullfile)assert(exist(fullfile,'file')==2,'Specified file does not exist.'));
            if isempty(cfgfilename) % user cancelled
                return;
            end
            obj.setClassDataVar('lastFastConfigFilePath',fileparts(cfgfilename));
            obj.fastCfgCfgFilenames{idx} = cfgfilename;
        end
        
        function fastCfgLoadConfig(obj,idx,tfBypassAutostart)
            %Load CFG file settings cached at a numbered FastCFG, autostarting acquisition if appropriate.
            
            % Load the idx'th fast config and autostart if
            % appropriate.
            % tfBypassAutostart: optional bool, defaults to false. If true, the
            % fastConfiguration is loaded but not autostarted, even if
            % autostart is on.
            if nargin < 3
                tfBypassAutostart = false;
            end
            
            obj.assertSINotActive('fastCfgLoadConfig');
            
            validateattributes(idx,{'numeric'},{'scalar' 'nonnegative' 'integer' '<=' obj.FAST_CONFIG_NUM_CONFIGS});
            validateattributes(tfBypassAutostart,{'logical'},{'scalar'});
            
            fname = obj.fastCfgCfgFilenames{idx};
            if isempty(fname)
                most.idioms.warn('ConfigurationSaver:fastCfgLoadConfig:noConfigFileLoaded',...
                    'No protocol file loaded for fast protocol #%d.',idx);
                return;
            end
            if exist(fname,'file')~=2
                most.idioms.warn('ConfigurationSaver:fastCfgLoadConfig:fileNotFound',...
                    'Config file ''%s'' not found.',fname);
                return;
            end
            
            if ~tfBypassAutostart && obj.fastCfgAutoStartTf(idx)
                obj.cfgLoadConfig(fname);
                autoStartType = obj.fastCfgAutoStartType{idx};
                switch autoStartType
                    case 'focus'
                        obj.hSI.startFocus();
                    case 'grab'
                        obj.hSI.startGrab();
                    case 'loop'
                        obj.hSI.startLoop();
                    otherwise
                        obj.cfgUnloadConfigOneShot();
                        assert(false,'AutoStart type must be set.');
                end
            else
                obj.cfgLoadConfig(fname);
            end
        end
        
        function fastCfgClearConfigFile(obj,idx)
            %Clear CFG file settings cached at a numbered FastCFG
            validateattributes(idx,{'numeric'},{'scalar' 'nonnegative' 'integer' '<=' obj.FAST_CONFIG_NUM_CONFIGS});
            obj.fastCfgCfgFilenames{idx} = '';
            obj.fastCfgAutoStartTf = obj.fastCfgAutoStartTf;
        end
    end
    
    %% INTERNAL METHODS
    methods (Hidden, Access=private)
        %TODO: Replace with Component mechanism
        function assertSINotActive(obj,name)
            assert(~obj.hSI.active,'Cannot access ''%s'' during an active acquisition',name);
        end
        
        function zprvEnsureClassDataFileProps(obj)
            obj.ensureClassDataFile(struct('lastUsrFile',''));
            obj.ensureClassDataFile(struct('lastConfigFilePath',''));
            obj.ensureClassDataFile(struct('lastFastConfigFilePath',''));
        end
        
        function fname = zprvUserCfgFileHelper(~,fname,fileFcn,verifyFcn) 
            % Get/preprocess/verify a config filename. Set 'lastConfigFilePath'
            % classdatavar, obj.cfgFilename.
            
            if isempty(fname)
                [f,p] = fileFcn();
                if isnumeric(f)
                    fname = [];
                    return;
                end
                fname = fullfile(p,f);
            else
                [p,f,e] = fileparts(fname);
                if isempty(p)
                    p = cd;
                end
                if isempty(e)
                    e = '.cfg';
                end
                f = [f e];
                fname = fullfile(p,f);
            end
            verifyFcn(p,f,fname);
        end
    end

    %%% Abstract method implementations (scanimage.interfaces.Component)
    methods (Hidden,Access = protected)
        function componentStart(~)
        end
        
        function componentAbort(~)
        end
    end
end

%% LOCAL 
function s = zlclInitPropAttributes()
s = struct();

%% usr/cfg/fastcfg
s.fastCfgCfgFilenames = struct('Classes','char','List',scanimage.components.ConfigurationSaver.FAST_CONFIG_NUM_CONFIGS,'AllowEmpty',1);
s.fastCfgAutoStartTf = struct('Classes','binaryflex','Attributes',{{'size',[scanimage.components.ConfigurationSaver.FAST_CONFIG_NUM_CONFIGS 1]}});
s.fastCfgAutoStartType = struct('Options',{{'focus';'grab';'loop'}},'List',scanimage.components.ConfigurationSaver.FAST_CONFIG_NUM_CONFIGS,'AllowEmpty',true);
end


%--------------------------------------------------------------------------%
% ConfigurationSaver.m                                                     %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
