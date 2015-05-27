classdef ThorDevice < most.MachineDataFile & most.APIWrapper & most.PDEPProp
    %THORDEVICE Summary of this class goes here
    %   Detailed explanation goes here
    %
    %% NOTES
    %   API Cached Data:
    %       enumNameMap: Map of all device enumeration strings (unique) to enumeration values (non-unique)   [used to compute other maps & for setting parameters string-encoded by this class]
    %       enumValMapMap: Map of parameter names to a Map of enumeration values to parameter-specific enumeration strings [used for getting parameters string-encoded by this class]
    %
    %   Constructor-initialized Maps:
    %       paramInfoMap: Map of parameter names to structure of information about the parameters [used for initialization/validation]
    %       paramCodeMap: Map of parameter names to parameter code values [used for GetParam/SetParam calls]
    %
    %   Subclass-initialized Map:
    %       prop2ParamMap: Map of class property names to API-defined parameters names
    %
    
    %% ABSTRACT PROPERTIES
    
    properties (Abstract, Constant, Hidden)
        deviceTypeDescriptorSDK; %Descriptor used by SDK for device type in function calls, e.g. 'Device', 'Camera', etc.
        prop2ParamMap; %Map of class property names to API-defined parameters names
    end
    
    %% ABSTRACT PROPERTY REALIZATIONS (most.MachineDataFile)
    properties (Constant, Hidden)
        mdfClassName = mfilename('class');
        mdfHeading = 'Thorlabs Devices';        
        
        mdfDependsOnClasses;
        mdfDirectProp=true;
        mdfPropPrefix;
    end 
    
    %% ABSTRACT PROPERTY REALIZATIONS (most.VAPIWrapper)
    
    %Following MUST be supplied with non-empty values for each concrete subclass
    properties (Constant, Hidden)
        apiSupportedVersionNames = {'1.4'}; %A list of shorthand names for API versions supported by this wrapper class
        
        apiCachedDataPath=fullfile(fileparts(fileparts(which('dabs.thorlabs.private.ThorDevice'))), 'private'); %Specifies path of apiData MAT file for this API wrapper class. If specified as empty(''), the class private directory will be used as default.
    end
    
    %Following properties are sometimes supplied values by concrete subclasses, or they can be left empty when realized - in which case default values are used.
    properties (SetAccess=protected, Hidden)
        
        %API 'pre-fab' cached data variables
        apiStandardFuncRegExp; %Regular expression used to parse function prototypes and identify 'standard' functions of the API, about which standard API data (e.g. methodNargoutMap, responseCodeMap) will be stored. If not supplied, data will be stored for /all/ functions found in library.
        apiHasFuncNargoutMap;  %<LOGICAL - Default=false> If true, 'funcNargoutMap' API data var is extracted from list of 'standard' functions, using extractFuncNargoutMap() method.
        
        %API response code handling
        apiResponseCodeSuccess = 1; %<NUMERIC> If specified, the first output argument of API 'standard' functions is taken to be a response code, with the specified response value(s) indicating call was successful.
        apiResponseCodeProcessor; %<One of {'none', 'apiResponseCodeMapHookFcn','apiResponseCodeHookFcn', <responseCodeMap regular expression>} - Default = 'none'>
        
        apiResponseCodeMapExtractionType; %<One of {'none', 'regexp', or 'method'} - Default = 'none'> If 'regexp' or 'function', the class has a 'responseCodeMap' API data var - a Map of response code names to response code values.
        
        %API 'custom' cached data variables
        apiCachedDataVarMap = containers.Map({'enumNameMap' 'enumValMapMap'}, {'initEnumMaps' 'initEnumMaps'}); %A Map whose keys (strings) specify custom class-specific data variables to store to API Data file, and whose values (strings) specify method names used to extract each of the 'apiCachedDataVars'. If same name is used for more than one variable, method is only invoked once.
        
        apiVersionDetectEnable; %<LOGICAL - Default=false> If true, indicates that subclass implements an 'apiVersionDetectHookFcn' method which performs auto-detection of API version installed on system, and returns apiCurrentVersion value. If false, the centrally maintained apiVersionData file is used for version specification of this API.
        
        apiHeaderRootPath = fullfile(fileparts(fileparts(which(mfilename('fullpath')))), 'private');
        apiHeaderFinalPaths;
        apiHeaderPathStem = 'ThorAPI';
        apiHeaderPlatformPaths = 'standard';
        
        apiDLLPaths='useApiHeaderPaths';
        %apiDLLPaths = mapInitAPIDLLPaths(); %version-indexed. By default, no path will be used, implying system default location.
        apiDLLPlatformPaths='standard';
        
        apiAuxFile1Names;
        apiAuxFile1Paths;
        
        apiAuxFile2Names;
        apiAuxFile2Paths;
    end
    
    
    %% ABSTRACT PROPERTY REALIZATIONS (most.PDEPProp)
    
    properties (Constant, Hidden)
        pdepSetErrorStrategy = 'restoreCached'; % <One of {'setEmpty','restoreCached','setErrorHookFcn'}>. setEmpty: stored property value becomes empty when driver set error occurs. restoreCached: restore value from prior to the set action generating error. setErrorHookFcn: The subclass implements its own setErrorHookFcn() to handle set errors in subclass-specific manner.
    end
    
    
    %% PUBLIC PROPERTIES
    properties (SetAccess=protected)
        deviceID;
    end
    
    %% PROTECTED/PRIVATE PROPERTIES
    properties (Hidden,SetAccess=protected)
        paramInfoMap; %Map of parameter names to structure of information about the parameters
        paramCodeMap; %Map of parameter names to parameter code values
    end
    
    properties (Hidden, SetAccess=private)
        currentDeviceIDMap = containers.Map('KeyType','char','ValueType','uint8'); %Map of ThorDevice classes to value indicating last deviceID for which property access or method was applied
        
        isConnected = false;
    end
    
    
    %% CONSTRUCTOR/DESTRUCTOR
    methods
        
        function obj = ThorDevice(deviceID)
            
            %Parse input arguments
            if ~nargin || isempty(deviceID)
                deviceID = 0;
            end
            
            %Add entry to currentDeviceIDMap for this class, if needed
            if ~obj.currentDeviceIDMap.isKey(class(obj))
                obj.currentDeviceIDMap(class(obj)) = -1;
            end
            
            %Check-out device index
            if obj.accessDeviceCheckoutList('checkout',deviceID)
                obj.deviceID = deviceID;
                obj.isConnected = true;
            else
                error('%s %d has already been constructed. Cannot construct new instance.',obj.deviceTypeDescriptorSDK, deviceID);
            end
            
            %             %Must invoke FindXXX in folder with XML file -- we require this to be in the 'machine data' folder
            %             origPath = pwd();
            %             cd(fileparts(obj.getClassDataVar('lastMachineDataFilePath')));
            %             resp = obj.apiCallRaw(sprintf('Find%ss',obj.deviceTypeDescriptorSDK), 0); %Use apiCallRaw, since apiCall() has been overridden to call SelectXXX() before each call
            %             cd(origPath);
            %             assert(ismember(resp,obj.apiResponseCodeSuccess),'Failed to find %s device',obj.apiPrettyName);
            %
            %             obj.isConnected = true;
            
            %Must select the device once
            startDir = pwd();
            cd(fileparts(obj.getClassDataVar('lastMachineDataFilePath')));
            funcName = ['Select' obj.deviceTypeDescriptorSDK];
            resp = obj.apiCallRaw(funcName,obj.deviceID);
            cd(startDir);
            if ~ismember(resp,obj.apiResponseCodeSuccess)
                obj.apiProcessErrorResponseCode(resp,funcName);
            end
            
            %Query API to build up Map of parameter information 
            znstInitParamMaps();
            
            return;
            
            function znstInitParamMaps()
                %Function initializes paramInfoMap/paramCodeMap properties
                %   paramInfoMap: map from class property name to information structure for corresponding param
                %   paramCodeMap: map from class property name to code number for corresponding param (for Get/SetParam() calls)
                %
                
                enumNameMap = obj.accessAPIDataVar('enumNameMap');
                obj.paramInfoMap = containers.Map();
                obj.paramCodeMap = containers.Map({'dummy'},{0}); obj.paramCodeMap.remove('dummy');
                
                propNames = obj.prop2ParamMap.keys();
                
                for j=1:length(propNames)
                    
                    paramName = obj.prop2ParamMap(propNames{j});
                    
                    if enumNameMap.isKey(paramName) %For backward compatibility to earlier API versions with less properties
                        paramCode = enumNameMap(paramName);
                        obj.paramCodeMap(propNames{j}) = paramCode;
                        
                        paramStruct = struct();
                        
                        paramInfoNames = {'paramType' 'paramAvailable' 'paramReadOnly' 'paramMin' 'paramMax' 'paramDefault'};
                        
                        paramInfoVals = cell(length(paramInfoNames),1);
                        
                        try
                            [paramInfoVals{:}] = obj.apiCall('GetParamInfo',paramCode,0,0,0,0,0,0);
                        catch %Handle backward-compatibility -- some of the current properties may not have been in previous API versions
                            continue;
                        end
                        
                        for k=1:length(paramInfoNames)
                            paramStruct.(paramInfoNames{k}) = paramInfoVals{k};
                        end
                        
                        obj.paramInfoMap(propNames{j}) = paramStruct;
                    end
                    
                end
            end
        end
        
        function delete(obj)
            
            if obj.isConnected
                %API device deletion
                obj.apiCall(sprintf('Teardown%s',obj.deviceTypeDescriptorSDK));
                
                %Update class instance counter
                try
                    obj.accessDeviceCheckoutList('checkin',obj.deviceID);
                catch ME
                    fprintf(2,'Error on delete: \n  %s',ME.getReport('extended','hyperlinks','off'));
                end
            end
        end
        
        function initialize(obj)
            
            %Initialize default parameter values
            paramNames = obj.paramInfoMap.keys();
            obj.pdepPropGlobalLock = true;
            for i=1:length(paramNames)
                if ~isempty(obj.findprop(paramNames{i}))
                    obj.(paramNames{i}) = obj.paramInfoMap(paramNames{i}).paramDefault;
                end
            end
            obj.pdepPropGlobalLock = false;           
        end
        
    end
    

    
    
    
    %% PROPERTY ACCESS
    
    %PDEP Property Handling
    methods (Hidden, Access=protected)
        
        
        function val = getParameterEncoded(obj,propName)
            rawVal = obj.getParameterSimple(propName);
            if isempty(rawVal) %For backward compatibility -- test for case that property doesn't exist               
                val = [];
                return;
            end
            
            %Convert raw (numeric) value to corresponding string
            enumValMapMap = obj.accessAPIDataVar('enumValMapMap');
            testKey = [upper(propName(1)) propName(2:end)];
            if enumValMapMap.isKey(testKey)
                enumValMap = enumValMapMap(testKey);
            else %Look for a close match
                allKeys = enumValMapMap.keys();
                matchedKeyIdx = find(~cellfun(@isempty,strfind(allKeys,testKey)));
                
                if isscalar(matchedKeyIdx)
                    enumValMap = enumValMapMap(allKeys{matchedKeyIdx});
                else            
                    error('Failed to find API data identifying the specified property ''%s''',propName);
                end
            end
            
            val = enumValMap(rawVal);  %Converts to string corresponding to value
            
        end
        
        function setParameterEncoded(obj,propName, val)
            
            %Convert string value to numeric value used by API
            if ischar(val)
                enumNameMap = obj.accessAPIDataVar('enumNameMap');
                assert(enumNameMap.isKey(val),'Supplied value (''%s'') is not a valid value for property ''%s''',val,propName);
                
                val = enumNameMap(val);
            end
            
            obj.setParameterSimple(propName,val);
        end
        
        function val = getParameterSimple(obj,propName)
            if obj.paramInfoMap.isKey(propName) %For backward compatibility -- test for case that property doesn't exist               
                val = obj.apiCall('GetParam',obj.paramCodeMap(propName),0);
            else
                val = [];
            end
        end
        
        function setParameterSimple(obj,propName,val)
            obj.apiCall('SetParam',obj.paramCodeMap(propName),val);
        end
        
        function val = getParameterMaxInf(obj,propName)
            if obj.paramInfoMap.isKey(propName) %For backward compatibility -- test for case that property doesn't exist               
                val = obj.apiCall('GetParam',obj.paramCodeMap(propName),0);
                
                if val == obj.paramInfoMap(propName).paramMax
                    val = inf;
                end
            else
                val = [];
            end
        end
        
        function setParameterMaxInf(obj,propName,val)
           
            maxVal = obj.paramInfoMap(propName).paramMax;
            if val >= maxVal
                obj.apiCall('SetParam',obj.paramCodeMap(propName),maxVal);
            else
                obj.apiCall('SetParam',obj.paramCodeMap(propName),val);
            end
        end
        
    end
    
    
    %% PRIVATE/PROTECTED METHODS
    
    
    
    %API Data var initializers
    methods (Hidden)
        
        function s = initEnumMaps(obj)
            
            s = struct();
            s.enumNameMap = containers.Map({'dummy'},{0});  %Map of all device enumeration strings (unique) to enumeration values (non-unique)
            s.enumValMapMap = containers.Map(); %Map of parameter names to a Map of enumeration values to parameter-specific enumeration strings
            
            s.enumNameMap.remove('dummy');
            
            
            %Extract enum information from prototype file
            [p,f] = fileparts(obj.apiHeaderFileName);
            currPath = cd();
            cd(p);
            [~,~,enumInfo] = eval(f);
            cd(currPath);
            
            enumTypeNames = fieldnames(enumInfo);
            for i=1:length(enumTypeNames)
                enumTypeName = enumTypeNames{i};
                
                s.enumValMapMap(enumTypeName) = containers.Map({0},{'dummy'});
                s.enumValMapMap(enumTypeName).remove(0);
                
                enumStruct = enumInfo.(enumTypeNames{i});
                enumNames = fieldnames(enumStruct);
                
                enumValMap = s.enumValMapMap(enumTypeName); %Handle to enum value-keyed Map for current enum type
                for j = 1:length(enumNames)
                    enumVal = enumStruct.(enumNames{j});
                    
                    s.enumNameMap(enumNames{j}) = enumVal; %#ok<AGROW>
                    enumValMap(enumVal) = enumNames{j};
                end
            end
            
        end                        
    end
    
    methods (Access=protected)
        function tf = accessDeviceCheckoutList(obj,mode,deviceNum)
            %Check against a master 'checkout' list to ensure that only one object handle is created per physical device instance

            persistent deviceCheckoutListMap
            
            if isempty(deviceCheckoutListMap)
                deviceCheckoutListMap = containers.Map('KeyType','char','ValueType','logical'); %Map of ThorDevice classes to logical array indicating which of the available devices of that class have been checked out (since Matlab started)
            end
            
            %             if ~deviceCheckoutListMap.isKey(class(obj))
            %
            %                 %Must invoke FindXXX in folder with XML file -- we require this to be in teh 'machine data' folder
            %                 startDir = pwd();
            %                 cd(fileparts(obj.getClassDataVar('lastMachineDataFilePath')));
            %                 [responseCode, numDevices] = obj.apiCallRaw(sprintf('Find%ss',obj.deviceTypeDescriptorSDK),0);
            %                 cd(startDir);
            %
            %                 if ~ismember(responseCode,obj.apiResponseCodeSuccess)
            %                     obj.apiProcessErrorResponseCode(responseCode); %Throws an error if response indicates failure
            %                 end
            %
            %                 deviceCheckoutListMap(class(obj)) = false(numDevices,1);
            %             end
            
            
            %Initialize checkoutListMap, if needed
            if ~deviceCheckoutListMap.isKey(class(obj))
                numDevices = znstFindDevices();
                deviceCheckoutListMap(class(obj)) = false(numDevices,1);
            end
            
            deviceCheckoutList = deviceCheckoutListMap(class(obj));
            
            switch mode
                case 'checkstatus'
                    tf = deviceCheckoutList(deviceNum+1);
                case 'checkout'                    

                    tf = ~deviceCheckoutList(deviceNum+1);
                    
                    if tf
                        znstFindDevices(); %ThorAPI presently requires that we FindDevices before we select a device that has been previously torn down
                    end
                    
                    deviceCheckoutList(deviceNum+1) = true;
                    deviceCheckoutListMap(class(obj)) = deviceCheckoutList;
                case 'checkin'
                    tf = true;
                    deviceCheckoutList(deviceNum+1) = false;
                    deviceCheckoutListMap(class(obj)) = deviceCheckoutList;
            end
            
            function numDevices = znstFindDevices()
                %Must invoke FindXXX in folder with XML file -- we require this to be in teh 'machine data' folder
                startDir = pwd();
                cd(fileparts(obj.getClassDataVar('lastMachineDataFilePath')));
                [responseCode, numDevices] = obj.apiCallRaw(sprintf('Find%ss',obj.deviceTypeDescriptorSDK),0);
                cd(startDir);
                
                if ~ismember(responseCode,obj.apiResponseCodeSuccess)
                    obj.apiProcessErrorResponseCode(responseCode); %Throws an error if response indicates failure
                end
                
            end
            
        end
        
    end
    
    %% METHOD OVERRIDES (most.APIWrapper)
    methods (Hidden)
        
        function varargout = apiCall(obj,funcName,varargin)
            
            %Must select the current instance before executing action
            if obj.deviceID ~= obj.currentDeviceIDMap(class(obj)) %Don't hit the API call, if not needed
                startDir = pwd();
                cd(fileparts(obj.getClassDataVar('lastMachineDataFilePath')));
                obj.apiCall@most.APIWrapper(['Select' obj.deviceTypeDescriptorSDK],obj.deviceID);
                cd(startDir);
            end
            
            varargout = cell(nargout,1);
            [varargout{:}] = obj.apiCall@most.APIWrapper(funcName,varargin{:});
        end                  
        
    end
    
    methods (Access=protected)        
        function smartLoadLibrary(obj,varargin)
            
            %Suppress 'enumeration exists' warning. Different ThorDevice classes may have headers defining the same enumeration, possibly with contradictory definitions, e.g. the Params enum
            %However, this class does not make any known use of enumeration arguments, so this warning is, for practical purposes, immaterial
            
            s = warning('query','all');
            warning('off','MATLAB:loadlibrary:EnumExists');            
            smartLoadLibrary@most.APIWrapper(obj,varargin{:});
            warning(s);
        end
    end    
    
end

%% HELPER FUNCTIONS



%
% function apiDLLPaths = zlclInitAPIDLLPaths()
%
% %TODO: This function should be obviated if APIWrapper makes option of placing DLL in same folder as header file more easily available
%
% apiDLLPaths = containers.Map();
%
% apiDLLPaths('0.1.1') = fullfile(fileparts(fileparts(mfilename('fullpath'))),'private','ThorAPI_0_1_1');
% apiDLLPaths('0.1.5') = fullfile(fileparts(fileparts(mfilename('fullpath'))),'private','ThorAPI_0_1_5');
%
%
% end

