classdef PMT < dabs.thorlabs.private.ThorDevice
    %PMT Summary of this class goes here
    %   Detailed explanation goes here
    
        %% ABSTRACT PROPERTY REALIZATIONS (dabs.thorlabs.private.ThorDevice)
    properties (Constant, Hidden)
        deviceTypeDescriptorSDK = 'Device'; %Descriptor used by SDK for device type in function calls, e.g. 'Device', 'Camera', etc.
        prop2ParamMap=zlclInitProp2ParamMap(); %Map of class property names to API-defined parameters names
    end
    
    
    %% ABSTRACT PROPERTY REALIZATIONS (most.APIWrapper)
    
    %Following MUST be supplied with non-empty values for each concrete subclass
    properties (Constant, Hidden)
        apiPrettyName='Thorlabs PMT Module';  %A unique descriptive string of the API being wrapped
        apiCompactName='ThorlabsPMT'; %A unique, compact string of the API being wrapped (must not contain spaces)        
       
        %Properties which can be indexed by version
        apiDLLNames = 'ThorPMT'; %Either a single name of the DLL filename (sans the '.dll' extension), or a Map of such names keyed by values in 'apiSupportedVersionNames'
        %apiHeaderFilenames = {'PMT_SDK_MOD.h' 'PMT_SDK_MOD.h'  'PMT_SDK_MOD.h' 'PMT_SDK.h' 'PMT_SDK.h' 'PMT_SDK.h' 'PMT_SDK.h' 'PMT_SDK.h'}; %Either a single name of the header filename (with the '.h' extension - OR a .m or .p extension), or a Map of such names keyed by values in 'apiSupportedVersionNames'
        apiHeaderFilenames = 'ThorPMT_proto.m';
    end
    
    
    %% DEVICE PROPERTIES (PSEUDO-DEPENDENT)
    properties (SetObservable, GetObservable, AbortSet)
        scanEnable;
        pmtGain1;
        pmtGain2;
        pmtEnable1;
        pmtEnable2;
        deviceType;        
    end
    
    %% PRIVATE/PROTECTED PROPERTIES
    
    properties (SetAccess=protected,Hidden)
       paramChangeFlag; %Logical indicating if a property has been changed
       %isConnected=false;  % true if PMT is actually connected
    end
    
    
    %% CONSTRUCTOR/DESTRUCTOR
    methods
        
        function obj = PMT(varargin)
            
            %Invoke superclass constructor
            obj = obj@dabs.thorlabs.private.ThorDevice(varargin{:});                       
            
            %Invoke superclass initializer
            obj.initialize();
            
        end
        
        %         function delete(obj)
        %             %             if  obj.isConnected
        %           `     %                 %obj.scanEnable = 0;
        %             %
        %             %                 % obj.accessDeviceCheckoutList('checkin',obj.deviceID);
        %             %                 % resp = obj.apiCallRaw('TeardownDevice');
        %             %                 %obj.apiCall('TeardownDevice');
        %             %                 %disp(['PMT delete TeardownDevice returned ' num2str(resp)]);
        %             % %                unloadlibrary('ThorPMT');
        %             %             end
        %         end
    end
    
    %% PROPERTY ACCESS
    
    %PDep Property Handling

    methods (Hidden, Access=protected)
        function pdepPropHandleGet(obj,src,evnt)
            propName = src.Name;
            
            if(~obj.isConnected)
                return
            end
            
            switch propName
                case {}
                    obj.pdepPropGroupedGet(@obj.getParameterEncoded,src,evnt);
                otherwise
                    obj.pdepPropGroupedGet(@obj.getParameterSimple,src,evnt);
            end
        end
        
        function pdepPropHandleSet(obj,src,evnt)
             propName = src.Name;
        
             if(~obj.isConnected)
                return 
             end
             
%              paramName = obj.paramEnumMap(propName);
%              enumNameMap = obj.accessAPIDataVar('enumNameMap');
%              paramCode = enumNameMap(paramName);
                paramCode = obj.paramCodeMap(propName);
             
             
             
%              disp(['PMT prop set paramName=' paramName ' paramCode=' num2str(paramCode)]);
%              switch(propName)
%                  case 'scanEnable' 
%                      paramName = 'PARAM_SCANNER_ENABLE';
%                      paramCode = 708;
%                  case 'pmtEnable1'
%                      paramName = 'PARAM_PMT1_ENABLE';
%                      paramCode = 701;
%                  case 'pmtEnable2'
%                      paramName = 'PARAM_PMT2_ENABLE';
%                      paramCode = 703;
%                  otherwise
%                      return
%              end
            % paramName = obj.enumMap(src.Name)
            
             
            

             %For now, since all we use PMT for is property access -- push
             %value to device!
            
%             err = obj.apiCallRaw('SetParam', paramCode, obj.(propName));
%             err1 = obj.apiCallRaw('SetupPosition');             
%             err2 = obj.apiCallRaw('StartPosition');

             
                
             %err = calllib('ThorPMT', 'SelectDevice', obj.deviceID);
             %obj.apiCall('SelectDevice', obj.deviceID);
             
 %            disp(['PMT.pdepPropHandleSet: SelectDevice err=' num2str(err)]);
             propval = obj.(propName);
             %err = calllib('ThorPMT', 'SetParam', paramCode, propval);
             obj.apiCall('SetParam', paramCode, propval);
             
             %Following sequence required for property change to take effect
             obj.apiCall('PreflightPosition');
             obj.apiCall('SetupPosition');
             obj.apiCall('StartPosition');
             obj.apiCall('PostflightPosition');

             %
             % %             disp(['PMT.pdepPropHandleSet: parameter ' paramName ' propval=' num2str(propval) ' SetParam err=' num2str(err)]);
             %              if(strcmp(propName, 'scanEnable'))
             %                  if(propval == 1) % start scan
             %                      %err = calllib('ThorPMT','PreflightPosition');
             %                      obj.apiCall('PreflightPosition');
             %     %                 disp(['PMT.pdepPropHandleSet: PreflightPosition err=' num2str(err)]);
             %                      %err = calllib('ThorPMT','SetupPosition');
             %                      obj.apiCall('SetupPosition');
             %    %                  disp(['PMT.pdepPropHandleSet: SetupPosition err=' num2str(err)]);
             %                      %err = calllib('ThorPMT','StartPosition');
             %                      obj.apiCall('StartPosition');
             %   %                   disp(['PMT: StartPosition err=' num2str(err)]);
             %                  else % stop scan
             %                      %err = calllib('ThorPMT','PostflightPosition');
             %                      obj.apiCall('PostflightPosition');
             %  %                    disp(['PMT: PostflightPosition err=' num2str(err)]);
             %                  end
             %              else
             %                 %err = calllib('ThorPMT','SetupPosition');
             %                 err = obj.apiCallRaw('PostflightPosition');
             % %                disp(['PMT.pdepPropHandleSet: parameter ' paramName ' SetupPosition err=' num2str(err)]);
             %              end
             %                % ' StartPosition err=' num2str(err2)]);

%             
%             switch propName
%                 case {}
%                     obj.pdepPropGroupedSet(@obj.setParameterEncoded,src,evnt);
%                 otherwise
%                     obj.pdepPropGroupedSet(@obj.setParameterSimple,src,evnt);
%             end
%             
%            obj.paramChangeFlag = true;
        end
        
    end

    %% DEVELOPER METHODS
    methods 
        function display(obj)
            obj.displaySmart({'scanEnable' 'pmtEnable1' 'pmtEnable2' 'pmtGain1' 'pmtGain2'});            
        end
        
    end
    
end

%% HELPERS


function prop2ParamMap = zlclInitProp2ParamMap()

prop2ParamMap = containers.Map('KeyType','char','ValueType','char');

prop2ParamMap('scanEnable') = 'PARAM_SCANNER_ENABLE';
prop2ParamMap('pmtGain1') = 'PARAM_PMT1_GAIN_POS';
prop2ParamMap('pmtGain2') = 'PARAM_PMT2_GAIN_POS';
prop2ParamMap('pmtEnable1') = 'PARAM_PMT1_ENABLE';
prop2ParamMap('pmtEnable2') = 'PARAM_PMT2_ENABLE';
prop2ParamMap('deviceType') = 'PARAM_DEVICE_TYPE';
%TODO: Fill in others!


end


% function apiHeaderFilenamesMap = mapInitApiHeaderFilenames()
% 
% apiHeaderFilenamesMap = containers.Map();
% 
% apiHeaderFilenamesMap('0.1.1') = 'PMT_SDK_0_1_1_MOD.h';
% 
% end
