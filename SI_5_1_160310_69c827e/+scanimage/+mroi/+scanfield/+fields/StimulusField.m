classdef StimulusField < scanimage.mroi.scanfield.ScanField
    %% Abstract property realization scanimage.mroi.scanfield.ScanField    
    properties (SetAccess = protected, Dependent)
        shortDescription;
    end

    properties(SetObservable)
        stimfcnhdl;          % function handle to parametric function that defines stimulus path
        stimparams;          % cell array of additional parameter name/value tuples to be passed into stimfcnhdl {'revolutions',5,}

        beamsfcnhdl;         % function handle to parametric function that defines stimulus beam powers (for power users)
        
        duration;            % time the stimulus will take in s
        repetitions;         % number of repetitions of the stimulus
        rotation;            % rotation in degrees
        powers;              % array of power of each beam in percent
    end
    
    properties(SetObservable,Dependent)
        centerXY;            % [x,y] center of the stimulus in FOV coordinates
        scalingXY;           % [x,y] scaling of the stimulus in FOV coordinates
    end
    
    %% Lifecycle
    methods
        function obj = StimulusField(stimfcnhdl,stimparams,duration,repetitions,...
                centerXY,scalingXY,rotation,powers)
            obj = obj@scanimage.mroi.scanfield.ScanField();
                        
            if isempty(stimparams)
                stimparams = {};
            end
            
            obj.stimfcnhdl = stimfcnhdl;
            obj.stimparams = stimparams;
            
            obj.duration = duration;
            obj.repetitions = repetitions;
            obj.centerXY = centerXY;
            obj.scalingXY = scalingXY;
            obj.rotation = rotation;
            obj.powers = powers;
            
            obj.beamsfcnhdl = @scanimage.mroi.stimulusbeamfunctions.beamPowers;
        end
    end
    
    %% Abstract methods realization of scanimage.mroi.scanfield.ScanField
    methods
        function s = getPropertyStructForSaving(obj)
            s=struct(...
                'stimfcnhdl',  func2str(obj.stimfcnhdl),...
                'stimparams',  {obj.stimparams},...
                'duration',    obj.duration,...
                'repetitions', obj.repetitions,...
                'centerXY',    obj.centerXY,...
                'scalingXY',   obj.scalingXY,...
                'rotation',    obj.rotation,...
                'powers',      obj.powers,...
                'beamsfcnhdl', func2str(obj.beamsfcnhdl) );
        end
        
        function out = interpolate(obj,other,frac)
            assert(strcmp(func2str(obj.stimfcnhdl),func2str(other.stimfcnhdl)),...
                  ['Cannot interpolate scanfields %s and %s.',...
                   'The scanfields use different stimulus path generation functions: %s   vs.   %s'],...
                   func2str(obj.stimfcnhdl),func2str(other.stimfcnhdl));
                
            assert(strcmp(func2str(obj.beamsfcnhdl),func2str(other.beamsfcnhdl)),...
                  ['Cannot interpolate scanfields %s and %s.',...
                   'The scanfields use different stimulus beam power generation functions: %s   vs.   %s'],...
                   func2str(obj.beamsfcnhdl),func2str(other.beamsfcnhdl));
                    
            duration_    = interpolateProp('duration');
            repetitions_ = interpolateProp('repetitions');
            centerXY_    = interpolateProp('centerXY');
            scalingXY_   = interpolateProp('scalingXY');
            rotation_    = interpolateProp('rotation');
            powers_      = interpolateProp('powers');
            
            out = scanimage.mroi.scanfield.fields.StimulusField(obj.stimfcnhdl,obj.stimparams,...
                    duration_,repetitions_,centerXY_,scalingXY_,rotation_,powers_);
            
            % local function
            function val = interpolateProp(propname)
                val = (other.(propname) - obj.(propname)) .* frac + obj.(propname);
            end
        end
            
        function rect = boundingbox(obj)
                disp('Todo: scanimage.mroi.scanfield.fields.StimulusField.boundingbox: implement this properly');
                rect = [];
        end
        
        function [xs,ys]=transform(obj,xs,ys,affineGlobal)
           if nargin < 4 || isempty(affineGlobal)
               affineGlobal = eye(3,3);
           end
           
           assert(isequal(size(affineGlobal),[3,3]));
           
           %% Transforms points from unit-scan space to fov space
           r=[xs(:)';ys(:)';ones(size(xs(:)'))];
           r=(affineGlobal*obj.affine())*r;
           xs=reshape(r(1,:),size(xs));
           ys=reshape(r(2,:),size(ys));
       end
    end
    
    methods
        function T=affine(obj)
            % Returns the affine transform
            centerX = obj.centerXY(1);
            centerY = obj.centerXY(2);
            
            scalingX = obj.scalingXY(1);
            scalingY = obj.scalingXY(2);
            
            radians = obj.rotation*pi/180;
            
            % scaling
            S=diag([scalingX scalingY 1]);
            
            % rotation
            R=eye(3);
            c = cos(radians);
            s = sin(radians);
            R(1:2,1:2) = [c -s; s c];
            
            % translation
            C=eye(3);
            C(:,3)=[centerX centerY 1];
            
            T = C*R*S;
        end
    end
    
    %% Static methods
    methods(Static)
        function obj=loadobj(s)
            obj=scanimage.mroi.scanfield.fields.StimulusField(str2func(s.stimfcnhdl),s.stimparams,...
                s.duration,s.repetitions,s.centerXY,s.scalingXY,s.rotation,s.powers);
            obj.beamsfcnhdl = str2func(s.beamsfcnhdl);
        end
    end
    
    %% Property setters / getters
    methods
        function val = get.shortDescription(obj)
            if isempty(obj.stimfcnhdl)
                stimfcnname = [];
            else
                stimfcnname = regexp(func2str(obj.stimfcnhdl),'[^.]*$','match','once');
            end
            val = sprintf('Stim: %s',stimfcnname);
        end
        
        function set.stimfcnhdl(obj,val)
            if isa(val,'char')
                val = str2func(val);
            end
            
            obj.stimfcnhdl = val;
            obj.fireChangedEvent();
        end
        
        function set.stimparams(obj,val)
            assert(iscell(val)&&size(val,1)<=1&&~mod(length(val),2),...
                'Expect stimparams to be a cell row vector with an even number of elements');
            obj.stimparams = val;
            obj.fireChangedEvent();
        end
        
        function set.beamsfcnhdl(obj,val)
            obj.beamsfcnhdl = val;
            obj.fireChangedEvent();
        end
        
        function set.duration(obj,val)
            obj.duration = val;
            obj.fireChangedEvent();
        end
        
        function set.repetitions(obj,val)
            obj.repetitions = val;
            obj.fireChangedEvent();
        end        

        function set.centerXY(obj,val)            
            if ~isempty(obj.rect)
                obj.rect(1:2)=val-obj.rect(3:4)/2; % calls fireChangedEvent
            else
                obj.rect=[val 0 0];
            end                                    
        end
        
        function val=get.centerXY(obj)
            r=obj.rect;
            val=r(1:2)+r(3:4)/2;
        end
        
        function set.scalingXY(obj,val)
            % stim normally ranges from -scalingXY to scalingXY 
            % so size is 2*scalingXY
            if ~isempty(obj.rect)                
                newrect=[obj.rect(1:2)+0.5*obj.rect(3:4)-val 2*val];
                obj.rect=newrect; % calls fireChangedEvent
            else
                obj.rect=[0 0 2*val];
            end            
        end
        
        function val=get.scalingXY(obj)
            r=obj.rect;            
            val=r(3:4)/2;
        end
        
        function set.rotation(obj,val)
            val = mod(val,360);
            obj.rotation = val;
            obj.fireChangedEvent();
        end
        
        function set.powers(obj,val)
            obj.powers = val;
            obj.fireChangedEvent();
        end
    end
    
    methods (Access = protected)
        function fireChangedEvent(obj,varargin)
            obj.fireChangedEvent@scanimage.mroi.scanfield.ScanField(varargin{:});
        end
    end
end


%--------------------------------------------------------------------------%
% StimulusField.m                                                          %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
