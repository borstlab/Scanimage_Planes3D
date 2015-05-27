classdef TestScanimage < most.testing.TestSuite
    %TESTSCANIMAGE Summary of this class goes here
    %   Detailed explanation goes here
    
    %% ABSTRACT PROPERTY REALIZATIONS (most.testing.TestSuite)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    properties (Constant)
        classUnderTest = 'scanimage';
        constructionPVArgs = {};
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    %% CLASS-SPECIFIC PROPERTIES
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    properties (Hidden, Access=public)
        robot = most.testing.Robot();
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    %% CONSTRUCTOR/DESTRUCTOR
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    methods
        function obj = TestScanimage()
            obj = obj@most.testing.TestSuite('isClass','false');
            obj.setup();
        end
        
        function delete(obj)
            scim_exit; 
        end
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    %% ABSTRACT METHODS REALIZATIONS
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    methods(Access=protected)
   
        function setup(obj)
            import most.testing.*;
            
            obj.addTest(Test(@obj.testGrab,'testName','Testing A Single Grab'));
            %obj.addTest(Test(@obj.testAbortedGrab,'testName','Testing Aborted Grabs'));
            
        end
        
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    %% CLASS METHODS
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    methods (Access=public)
        
        function [didPass,output] = testGrab(obj)
            global gh
            
            try
                obj.robot.moveToUiComponent(gh.mainControls.grabOneButton,true);
                obj.robot.leftClick();
            catch ME
                didPass = false;
                output = ME.message;
                return;
            end
        
            didPass = true;
            output = 'Success.';
        end
        
        function [didPass,output] = testAbortedGrab(obj)
            global gh
            
            N = 4;
            CLICK_DELAY = 0.1;
            REPEAT_DELAY = 1.0;
            
            for i=1:N
                try
                    obj.robot.moveToUiComponent(gh.mainControls.grabOneButton,true);
                    obj.robot.leftClick();
                    pause(CLICK_DELAY);
                    obj.robot.leftClick();
                catch ME
                    didPass = false;
                    output = ME.message;
                    return;
                end
                
                pause(REPEAT_DELAY);
            end
            
            didPass = true;
            output = 'Success.';
        end
        
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

