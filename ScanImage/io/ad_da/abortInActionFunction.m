function abortInActionFunction
% abort GRAB or LOOP from within an action function
	stopGrab(true); %VI090309A: Identify as abort operation
	scim_parkLaser;
    closeShutter;	% BSMOD
	%putDataGrab; %VI090309B: Not used with new DAQmx interface
    flushAOData; %VI090309B: Refresh mirror data after parking beam
	pause(.01);

	
