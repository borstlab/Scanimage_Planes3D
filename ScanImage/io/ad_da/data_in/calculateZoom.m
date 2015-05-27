function calculateZoom(aiZoom, SamplesAcquired)
global state

try
	tempData = getdata(state.init.aiZoom, 'native');
	state.acq.rboxZoomSetting = round(10*mean(tempData)/1.11571428571429)/10;
	updateHeaderString('state.acq.rboxZoomSetting');
catch
	setStatusString('Error in reading rotation box zoom');
	disp('calculateZoom: caught error');
end
	