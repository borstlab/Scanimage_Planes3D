function updateBidiScanDelay()
%UPDATEBIDISCANDELAY Update the scan delay values based on other acq parameters, when bidi scanning

global state gh

if state.acq.bidirectionalScan 
   
   %Handle the current fillFrac/msPerLine values
   samplesPerLine = round(state.acq.outputRate * state.acq.msPerLine * 1e-3);  %will be integer valued, but rounding done anyway
   numAcqSamples = round(samplesPerLine*state.acq.fillFraction);
   state.acq.scanDelay = (samplesPerLine - numAcqSamples) / (2 * state.acq.outputRate);
   state.internal.scanDelayGUI = state.acq.scanDelay * 1e6;   
   updateGUIByGlobal('state.internal.scanDelayGUI');
   
   %Handle the config fillFrac/msPerLine values
   [fillFraction, msPerLine] = decodeFillFractionGUI(state.internal.fillFractionGUIConfig);
   samplesPerLine = round(state.acq.outputRate * msPerLine * 1e-3);
   numAcqSamples = round(samplesPerLine*fillFraction);
   state.internal.scanDelayConfig = 1e6 * (samplesPerLine - numAcqSamples) / (2 * state.acq.outputRate);
   updateGUIByGlobal('state.internal.scanDelayConfig');            
    
end

