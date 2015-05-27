% --------------------------------------------------------------------
% Save the state data for Pockels cell operation.
function saveEomState(varargin);
global state;

lut = state.init.eom.lut;
eom_min = state.init.eom.min;
maxPower = state.init.eom.maxPower;
maxLimit = state.init.eom.maxLimit;

save PockelsCalibrationCurve lut eom_min maxPower maxLimit;

clear lut eom_min maxPower maxLimit;