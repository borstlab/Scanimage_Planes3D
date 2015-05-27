function tf = motorErrorPending()
%MOTORERRORQUERY Returns true if there is a pending motor error 

global state
tf = state.motor.motorOn && (state.motor.hMotor.lscErrPending || (state.motor.motorZOn && state.motor.hMotorZ.lscErrPending));


end

