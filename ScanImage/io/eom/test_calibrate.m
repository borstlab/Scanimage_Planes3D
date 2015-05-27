global state;

state.init.eom.ai = analoginput('nidaq', 1);
addchannel(state.init.eom.ai, 3);
%state.init.eom.ao = analogoutput('nidaq', 2);
%addchannel(state.init.eom.ao, 0);
state.init.eom.ao = getfield(state.init,['ao'  num2str(state.init.eom.scanLaserBeam)]);
state.init.eom.low_lim = 4;%Percentage of modulation voltage at which error begins to dominate (typically 30-40mV).
state.init.eom.calibration_interval = .005;%The step-size, for calibrating over the 0-2 volt range. At least 100 steps should be used, preferably more.
state.init.eom.rejected_light = 0;%The calibration input comes from accepted/rejected light.

state.init.eom.min = 1;
state.init.eom.min_power = 1;
state.init.eom.max_power = 100;

state.init.eom.max_limit = 100;

state.init.eom.filter = 0;%Enable/Disable filtering.

%[eom_max eom_min avg_dev] = calibrate_eom;

%fprintf(1, '\neom_max: %2.3f\neom_min: %2.3f\navg_dev: %2.3f\n\n', eom_max, eom_min, avg_dev);

%for i=1:10:100
   %fprintf(1, '\get_eom_modulation_voltage(%2f): %f\n', i, get_eom_modulation_voltage(i));
%    fprintf(1, 'EOM Modulation Voltage - %2.0f [%% Intensity] :: %2.3f [V]\n', i, state.init.eom.lut(i));
%end

%plot(1:100, state.init.eom.lut);
%xlabel('% Total Intensity');  
%ylabel('Modulation Voltage [V]');