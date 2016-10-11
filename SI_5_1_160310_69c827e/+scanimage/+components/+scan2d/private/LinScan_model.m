%% LinScan
deviceNameAcq   = 'PXI1Slot3';      % string identifying NI DAQ board for PMT channels input
deviceNameGalvo = 'PXI1Slot3';      % string identifying NI DAQ board for controlling X/Y galvo. leave empty if same as deviceNameAcq
deviceNameAux   = 'PXI1Slot3';      % string identifying NI DAQ board for outputting clocks. leave empty if unused. Must be a X-series board

%Optional
channelsInvert = false;             % scalar or vector identifiying channels to invert. if scalar, the value is applied to all channels
beamDaqID = [];                     % Numeric: ID of the beam DAQ to use with the linear scan system
shutterIDs = 1;                     % Array of the shutter IDs that must be opened for linear scan system to operate

resScanframeClockIn = '';           % one of {'',PFI8} to which frame clock from ResScan is connected on Aux board. Leave empty for automatic routing via PXI bus
referenceClockIn = '';              % one of {'',PFI14} to which 10MHz reference clock is connected on Aux board. Leave empty for automatic routing via PXI bus

enableRefClkOutput = false;         % Enables/disables the export of the 10MHz reference clock on PFI14
%Scanner control
XMirrorChannelID = 0;               % The numeric ID of the Analog Output channel to be used to control the X Galvo.
YMirrorChannelID = 1;               % The numeric ID of the Analog Output channel to be used to control the y Galvo.

xGalvoAngularRange = 15;            % max range in optical degrees (pk-pk) for x galvo
yGalvoAngularRange = 15;            % max range in optical degrees (pk-pk) for y galvo

scanParkAngleX = -7.5;              % Numeric [deg]: Optical degrees from center position for X galvo to park at when scanning is inactive
scanParkAngleY = -7.5;              % Numeric [deg]: Optical degrees from center position for Y galvo to park at when scanning is inactive

voltsPerOpticalDegreeX = 1;         % galvo conversion factor from optical degrees to volts (negative values invert scan direction)
voltsPerOpticalDegreeY = 1;         % galvo conversion factor from optical degrees to volts (negative values invert scan direction)

scanOffsetAngleX = 0;               % angle in optical degrees to shift command waveform applied to X-scanner
scanOffsetAngleY = 0;               % angle in optical degrees to shift command waveform applied to Y-scanner

%Acquisition
channelIDs = [];                    % Array of numeric channel IDs for PMT inputs. Leave empty for default channels (AI0...AIN-1)
