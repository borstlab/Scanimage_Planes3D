function configureFrameAcquiredEvent(obj,LSMCommand,commandArg)
%CONFIGUREFRAMEACQUIREDEVENT Shared gateway to Thorlabs LSM MEX functionality
%
%% SYNTAX
%   LSMCommand: One of the valid LSM commands, described below
%   commandArg: Some of the LSM commands accept a second argument
%
%% COMMANDS
%   initialize: initializes an LSM instance, i.e. the data stores for each new scanner object
%   configBuffers: initializes data buffers, again one per LSM instance. Should be called anytime any parameter affecting size of buffer changes.
%   configCallback: 
%   configLogFile: enables/disables logging (logDataToDisk), updates filename & file settings, sets new-file flag
%   newfile: sets new-file flag (informing logging thread to start new file)
%   preflight: calls PreflightAcquisition() with this scanner's single-frame buffer
%   postflight: calls PostflightAcquisition() with this scanner's single-frame buffer
%   setup: calls SetupAcquisition() with this scanner's single-frame buffer
%   start: calls StartAcquisition() with this scanner's single-frame buffer; sets acquireData flag (checked by frameAcquired callbackWrapper and the frame processing thread)
%   stop: calls StopAcquisition() with this scanner's single-frame buffer; stops logging thread, if any
%   flush: resets all data queues
%   newacq: flushes data from buffers; resets frame counters; starts logging thread; can be called in middle of acqusition (suspends acquireData flag) -- not sure why though
%   resume: sets acquireData flag to true
%   getdata: <commandArg = # frames to get> removes specified # of frames from circular data buffer
%   get: <commandArg = code for parameter to get> Retrieves LSM parameters, via GetAttrib call
%   destroy: Destroys /all/ scanner objects (data stores) 
%
%   test:
%   debugMessages:

%% NOTES
%   * Resume operation is not used at moment. Seems like one plan was to allow callback to be changed in middle of acquisition -- is this useful??
%   * Not 100% sure that the acquireData flag is needed either
%   * Not sure if there's any reason for flush operation to be user-facing -- can maybe remove


end

