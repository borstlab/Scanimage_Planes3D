function frameAcquiredCallback(src, evnt)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    
%     disp (src);
%     disp(evnt);
%     return;
%     
global LSMTest;

%disp('frameAcqCallback');
%return;

frameCount = evnt.frameCount; %which frame is this?
droppedFramesLast = evnt.droppedFramesLast;
droppedFramesTotal = evnt.droppedFramesTotal;

hLSM = LSMTest.hLSM;
try
    if(evnt.droppedFramesLast > 0)
        %disp(['dropped ' num2str(evnt.droppedFramesLast) ' frame(s) after frame ' num2str(LSMTest.framesAcquired)]);
        disp(['dropped ' num2str(droppedFramesLast) ' frame(s) after frame ' num2str(frameCount)]);
    end
    
    
    if true
       
        %   tic;
        data = hLSM.getData(1);
        %fprintf(1,'Get Data Time: %g\n',toc);
        
        %disp(['frame= ' num2str(hLSM.frameCount)]);
        
        %         sz = size(data);
        %         fprintf(1,'size(data)=%s\tclass(data)=%s\n',mat2str(sz), class(data));
        %
        %    tic
        if ~isempty(data)
            channelsActive = hLSM.channelsActive;
            for i=1:length(channelsActive)
                chanIdx = channelsActive(i);
                set(LSMTest.(sprintf('image%dHnd',chanIdx)),'CData',data(:, :, i, 1)');
            end
        
                %figure(1);
                %imagesc(data(:, :, 1, 1)');
        
                %imagesc(data(:, :, 1, 1)', 'Parent', LSMTest.axes1Hnd);
        
                %                 chIdx = 1;
                %                 if(ismember(1, LSMTest.channelsActiveSnapshot))
                %                     set(LSMTest.image1Hnd,'CData',data(:, :, chIdx, 1)');
                %                     chIdx = chIdx + 1;
                %                 end
                %
                %                 if(ismember(2, LSMTest.channelsActiveSnapshot))
                %                     set(LSMTest.image2Hnd,'CData',data(:, :, chIdx, 1)');
                %                 end
                    %figure(2);
                    %imagesc(data(:, :, 2, 1)');
                    %    imagesc(data(:, :, 2, 1)', 'Parent', LSMTest.axes2Hnd);
        end
        %    fprintf(1, 'Display time =%g seconds', toc);
        %      [status, lastFrameIndex] = hLSM.statusAcquisitionEx();
        %     disp(['evnt.framesAvailable=' num2str(evnt.framesAvailable)...
        %         ' evnt.droppedFramesLast=' num2str(evnt.droppedFramesLast)...
        %         ' evnt.droppedFramesTotal=' num2str(evnt.droppedFramesTotal)...
        %         ' status=' num2str(status) ' lastFrameIndex=' num2str(lastFrameIndex) ]);
        
        
        
        if (mod(frameCount, 50) == 0)
            endTime = toc;
            tic;
            
            disp(['frameAcquiredCallback fps=' num2str(50/endTime)]); % ' (50 frame average) droppedFramesTotal=' num2str(evnt.droppedFramesTotal)]);
            
        end
        
        %LSMTest.lastFrameData = data;

    end
catch e
    disp(e);
end
%     hLSM.frameCount = hLSM.frameCount + 1;
    
    
%if(hLSM.framesAcquired >= LSMTest.framesToAcquire)
if(frameCount >= LSMTest.framesToAcquire)

%    disp(['evnt.droppedFramesTotal=' num2str(evnt.droppedFramesTotal)]);
    disp('frameAcquiredCallback: finished acquisition');    
    hLSM.stop();    
    
    
    %fprintf(1, 'frameCount=%d droppedFrames=%d', frameCount, hLSM.droppedFramesTotal);
    fprintf(1, 'frameCount=%d droppedFrames=%d\n', frameCount, droppedFramesTotal);

%    disp(['frameAcquiredCallback dropped frames= ' num2str(droppedFramesTotal) ' out of ' num2str(hLSM.frameCount) ' = ' num2str(100*double(droppedFramesTotal)/hLSM.frameCount) ' percent dropped']);

    
%        hLSM.apiCall('TeardownCamera');
 %       hLSM.hPMTModule.apiCall('TeardownDevice');
%        hLSM.configureFrameAcquiredEvent('clear');
 %       hLSM.needsConfig = 1;
%        hLSM.stopPMT;

else
    if strcmp(hLSM.triggerMode, 'SW_SINGLE_FRAME')
        hLSM.nextFrame();
    end
end

