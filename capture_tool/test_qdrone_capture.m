function test_qdrone_capture
folder=fileparts(mfilename('fullpath'));addpath(folder,fullfile(fileparts(folder),'optical_flow'));
test_qdrone_of;
t=0.033;a=(1:5)';s=(1:11)';f=(1:8)';
p=qdrone_capture_packet(t,a,s,f,2,t);assert(numel(p)==30 && p(9)==5 && p(24)==4);
[p1,rest,bad]=qdrone_parse_telemetry(p(1:11));assert(isempty(p1) && numel(rest)==11 && bad==0);
[p2,rest,bad]=qdrone_parse_telemetry([rest;p(12:end);p]);assert(isequal(p2,[p p]) && isempty(rest) && bad==0);
[p3,rest,bad]=qdrone_parse_telemetry([42;p]);assert(isequal(p3,p) && bad==1 && isempty(rest));
clear qdrone_capture_frame;
im=uint8(reshape(mod(0:480*640-1,251),480,640));
emptyPacket=qdrone_capture_frame(im,false,0,1);
[initial,remaining,bad]=qdrone_parse_frames(emptyPacket);assert(numel(initial)==1 && initial{1}.header(2)==0 && isempty(remaining) && bad==0);
[~,remaining,bad]=qdrone_parse_frames(zeros(1000000,1,'uint8'));assert(numel(remaining)==3 && bad==999997);
packet=qdrone_capture_frame(im,true,t,1);assert(numel(packet)==307260);
[frames,rest,bad]=qdrone_parse_frames(packet(1:100));assert(isempty(frames) && numel(rest)==100 && bad==0);
[frames,rest,bad]=qdrone_parse_frames([rest;packet(101:end);packet]);
assert(numel(frames)==2 && isempty(rest) && bad==0);
assert(isequal(reshape(frames{1}.pixels,480,640),im));
[frames,rest,bad]=qdrone_parse_frames([uint8(99);packet]);assert(numel(frames)==1 && bad==1 && isempty(rest));
depth=uint16(reshape(mod(0:480*640-1,4001),480,640));
packet=qdrone_capture_frame(depth,true,t,3);[frames,~,~]=qdrone_parse_frames(packet);
assert(isequal(reshape(typecast(frames{1}.pixels,'uint16'),480,640),depth));
rgb=cat(3,im,im,im);packet=qdrone_capture_frame(rgb,true,t,2);[frames,~,~]=qdrone_parse_frames(packet);
assert(isequal(reshape(frames{1}.pixels,480,640,3),rgb));
% Actual QUARC localhost transport: verify nonblocking connection and typed wire data.
listener=stream_listen('tcpip://localhost:18990',true);listenerCleanup=onCleanup(@()stream_close(listener)); %#ok<NASGU>
[client,pending]=stream_connect('tcpip://localhost:18990',true,65536);clientCleanup=onCleanup(@()stream_close(client)); %#ok<NASGU>
peer=[];connected=~pending;clock=tic;
while toc(clock)<3
    if isempty(peer)
        [candidate,wouldBlock]=stream_accept(listener,65536);
        if ~wouldBlock,peer=candidate;end
    end
    if ~connected,connected=any(stream_poll(client,0,'connect'));end
    if ~isempty(peer) && connected,break;end
    pause(0.005);
end
assert(connected && ~isempty(peer),'Local QUARC connection failed');peerCleanup=onCleanup(@()stream_close(peer)); %#ok<NASGU>
stream_send(peer,p);stream_flush(peer);received=[];clock=tic;
while numel(received)<numel(p) && toc(clock)<3
    [chunk,wouldBlock]=stream_receive(client,zeros(60,1)); %#ok<ASGLU>
    received=[received;chunk(:)]; %#ok<AGROW>
    pause(0.005);
end
assert(isequal(received,p),'QUARC typed transport changed telemetry payload');
fprintf('PASS: telemetry framing, fragmented packets, resynchronization, lossless grayscale/RGB/depth, QUARC TCP loopback.\n');
end

