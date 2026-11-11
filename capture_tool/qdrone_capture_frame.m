function packet=qdrone_capture_frame(image,fresh,t,id)
%#codegen
% Fixed wire header: QDCF + seven little-endian float64 values + raw pixels.
% Header: [camera_id frame_seq target_time height width channels bytes_per_element].
% Target time is software read time, not hardware exposure time.
persistent counts times
if isempty(counts),counts=zeros(5,1);times=zeros(5,1);end
index=double(id);
if fresh,counts(index)=counts(index)+1;times(index)=t;end
h=size(image,1);w=size(image,2);channels=size(image,3);
bytes=typecast(image(:),'uint8');
header=typecast(double([index;counts(index);times(index);h;w;channels;numel(bytes)/numel(image)]),'uint8');
packet=[uint8([81;68;67;70]);header(:);bytes(:)];
end
