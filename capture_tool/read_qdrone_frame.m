function image=read_qdrone_frame(session,id,index)
% Read one losslessly stored image using frames.csv (index is per camera).
if nargin<3,index=1;end
t=readtable(fullfile(session,'frames.csv'));rows=find(t.camera_id==id);
assert(index>=1 && index<=numel(rows),'Frame index outside available range');r=rows(index);
f=fopen(fullfile(session,sprintf('camera_%d.bin',id)),'r','ieee-le');assert(f>=0);cleanup=onCleanup(@()fclose(f)); %#ok<NASGU>
assert(fseek(f,t.byte_offset(r),'bof')==0);
bytes=fread(f,t.payload_bytes(r),'*uint8');assert(numel(bytes)==t.payload_bytes(r));
if t.bytes_per_element(r)==2,values=typecast(bytes,'uint16');else,values=bytes;end
image=reshape(values,[t.height(r) t.width(r) t.channels(r)]);
end
