function residual=qdrone_ry_aw_step(motor,rawTorque,limitedTorque,inverseMixer)
%#codegen
m=reshape(motor,4,1);raw=reshape(rawTorque,3,1);lim=reshape(limitedTorque,3,1);
inv=reshape(inverseMixer,4,4);residual=zeros(3,1);
if coder.target('MATLAB')
    if any(~isfinite([m;raw;lim;inv(:)])),residual(:)=NaN;return;end
    delta=inv*(min(max(m,0),1)-m);residual=lim-raw+delta(2:4);
else
    residual=QDroneRYPIBuildable.allocator(m,raw,lim,inv);
end
end
