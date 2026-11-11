function [flow,sequence,frameTime]=qdrone_capture_flow(image,fresh,t)
%#codegen
persistent seq stamp
if isempty(seq),seq=0;stamp=0;end
if fresh,seq=seq+1;stamp=t;end
flow=qdrone_of_step(image,fresh,t,false);
sequence=seq;frameTime=stamp;
end
