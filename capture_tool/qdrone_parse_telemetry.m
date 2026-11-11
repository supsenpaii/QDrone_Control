function [packets,remainder,bad]=qdrone_parse_telemetry(buffer)
buffer=double(buffer(:));packets=zeros(30,0);bad=0;
while numel(buffer)>=30
    if buffer(1)~=51234567 || buffer(2)~=1 || any(~isfinite(buffer(3:4)))
        buffer(1)=[];bad=bad+1;continue;
    end
    packets(:,end+1)=buffer(1:30);buffer(1:30)=[]; %#ok<AGROW>
end
remainder=buffer;
end
