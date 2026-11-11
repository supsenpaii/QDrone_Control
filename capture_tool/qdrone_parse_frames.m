function [packets,remainder,bad]=qdrone_parse_frames(buffer)
buffer=uint8(buffer(:));packets={};bad=0;
while numel(buffer)>=60
    % Scan once to resynchronize, avoiding quadratic byte-by-byte deletion.
    starts=find(buffer(1:end-3)==81 & buffer(2:end-2)==68 & buffer(3:end-1)==67 & buffer(4:end)==70,1);
    if isempty(starts)
        bad=bad+numel(buffer)-3;buffer=buffer(end-2:end);break;
    end
    if starts>1,bad=bad+starts-1;buffer=buffer(starts:end);end
    if numel(buffer)<60,break;end
    header=typecast(buffer(5:60),'double');header=header(:);
    if any(~isfinite(header)) || header(1)<1 || header(1)>5 || header(1)~=fix(header(1)) || ...
            header(2)<0 || header(2)~=fix(header(2)) || header(4)~=480 || header(5)~=640 || ...
            ~ismember(header(6),[1 3]) || ~ismember(header(7),[1 2])
        buffer(1)=[];bad=bad+1;continue;
    end
    n=prod(header(4:7));
    if numel(buffer)<60+n,break;end
    packets{end+1}=struct('header',header,'pixels',buffer(61:60+n)); %#ok<AGROW>
    buffer(1:60+n)=[];
end
remainder=buffer;
end
