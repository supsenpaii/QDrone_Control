function y = qdrone_of_step(image, fresh, t, reset)
%#codegen
% 640x480 uint8 input. Output double[8]:
% [du_px dv_px quality valid frame_dt age textured_count inlier_count].
% Image motion (right/down positive), NOT aircraft velocity or position.
persistent previous previousTime lastGoodTime lastTick ready lastResult
if isempty(ready)
    previous = zeros(120,160,'single');
    previousTime = 0; lastGoodTime = -1e9; lastTick = 0;
    ready = false; lastResult = zeros(8,1);
end
if reset || ~isfinite(t) || t < lastTick
    ready = false; lastResult = zeros(8,1); lastGoodTime = -1e9;
end
lastTick = t;
y = lastResult;
y(6) = max(0,t-lastGoodTime);
if y(6) > 0.15, y(1:4) = 0; end
if ~fresh || ~isfinite(t), return; end
% Area downsample rather than decimation to reduce aliasing.
current = zeros(120,160,'single');
for r=1:120
    for c=1:160
        total = single(0);
        for a=0:3
            for b=0:3
                total = total + single(image(4*r-a,4*c-b));
            end
        end
        current(r,c) = total / single(4080);
    end
end
dt = t-previousTime;
y = zeros(8,1); y(5) = dt; y(6) = max(0,t-lastGoodTime);
if ~ready || dt < 0.005 || dt > 0.15
    previous = current; previousTime = t; ready = true;
    lastResult = y; return;
end
dx = zeros(35,1); dy = zeros(35,1); n=0; textured=0;
for r=20:20:100
    for c=20:20:140
        sumValue=single(0); sumSquare=single(0);
        for a=-4:4
            for b=-4:4
                v=previous(r+a,c+b);
                sumValue=sumValue+v; sumSquare=sumSquare+v*v;
            end
        end
        variance = sumSquare/81-(sumValue/81)^2;
        if variance < 0.0004, continue; end
        textured=textured+1;
        costs=zeros(13,13); best=1e30; bx=0; by=0;
        for sy=-6:6
            for sx=-6:6
                cost=single(0);
                for a=-4:4
                    for b=-4:4
                        d=previous(r+a,c+b)-current(r+a+sy,c+b+sx);
                        cost=cost+d*d;
                    end
                end
                costs(sy+7,sx+7)=double(cost)/81;
                if costs(sy+7,sx+7)<best
                    best=costs(sy+7,sx+7); bx=sx; by=sy;
                end
            end
        end
        % Reject search boundaries, bad matches and repeated textures.
        if abs(bx)==6 || abs(by)==6 || best>0.015, continue; end
        second=1e30;
        for sy=-6:6
            for sx=-6:6
                if abs(sx-bx)>1 || abs(sy-by)>1
                    second=min(second,costs(sy+7,sx+7));
                end
            end
        end
        if best > 0.8*second || second < 1e-8, continue; end
        n=n+1;
        dx(n)=4*(bx+subpixel(costs(by+7,bx+6),best,costs(by+7,bx+8)));
        dy(n)=4*(by+subpixel(costs(by+6,bx+7),best,costs(by+8,bx+7)));
    end
end
previous=current; previousTime=t;
y(7)=textured;
if n>=8
    mx=middle(dx,n); my=middle(dy,n); inliers=0;
    for k=1:n
        if hypot(dx(k)-mx,dy(k)-my)<=3, inliers=inliers+1; end
    end
    y(3)=inliers/35; y(8)=inliers;
    if inliers>=8 && inliers/n>=0.65
        y(1)=mx; y(2)=my; y(4)=1;
        lastGoodTime=t; y(6)=0;
    end
end
lastResult=y;
end

function d=subpixel(left,center,right)
den=left-2*center+right;
d=0;
if den>1e-9, d=max(-0.5,min(0.5,0.5*(left-right)/den)); end
end

function m=middle(v,n)
% Fixed allocation; insertion sort of only the populated elements.
for i=2:n
    z=v(i); j=i-1;
    while j>=1
        if v(j)<=z, break; end
        v(j+1)=v(j); j=j-1;
    end
    v(j+1)=z;
end
if mod(n,2)==0, m=(v(n/2)+v(n/2+1))/2;
else, m=v((n+1)/2); end
end
