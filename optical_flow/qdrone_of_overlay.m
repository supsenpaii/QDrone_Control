function rgb=qdrone_of_overlay(image,flow)
%#codegen
% Display-only overlay. Raw input to the estimator is unchanged.
rgb=zeros(480,640,3,'uint8');
for r=1:480
    for c=1:640
        rgb(r,c,1)=image(r,c); rgb(r,c,2)=image(r,c); rgb(r,c,3)=image(r,c);
    end
end
color=uint8([255 50 50]);
if flow(4)>0.5, color=uint8([40 255 80]); end
for r=1:6
    for c=1:640
        for ch=1:3, rgb(r,c,ch)=color(ch); end
    end
end
width=round(640*max(0,min(1,flow(3))));
for r=9:14
    for c=1:640
        rgb(r,c,:)=uint8(0);
        if c<=width
            rgb(r,c,1)=color(1);rgb(r,c,2)=color(2);rgb(r,c,3)=color(3);
        end
    end
end
if flow(4)>0.5 && all(isfinite(flow(1:2)))
    dx=max(-120,min(120,8*flow(1))); dy=max(-100,min(100,8*flow(2)));
    rgb=stroke(rgb,320,240,320+dx,240+dy,color);
    lengthValue=hypot(dx,dy);
    if lengthValue>3
        ux=dx/lengthValue;uy=dy/lengthValue;
        rgb=stroke(rgb,320+dx,240+dy,320+dx-12*ux+6*uy,240+dy-12*uy-6*ux,color);
        rgb=stroke(rgb,320+dx,240+dy,320+dx-12*ux-6*uy,240+dy-12*uy+6*ux,color);
    end
    rgb=stroke(rgb,315,240,325,240,color);
    rgb=stroke(rgb,320,235,320,245,color);
else
    rgb=stroke(rgb,312,232,328,248,color);
    rgb=stroke(rgb,312,248,328,232,color);
end
end
function rgb=stroke(rgb,x0,y0,x1,y1,color)
steps=max(1,ceil(max(abs(x1-x0),abs(y1-y0))));
for k=0:steps
    c=round(x0+(x1-x0)*k/steps);r=round(y0+(y1-y0)*k/steps);
    for a=-1:1
        for b=-1:1
            rr=r+a;cc=c+b;
            if rr>=1 && rr<=480 && cc>=1 && cc<=640
                for ch=1:3,rgb(rr,cc,ch)=color(ch);end
            end
        end
    end
end
end
