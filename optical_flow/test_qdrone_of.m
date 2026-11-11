function test_qdrone_of
% Algorithm checks only; these do not validate flight or camera calibration.
rng(7);
small=uint8(randi([0 255],120,160));
image=uint8(kron(double(small),ones(4,4)));
clear qdrone_of_step;
y=qdrone_of_step(image,true,0,true); assert(y(4)==0);
y=qdrone_of_step(image,true,0.033,false);
assert(y(4)==1 && norm(y(1:2))<0.2,'Stationary frame rejected');
for shift=[-4 -2 1 3 4]
    clear qdrone_of_step;
    qdrone_of_step(image,true,0,true);
    moved=circshift(image,[4*shift,-4*shift]);
    y=qdrone_of_step(moved,true,0.033,false);
    assert(y(4)==1 && abs(y(1)+4*shift)<0.6 && abs(y(2)-4*shift)<0.6,'Translation/sign error');
end
y=qdrone_of_step(image,false,0.3,false); assert(y(4)==0,'Stale flow remained valid');
clear qdrone_of_step;
blank=zeros(480,640,'uint8');
qdrone_of_step(blank,true,0,true);
y=qdrone_of_step(blank,true,0.033,false); assert(y(4)==0,'Blank scene accepted');
clear qdrone_of_step;
qdrone_of_step(image,true,0,true);
y=qdrone_of_step(image,true,1,false); assert(y(4)==0,'Frame gap accepted');
y=qdrone_of_step(image,true,0.1,false); assert(y(4)==0,'Clock rewind accepted');
fprintf('PASS: stationary, signed translations, stale frames, blank scene, frame gap, clock rewind.\n');
end
