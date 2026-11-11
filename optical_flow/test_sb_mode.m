function test_sb_mode
% Shared C/MEX and wrapper checks, no hardware operations.
assert(isfile(fullfile(fileparts(mfilename('fullpath')),'qdrone_sb_mode_mex.mexw64')),'Run build_sb_native.cmd first.');
clear qdrone_sb_mode_step
for k=1:300
    [mode,~]=qdrone_sb_mode_step(1,1,k*.001,0,1);
    assert(mode==0,'Startup with SB down must not select FA.');
end
for k=301:600,[mode,~]=qdrone_sb_mode_step(-1,1,k*.001,0,1);end
assert(mode==0);
for k=601:900,[mode,d]=qdrone_sb_mode_step(1,1,k*.001,0,1);end
assert(mode==2&&d(1)==4);
[mode,~]=qdrone_sb_mode_step(0,1,.901,0,1);assert(mode==0);
clear qdrone_sb_mode_step
fprintf('PASS: SB wrapper/shared MEX: startup interlock, UP -> DOWN FA, middle exits.\n');
end
