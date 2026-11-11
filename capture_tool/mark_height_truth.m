function mark_height_truth(heightMetres,label)
% Record a ruler-measured RELATIVE rise from the calibrated ground position.
% This marks data only; never modifies p0, scale, model parameters or control.
if nargin<2,label='ruler_measurement';end
assert(isnumeric(heightMetres)&&isscalar(heightMetres)&&isfinite(heightMetres),'Height must be one finite number in metres.');
observer=getappdata(0,'QDroneHeightObserver');
assert(isa(observer,'QDroneHeightObserver')&&isvalid(observer)&&~observer.Closing,'Start the live height observer first.');
observer.tick;
clock=toc(observer.StartClock);
assert(clock-observer.LastAdvance<0.75 && clock-observer.LastGoodRead<0.75,'No fresh viewing-block data.');
y=observer.Values;
assert(y(4)>0.5&&isfinite(y(1)),'Height reference not calibrated or calculation invalid.');
assert(any(strcmp(observer.LastState,{'running','external'})),'Height model is not running.');
label=regexprep(char(label),'[^A-Za-z0-9_ -]','_');
path=fullfile(observer.Session,'ground_truth.csv');exists=exist(path,'file')~=0;
f=fopen(path,'a');assert(f>=0,'Cannot write ground truth markers');cleanup=onCleanup(@()fclose(f)); %#ok<NASGU>
if ~exists
 fprintf(f,'host_utc_s,target_s,epoch,measured_relative_m,estimated_filtered_m,estimated_raw_m,reference_Pa,label\n');
end
fprintf(f,'%.6f,%.9f,%d,%.6f,%.9f,%.9f,%.9f,%s\n',observer.utc,observer.LastTime,observer.Epoch,heightMetres,y(1),y(2),y(5),label);
fprintf('Marked measured %.3f m; estimated filtered %.3f m; raw %.3f m; target time %.3f s.\n',heightMetres,y(1),y(2),observer.LastTime);
fprintf('Marker saved to %s. No calibration or target command changed.\n',path);
end
