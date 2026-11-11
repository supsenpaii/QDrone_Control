function destination=restore_source_model(name)
% Copy an intermediate source model to the root without overwriting files.
project=setup_project;
validateattributes(name,{'char'},{'row','nonempty'});
[folder,base,ext]=fileparts(name);
assert(isempty(folder),'Supply a model name only.');
if isempty(ext)
    if isfile(fullfile(project,'source_models',[base '.mdl'])),ext='.mdl';else,ext='.slx';end
end
assert(any(strcmp(ext,{'.mdl','.slx'})),'Only source model files can be restored.');
source=fullfile(project,'source_models',[base ext]);
destination=fullfile(project,[base ext]);
assert(isfile(source),'Source model not included: %s',source);
assert(~isfile(destination),'Destination already exists: %s',destination);
copyfile(source,destination);
fprintf('Copied source model to %s\n',destination);
end
