function func_mphsave(ii,filenamestr)

global model

mphsave(model, ['outputs/' filenamestr '/forwardstep_' num2str(ii) '_' filenamestr '.mph'])

return
