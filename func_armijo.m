function [gamma,armijo_steps,objFunc] = func_armijo(objFuncValue,ii,namestr,gradmag,params)

% backtracking search (Armijo method)
% modified from 2010 m.bangert@dkfz.de (https://www.mathworks.com/matlabcentral/fileexchange/34835-optimization-tutorial?s_tid=prof_contriblnk)
% objFuncValue - current objective function value
% ii           - iteration of adjoint looping
% namestr      - string defining the name of studies, solvers, and datasets for the current index ii
% gradmag      - magnitude of gradient of cost function
%               *gradient is contained in the model (Ggrad or Agrad)
% params       - solver parameters

gammaMax    = params.armijo_gammaMax;       % this is the maximum step length
gradstepMin = params.armijo_gradstepMin;    % minimum value
gamma       = gammaMax;                     % starting value
beta        = params.armijo_beta;           % < 1 reduction factor of alpha (how fast to rescale)
mu          = params.armijo_mu;             % < 1/2 (choose initial target)

armijo_steps=0;
global model

if ii<(params.armijoskipcutoff+1) && params.armijoskipfirst==1

    disp(['        skipping armijo loop for ii=1, fixed gradient ' num2str(params.gradstep(ii))]);
    model.param.set('gradstep', num2str(params.gradstep(ii)));
    tic; model.sol(['sol_update_' namestr]).runAll; disp(['        update step complete, ' num2str(toc) ' [s]']);
    tic; model.sol('sol_forward').runAll; disp(['        forward step complete, ' num2str(toc) ' [s]']);
    [objFunc,~]=func_cost(ii+1,params);

else

    %initialize cost function and gradient step size
    disp(['    initializing armijo looping (k=0)'])
    model.param.set('gradstep', num2str(gamma/gradmag));
    tic; model.sol(['sol_update_' namestr]).runAll; disp(['        initial update step complete, ' num2str(toc) ' [s]'])

    tic; model.sol('sol_forward').runAll; disp(['        initial forward step complete, ' num2str(toc) ' [s]'])
    [objFunc,~]=func_cost(ii+1,params);

    objFuncTarget=objFuncValue - mu*gamma*gradmag;
    disp(['        cost target ______________________ : ' num2str(objFuncTarget)]);


    %sometimes if mu is too large the target objective function value will be negative - this loop crudely fixes that issue
    while objFuncTarget < 0
        disp(['        rescaling feasibility region']);
        mu=params.armijo_mureduce*mu;
        disp(['             new mu : ' num2str(mu)]);
        objFuncTarget=objFuncValue - mu*gamma*gradmag;
        disp(['             new cost target : ' num2str(objFuncTarget)]);
    end

disp(['        armijo loop : ' num2str(armijo_steps) ', gamma = ' num2str(gamma) ', gradstep = ' num2str(gamma/gradmag)]);

    while (gamma/gradmag > gradstepMin) & (objFunc > (objFuncValue - mu*gamma*gradmag)) %objFuncValue + mu*gamma*dir'*fgrad;

        disp(['    in armijo loop (k=' num2str(armijo_steps) ')'])
        gamma = beta*gamma;
        model.param.set('gradstep', num2str(gamma/gradmag));

        disp(['        armijo loop : ' num2str(armijo_steps) ', gamma = ' num2str(gamma) ', gradstep = ' num2str(gamma/gradmag)]);
        tic; model.sol(['sol_update_' namestr]).runAll; disp(['        update step complete, ' num2str(toc) ' [s]']);
%         model.result.dataset.remove('dset_forward');
        tic; model.sol('sol_forward').runAll; disp(['        forward step complete, ' num2str(toc) ' [s]']);
        [objFunc,~]=func_cost(ii+1,params);

        if gamma < 100*eps
            error('Error in Line search - alpha close to working precision');
        end

        armijo_steps=armijo_steps+1;

    end
end

return
