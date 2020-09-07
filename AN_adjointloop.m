% XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
% Active Nematic Optimal Control
% Adjoint Loop Script
% (C) 2020 Michael M. Norton
% Brandeis University, Physics
% Pennsylvania State University, Center for Neural Engineering
% XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

close all;
clear all;
clc;

global model

% XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
% generate lists of parameters to sweep
Qweight_A_list=[0];
Qweight_C_list=[900];%ccw to cw/ccw torque: 700 %ccw to ccw stress 900;%%ccw to cw stress: [1400];%900;%[200 300 400 500 600 700 750];%900
params.period=5.7692;
theta_list_full=linspace(15-params.period,15,11);
theta_list=theta_list_full; %12.1154;% 9.2308;%9.9520;%
gradstepfactor_list=[1];
thetaforceweight_list =[0];
Nloop_list=[50];

%generate all permutations of parameter lists, search for paramsweep to see how these are utilized
paramsweep=combvec(theta_list,...
    Qweight_A_list,...
    gradstepfactor_list,...
    thetaforceweight_list,...
    Qweight_C_list,...
    Nloop_list);

[~,N_runs]=size(paramsweep);

% XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
% choose control goal (ccw or cw) and control field (torque or stress) and other settings
params.controltype='stress'; %or 'stress' 'torque'
params.startstr='CCW';       %pull initial condition from which attractor?
params.thetastart='3';       %phase of initial condition
params.targetstr='CW';       %target attract, phase defined within loop, search "params.theta"
params.optimalphase='false'; %optimize phase of target solution, this shouldn't break the code if true but is relatively untested
params.debugsave=0;          % 0 = only save final step of adjoint loop, 1 = save intermediate steps including adjoint steps (use for debugging)

params.limitcyclestr = 'LimitCycles_xxxxxxx'; %change this to the name of the file produced by AN_adjointloop_setup.m
params.np = 2;             % number of processors to use when creating initializing comsol server, I haven't noticed a huge boost in going beyond np=2
params.compstr='XXXXXXXX'; % make up a name for the computer this will run on... helpful if switching between multiple workstations
params.cont=0;             % continue from previous solution? keep this set at zero - params.cont=1 is out of date

params.Tf   =   2;      %duration of control window
params.dt   =   0.05;   %time step
params.tlist=   0:params.dt:params.Tf;

%there are sometimes settings that need to be adjusted for the different control fields, these scripts just help configure things
switch lower(params.controltype)
    case 'torque'

        switch lower(params.targetstr)
            case 'ccw'
                params.G0                   = 0.1;
            case 'cw'
                params.G0                   = -0.75;
        end

        params.armijoskipfirst      = 0; %1= skip, 0= no skip
        params.armijoskipcutoff     = 1; %preset step size
        params.gradstep             = [5e-4 1e-7 1e-7 1e-7 1e-7 1e-7]; %initial gradient step

        params.armijo_gammaMax      = 1; % this is the maximum step length
        params.armijo_gradstepMin   = 1e-11;
        params.armijo_beta          = 0.2;%4; % < 1 reduction factor of alpha
        params.armijo_mu            = 1e-6; %1e-3;%7e-6; %if large: large feasible area, if small: small feasible area
        params.armijo_mureduce      = 0.6;

        params.costtol              = 0*1e-5;
        params.Q_err_tol            = 0*0.02;

        params.thetastepmin_factor   = 1/20;    %fraction of a period
        params.thetastepdJdtheta_factor= 1e-3;  %multiplies the gradient dJdtheta

    case 'stress'
        params.G0                   = 0;
        params.ucut                 = 99; %harshcutoff

        params.control_max_weight   = 4000;
        params.control_max_soft     = 16;
        params.control_max_power    = 4;


        params.armijoskipfirst      = 0; %1= skip, 0= no skip?
        params.armijoskipcutoff     = 1; %number of preset steps to use

        % power law grad step size selection ?
        %params.gradstepMax          = 4e-3;
        %params.gradstepMin          = 5e-6;
        %params.gradstepP            = -7;       %-2.6;
        %params.gradstepoffset       = (params.gradstepMax-params.gradstepMin)^(1/params.gradstepP)-1;
        %params.gradstep             = ([1:1:max(Nloop_list)]+params.gradstepoffset).^(params.gradstepP)+params.gradstepMin; %initial gradient step

        % or ad hoc list of grad step sizes
        params.gradstep             = [0.0050 0.0005 0.0001 1.0000e-06 1.0000e-07 1.0000e-07 1.0000e-07 1.0000e-07];%

        params.armijo_gammaMax      = 1;        % this is the maximum step length
        params.armijo_gradstepMin   = 1e-11;    %5e-10;
        params.armijo_beta          = 0.2;      %0.6; % < 1 reduction factor of alpha
        params.armijo_mu            = 2e-6;     %(1e-8)/params.armijo_gammaMax;
        params.armijo_mureduce      = 0.6;

        params.costtol              = 1e-6;
        params.Q_err_tol            = 0*0.01;

        params.thetastepmin_factor   = 1/20;    %fraction of a period
        params.thetastepdJdtheta_factor= 1e-3;  %multiplies the gradient dJdtheta

end

% choose which a parameter set to use
i_set=input(['Select a data set index between 1 and ' num2str(N_runs) ' : ']);

for i_loop=i_set;
    filenamestr=[datestr(now,'yyyymmdd_HHMM') '_' num2str(i_loop)];
    %% Configure Directories
    if strcmp(params.compstr,'XXXXXXXX')==1
        disp(['Configured for ' params.compstr])
        pathstr_main='/home/user/Documents/AN_control/';    % home directory of scripts
        cd(pathstr_main)
        addpath('/usr/local/comsol52/multiphysics/mli');    % location of matlab livelink
        addpath('/usr/local/comsol52/multiphysics/mli/startup');
        mkdir([pathstr_main 'outputs/'],filenamestr)        % create a directory for the current simulation
        pathstr=[pathstr_main 'outputs/' filenamestr '/'];
    else
    end

    jj=1;
    i_offset=0;
    %some code to try different ports for comsol mphserver if they are unavailable
    while jj == 1
        try
            jj=jj-1;
            %s2 = system(['comsol mphserver -np ' num2str(params.np) ' -mpmode owner &']);
            s2 = system(['comsol mphserver -np ' num2str(params.np) ' -port ' num2str(2036+(i_loop-1)+i_offset) ' -mpmode owner &']);
            pause(10)
            disp(['attemping comsol server start on port ' num2str(2036+(i_loop-1+i_offset))])
            mphstart(2036+(i_loop-1+i_offset))
        catch
            disp(['failed port ' num2str(2036+(i_loop-1+i_offset)) ', trying a new port'])
            jj=jj+1;
            i_shift=input(['select a port offset : ']);
            i_offset=i_offset+i_shift;
        end
    end

    import com.comsol.model.*
    import com.comsol.model.util.*
    ModelUtil.showProgress([pathstr 'log.txt']);

    disp(['start time: ' datestr(now,'mmmm dd, yyyy HH:MM:SS.FFF')])

    if params.cont==1
        solpathstr= [pathstr_main 'outputs/' params.solnamestr '/forwardstep_' params.soliter '_' params.solnamestr '.mph'];%build path from name
        model=mphload([solpathstr]);
    elseif params.cont==0
        %model=mphload('LimitCycles_20191211_1505.mph');
        model=mphload([params.limitcyclestr '.mph']);
    else
    end

    params.theta=paramsweep(1,i_loop); %stand in for phase - time of target solution to shoot for-if unspecified, probably t=10;
    params.A_Q_weight=paramsweep(2,i_loop);
    params.B_U_weight=0;
    params.C_Q_weight=paramsweep(5,i_loop);
    params.D_U_weight=0;
    params.E_thetaforce_weight=paramsweep(4,i_loop);%*params.A_Q_weight;
    params.Nloop=paramsweep(6,i_loop);


    disp(['///\\\///\\\///\\\///\\\///\\\///\\\///\\\///\\\///\\\///\\\///\\\']);
    disp(['\\\///\\\///\\\///\\\///\\\///\\\///\\\///\\\///\\\///\\\///\\\///']);
    disp(['Initial State: ' params.startstr ', Initial Phase: ' num2str(params.thetastart)]);
    disp(['Target State: ' params.targetstr ', Target Phase: ' num2str(params.theta)]);
    disp(['Terminal Q Cost: ' num2str(params.A_Q_weight) ', Stage Q Cost: ' num2str(params.C_Q_weight)]);
    disp(['Terminal u Cost: ' num2str(params.B_U_weight)  ', Stage u Cost: ' num2str(params.D_U_weight)])
    disp(['Theta Force Cost: ' num2str(params.E_thetaforce_weight)]);
    disp(['Control Max Weight: ' num2str(params.control_max_weight)]);
    disp(['Actuation Force Type: ' num2str(params.controltype)]);
    disp(['///\\\///\\\///\\\///\\\///\\\///\\\///\\\///\\\///\\\///\\\///\\\']);

    model.param.set('gradstep', num2str(params.gradstep(1)));
    model.param.set('A_Q_weight',  num2str(params.A_Q_weight));
    model.param.set('B_U_weight',  num2str(params.B_U_weight));
    model.param.set('C_Q_weight',  num2str(params.C_Q_weight));
    model.param.set('D_U_weight',  num2str(params.D_U_weight));
    model.param.set('Gamma_alpha', '0.1');              %weight on gradients in alpha
    model.param.set('Gamma_g', '0.1');                  %weight on gradients in g
    model.param.set('control_max_weight',  num2str(params.control_max_weight));
    model.param.set('control_max_soft',  num2str(params.control_max_soft));
    model.param.set('control_max_power',  num2str(params.control_max_power));


    model.param.set('E_thetaforce_weight',  num2str(params.E_thetaforce_weight));
    %model.param.set('epsreg',  num2str(params.epsreg));
    model.param.set('epsreg',  '1e-8');
    model.param.set('Tf', num2str(params.Tf));
    model.param.set('dt', num2str(params.dt));
    model.param.set('slope', '0');
    model.param.set('theta', num2str(params.theta));
    model.param.set('Rcutoff', '0.1');
    model.param.set('Rcutoff_transition', '0.1');
    model.param.set('dJdtheta', '0');

    model.param.set('G0', num2str(params.G0));
    model.func('pw2').set('pieces', {'0' 'Tf/2' 'G0*(1-x/(Tf/2))' 'Tf/2' 'Tf' '0';  ...
    'Tf/2' 'Tf' '0' '' '' ''});

    model.func('pw2').set('smoothzone', '0.05');
    model.func('pw2').set('smooth', 'contd2');
    model.func('pw2').set('funcname', 'G');

    model.func.create('step1', 'Step');
    model.func('step1').label('radialcutoff');
    model.func('step1').set('location', 'Rcutoff');
    model.func('step1').set('smooth', 'Rcutoff_transition');
    model.func('step1').set('funcname', 'radialcutoff');

    model.param.set('U0', num2str(params.ucut));
    model.func.create('pw3', 'Piecewise');
    model.func('pw3').setIndex('pieces', '-100', 0, 0);
    model.func('pw3').setIndex('pieces', 'U0', 0, 1);
    model.func('pw3').setIndex('pieces', '-U0', 0, 2);
    model.func('pw3').setIndex('pieces', '-U0', 0, 1);
    model.func('pw3').setIndex('pieces', '-U0', 1, 0);
    model.func('pw3').setIndex('pieces', 'U0', 1, 1);
    model.func('pw3').setIndex('pieces', 'x', 1, 2);
    model.func('pw3').setIndex('pieces', 'U0', 2, 0);
    model.func('pw3').setIndex('pieces', '100', 2, 1);
    model.func('pw3').setIndex('pieces', 'U0', 2, 2);
    model.func('pw3').set('smooth', 'contd2');
    model.func('pw3').set('zonelengthtype', 'absolute');
    model.func('pw3').set('extrap', 'interior');
    model.func('pw3').set('funcname', 'controlcutoff');

    save([pathstr 'params_' filenamestr '.mat'],'params')

    %% Initialization
    if params.cont==0 %
        %tic
        % initial guess: constant
        model.func('pw1').set('pieces', {'0' '5' 'alpha0'; '5' '10' 'alpha0'});
        %model.func('pw1').set('pieces', {'0' 'Tf/2' 'alpha0*(1-x*slope)'; 'Tf/2' 'Tf' 'alpha0*((x-Tf/2)*slope+(1-Tf*slope/2))'});
        model.func('pw1').set('pieces', {'0' '5' 'alpha0'; '5' '10' 'alpha0'});
        %model.func('an12').set('expr', 'tanh((t-2)*1.25)/2+1/2');
        model.func('an12').set('expr', '1');

        ii=0;

        % Forward Step 0
        disp(['forward step :: ' num2str(ii) '... running'])
        func_config_forward(ii,params)
        tic; model.sol('sol_forward_0').runAll;         disp(['    forward step complete, ' num2str(toc) ' [s]']);


        if params.debugsave==1
            disp(['forward step :: ' num2str(ii) '... saving'])
            mphsave(model, ['outputs/' filenamestr '/forwardstep_' num2str(ii) '_' filenamestr '.mph'])
            disp(['forward step :: ' num2str(ii) '... saved'])
        else
        end

        func_plotcontrol(ii,params)

        disp(['adjoint step :: ' num2str(ii) '... running']);

        func_config_adjoint(ii,params);;
        tic; model.sol('sol_adjoint_0').runAll;          disp(['    adjoint step complete, ' num2str(toc) ' [s]']);


        %disp(['adjoint step :: ' num2str(ii) '... complete']);
        if params.debugsave==1
            disp(['adjoint step :: ' num2str(ii) '... saving'])
            mphsave(model, ['outputs/' filenamestr '/adjointstep_' num2str(ii) '_' filenamestr '.mph'])
            disp(['adjoint step :: ' num2str(ii) '... saved'])
        else
        end
        %
        %toc
    else
        disp(['running from previous solution, no initialization'])
    end

    %% XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
    % XXXXXXXXXXXXXXXXXXXXXX Begin Adjoint Loop XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
    % XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

    for ii=1:params.Nloop
        %tic

        disp(['///\\\///\\\///\\\///\\\///\\\///\\\///\\\///\\\///\\\///\\\///\\\']);
        disp(['            iteration ' num2str(ii) ' of ' filenamestr])
        disp(['\\\///\\\///\\\///\\\///\\\///\\\///\\\///\\\///\\\///\\\///\\\///']);

        %a crude way to chart the progress of the control maneuver is to calculation the circulation at the final state,
        if ii==1
            circ = mphint2(model,{'(-ux*y+uy*x)/sqrt(x^2+y^2)'},'surface','dataset','dset_forward_0','t',params.Tf);
        else
            circ = mphint2(model,{'(-ux*y+uy*x)/sqrt(x^2+y^2)'},'surface','dataset','dset_forward','t',params.Tf);
        end
        disp(['    circulation at T=' num2str(params.Tf) ': ' num2str(circ)])

        datasave.circ(ii)=circ;

        % during the control field update step, the previous control field is needed,
        % to avoid saving all previous steps, switch between A and B datasets

        if ii==1                                % first iteration
            namestr='0';
        else
            if mod(ii,2)==1                     % odd ii : A side
                namestr='a';
            elseif mod(ii,2)==0                 % even ii : B side
                namestr='b';
            end
        end

        disp(['calculating cost function for step :: ' num2str(ii) ' ...'])
        %evaluate cost function for the current control (before updating):
        %if ii==1 %only need to do this for first ii, afterward, func_armijo() will calculate
        [objFuncValue,~]=func_cost(ii,params);

        %else
        %end

        % XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
        % XXXXXXXXXXXXXXXXXXX Control Field Update Step XXXXXXXXXXXXXXXXXXXXXX
        % XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

        %initialize update
        func_config_update(namestr,ii,params);

        %run update just so we have the gradient of J w.r.t. alpha(x,t) or g(x,t) calculated, need it for armijo stepping
        model.param.set('gradstep', '1e-10');
        disp(['update step :: ' num2str(ii) ' running...'])
        model.sol(['sol_update_' namestr]).runAll;
        disp(['update step :: ' num2str(ii) ' complete'])

        switch lower(params.optimalphase)

            case 'true'
                thetastep=params.thetastepdJdtheta_factor*str2num(model.param.get('dJdtheta'));
                disp(['     dJdtheta = ' num2str(thetastep)])
                while abs(thetastep) > params.period*params.thetastepmin_factor
                    thetastep=thetastep*0.5;
                    %disp(['     dJdtheta = ' num2str(thetastep)])
                end
                disp(['     old theta = ' num2str(str2num(model.param.get('theta')))])
                theta_new=str2num(model.param.get('theta'))-thetastep;
                datasave.theta(ii)=theta_new;
                model.param.set('theta', num2str(theta_new));
                disp(['     new theta = ' num2str(theta_new)])
            case 'false'
                disp(['**theta optimization is off, checking:**'])
                disp(['     new theta = ' num2str(str2num(model.param.get('theta')))]);
                disp(['     old theta = ' num2str(params.theta)]);
        end

        switch lower(params.controltype)
            case 'stress'
                controlgrad_int=mphint2(model,{'Agrad^2'},'surface','dataset',['dset_update_' namestr],'t',params.tlist,'intorder',2);
                disp('----------------------------------------------------------------')
                disp(['     plotting Agrad^2, scale = ' num2str(max(controlgrad_int))])
                asciiplot(params.tlist,controlgrad_int);
                disp('----------------------------------------------------------------')
                controlgrad_mag=sqrt(trapz(params.tlist,controlgrad_int));
            case 'torque'
                controlgrad_int=mphint2(model,{'Ggrad^2'},'surface','dataset',['dset_update_' namestr],'t',params.tlist,'intorder',2);
                disp('----------------------------------------------------------------')
                disp(['     plotting Ggrad^2, scale = ' num2str(max(controlgrad_int))])
                asciiplot(params.tlist,controlgrad_int);
                disp('----------------------------------------------------------------')
                controlgrad_mag=sqrt(trapz(params.tlist,controlgrad_int));
        end

        disp(['        gradient magnitude : ' num2str(controlgrad_mag)])

        if params.debugsave==1
            disp(['    update step :: ' num2str(ii) ' saving...'])
            mphsave(model, ['outputs/' filenamestr '/updatestep_' num2str(ii) '_' filenamestr '.mph'])
            disp(['    update step :: ' num2str(ii) ' saved!'])
            disp(['    current time: ' datestr(now,'mmmm dd, yyyy HH:MM:SS.FFF')])
        else
        end

        % XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
        % XXXXXXXXXXXXXXXXXXXXXXXXXX Forward Step XXXXXXXXXXXXXXXXXXXXXXXXXXXX
        % XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

        %initialize forward solver but don't run anything yet
        func_config_forward(ii,params);

        %now find a gradient step size that ensures a decrease in the objective function
        disp(['    starting armijo looping'])
        [~,~,~] = func_armijo(objFuncValue,ii,namestr,controlgrad_mag,params);

        [objFuncValue,Q_term_err]=func_cost(ii+1,params);
        datasave.Q_term_err(ii)=Q_term_err;
        datasave.cost(ii) = objFuncValue;

        %now update for real! but...
        % armijo function updates control and forward time step in last
        % step, so no need to re-run 'sol_update_' and 'sol_forward'

        disp(['    update step :: ' num2str(ii) ' complete']);
        disp(['    forward step :: ' num2str(ii) ' complete']);

        func_plotcontrol(ii,params)
        datasave.gradstep(ii)=str2num(model.param.get('gradstep'));

        save([pathstr 'debug.mat'],'params','datasave','paramsweep','i_loop');

        % stop condition
        if ii > 1
            deltacost=abs(datasave.cost(ii)-datasave.cost(ii-1))/datasave.cost(ii-1);
            if deltacost < params.costtol || Q_term_err < params.Q_err_tol
                disp('cost function convered to within tolerance, saving...')
                mphsave(model, ['outputs/' filenamestr '/forwardstep_' num2str(ii) '_' filenamestr '.mph'])
                disp(['final forward step :: ' num2str(ii) ' saved!']);
                disp(['end time: ' datestr(now,'mmmm dd, yyyy HH:MM:SS.FFF')])
                break
            else
            end
        else
        end

        if ii==params.Nloop
            disp(['final forward step :: ' num2str(ii) ' saving....']);
            mphsave(model, ['outputs/' filenamestr '/forwardstep_' num2str(ii) '_' filenamestr '.mph'])
            disp(['final forward step :: ' num2str(ii) ' saved!']);
            disp(['end time: ' datestr(now,'mmmm dd, yyyy HH:MM:SS.FFF')])
            break
        else
        end

        if params.debugsave==1
            disp(['    forward step :: ' num2str(ii) ' saving...'])
            mphsave(model, ['outputs/' filenamestr '/forwardstep_' num2str(ii) '_' filenamestr '.mph'])
            disp(['    forward step :: ' num2str(ii) ' saved!'])
            disp(['    current time: ' datestr(now,'mmmm dd, yyyy HH:MM:SS.FFF')])
        else
        end

        % XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
        % XXXXXXXXXXXXXXXXXXXXXXXXXX Adjoint Step XXXXXXXXXXXXXXXXXXXXXXXXXXXX
        % XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

        func_config_adjoint(ii,params);
        tic; model.sol('sol_adjoint').runAll;          disp(['    adjoint step complete, ' num2str(toc) ' [s]']);

        disp(['    adjoint step :: ' num2str(ii) ' complete']);
        if params.debugsave==1
            disp(['    adjoint step :: ' num2str(ii) ' saving...'])
            mphsave(model, ['outputs/' filenamestr '/adjointstep_' num2str(ii) '_' filenamestr '.mph'])
            disp(['adjoint step :: ' num2str(ii) ' saved!'])
            disp(['    current time: ' datestr(now,'mmmm dd, yyyy HH:MM:SS.FFF')])
        else
        end

    end

    %% XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
    % XXXXXXXXXXXXXXXXXXXXXXXX End Adjoint Loop XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
    % XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

end
