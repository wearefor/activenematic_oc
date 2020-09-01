function func_config_adjoint(ii,params)

% Adjoint Step 0
% w4/feq2 : Use initial ramp A(t)
% w4/feq3 : Uses Anew_a
% w4/feq5 : Uses Anew_b

global model

if ii==0

    %model.variable('var_Anew_0').active(true);
    %model.variable('var_Anew_b').active(false);
    %model.variable('var_Anew_a').active(false);
    model.study.create('std_adjoint_0');
    model.study('std_adjoint_0').create('time', 'Transient');
    model.study('std_adjoint_0').feature('time').set('activate', {'w' 'off' 'w2' 'off' 'w3' 'off' 'w4' 'on' 'w5' 'on' 'w6' 'on' 'w7' 'off' 'w8' 'off'});
    model.study('std_adjoint_0').label('Initial Adjoint Step');
    model.study('std_adjoint_0').feature('time').set('notstudy', 'std_forward_0');
    model.study('std_adjoint_0').feature('time').set('usesol', 'on');
    %model.study('std_adjoint_0').feature('time').set('disabledvariables', {'var_Anew_a' 'var_Anew_b'});
    model.study('std_adjoint_0').feature('time').set('tlist', 'range(Tf,-dt,0)');
    model.study('std_adjoint_0').feature('time').set('useadvanceddisable', true);
    model.study('std_adjoint_0').feature('time').set('notsolnum', 'auto');
    %model.study('std_adjoint_0').feature('time').set('disabledphysics', {'w4/wfeq3' 'w4/wfeq5' 'w4/wfeq4' 'w5/wfeq2'}); %last two are driving terms

    switch lower(params.controltype)
        case 'torque'
            model.study('std_adjoint_0').feature('time').set('disabledphysics', {'w4/wfeq2'});
            model.physics('w4').feature('wfeq7').set('weak',...
            {'(-1)*test(psixx)*psixy*G(t)';...
            'test(psixy)*psixx*G(t)'});
            model.physics('w4').feature('wfeq7').label('activetorque');
            model.physics('w4').feature('wfeq6').set('weak',...
            {'test(psixx)*(alpha0*(nuyy-nuxx))';...
            'test(psixy)*(-alpha0*(nuxy+nuyx))'});
            model.physics('w4').feature('wfeq6').label('activeterm_uniform');
        case 'stress'
            model.study('std_adjoint_0').feature('time').set('disabledphysics', {'w4/wfeq6' 'w4/wfeq7'});
            model.physics('w4').feature('wfeq2').set('weak',...
                {'test(psixx)*(A(t)*(nuyy-nuxx))';...
                'test(psixy)*(-A(t)*(nuxy+nuyx))'});
    end

    model.study('std_adjoint_0').feature('time').set('notsolmethod', 'sol');
    model.study('std_adjoint_0').feature('time').set('rtol', '1e-8');

    %%
    %% Adjoint Step: Initial Conditions
    model.physics('w4').feature('init1').set('psixx', ['A_Q_weight*(withsol(''sol_forward_0'',comp1.Qxx,setval(t,Tf))-withsol(''sol_isolate_'  params.targetstr ''',comp1.Qxx,setval(t,theta)))-radialcutoff(sqrt(x^2+y^2))*E_thetaforce_weight*2*(x*y/(x^2+y^2+epsreg)^(3/2))*(withsol(''sol_forward_0'',(comp1.Qxxx+comp1.Qxyy)*(-y/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxyx-comp1.Qxxy)*(x/(x^2+y^2+epsreg)^(1/2)),setval(t,Tf))-withsol(''sol_isolate_'  params.targetstr ''',(comp1.Qxxx+comp1.Qxyy)*(-y/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxyx-comp1.Qxxy)*(x/(x^2+y^2+epsreg)^(1/2)),setval(t,theta)))-radialcutoff(sqrt(x^2+y^2))*E_thetaforce_weight*(-y/(x^2+y^2+epsreg)^(1/2)*(withsol(''sol_forward_0'',(comp1.Qxxxx+comp1.Qxyxy)*(-y/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxyxx-comp1.Qxxxy)*(x/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxxx+comp1.Qxyy)*(x*y/(x^2+y^2+epsreg)^(3/2))+(comp1.Qxyx-comp1.Qxxy)*(1/(x^2+y^2+epsreg)^(1/2)-x^2/(x^2+y^2+epsreg)^(1/2)),setval(t,Tf))-withsol(''sol_isolate_'  params.targetstr ''',(comp1.Qxxxx+comp1.Qxyxy)*(-y/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxyxx-comp1.Qxxxy)*(x/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxxx+comp1.Qxyy)*(x*y/(x^2+y^2+epsreg)^(3/2))+(comp1.Qxyx-comp1.Qxxy)*(1/(x^2+y^2+epsreg)^(1/2)-x^2/(x^2+y^2+epsreg)^(1/2)),setval(t,theta)))-x/(x^2+y^2+epsreg)^(1/2)*(withsol(''sol_forward_0'',(comp1.Qxxxy+comp1.Qxyyy)*(-y/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxyxx-comp1.Qxxxy)*(x/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxxx+comp1.Qxyy)*(y^2/(x^2+y^2+epsreg)^(1/2)-1/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxyx-comp1.Qxxy)*(-x*y/(x^2+y^2+epsreg)^(3/2)),setval(t,Tf))-withsol(''sol_isolate_'  params.targetstr ''',(comp1.Qxxxy+comp1.Qxyyy)*(-y/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxyxx-comp1.Qxxxy)*(x/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxxx+comp1.Qxyy)*(y^2/(x^2+y^2+epsreg)^(1/2)-1/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxyx-comp1.Qxxy)*(-x*y/(x^2+y^2+epsreg)^(3/2)),setval(t,theta))))']);
    model.physics('w4').feature('init1').set('psixy', ['A_Q_weight*(withsol(''sol_forward_0'',comp1.Qxy,setval(t,Tf))-withsol(''sol_isolate_'  params.targetstr ''',comp1.Qxy,setval(t,theta)))-radialcutoff(sqrt(x^2+y^2))*E_thetaforce_weight*((y^2-x^2)/(x^2+y^2+epsreg)^(3/2))*(withsol(''sol_forward_0'',(comp1.Qxxx+comp1.Qxyy)*(-y/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxyx-comp1.Qxxy)*(x/(x^2+y^2+epsreg)^(1/2)),setval(t,Tf))-withsol(''sol_isolate_'  params.targetstr ''',(comp1.Qxxx+comp1.Qxyy)*(-y/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxyx-comp1.Qxxy)*(x/(x^2+y^2+epsreg)^(1/2)),setval(t,theta)))-radialcutoff(sqrt(x^2+y^2))*E_thetaforce_weight*(x/(x^2+y^2+epsreg)^(1/2)*(withsol(''sol_forward_0'',(comp1.Qxxxx+comp1.Qxyxy)*(-y/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxyxx-comp1.Qxxxy)*(x/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxxx+comp1.Qxyy)*(x*y/(x^2+y^2+epsreg)^(3/2))+(comp1.Qxyx-comp1.Qxxy)*(1/(x^2+y^2+epsreg)^(1/2)-x^2/(x^2+y^2+epsreg)^(1/2)),setval(t,Tf))-withsol(''sol_isolate_'  params.targetstr ''',(comp1.Qxxxx+comp1.Qxyxy)*(-y/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxyxx-comp1.Qxxxy)*(x/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxxx+comp1.Qxyy)*(x*y/(x^2+y^2+epsreg)^(3/2))+(comp1.Qxyx-comp1.Qxxy)*(1/(x^2+y^2+epsreg)^(1/2)-x^2/(x^2+y^2+epsreg)^(1/2)),setval(t,theta)))-y/(x^2+y^2+epsreg)^(1/2)*(withsol(''sol_forward_0'',(comp1.Qxxxy+comp1.Qxyyy)*(-y/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxyxx-comp1.Qxxxy)*(x/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxxx+comp1.Qxyy)*(y^2/(x^2+y^2+epsreg)^(1/2)-1/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxyx-comp1.Qxxy)*(-x*y/(x^2+y^2+epsreg)^(3/2)),setval(t,Tf))-withsol(''sol_isolate_'  params.targetstr ''',(comp1.Qxxxy+comp1.Qxyyy)*(-y/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxyxx-comp1.Qxxxy)*(x/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxxx+comp1.Qxyy)*(y^2/(x^2+y^2+epsreg)^(1/2)-1/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxyx-comp1.Qxxy)*(-x*y/(x^2+y^2+epsreg)^(3/2)),setval(t,theta))))']);

    %model.physics('w4').feature('init1').set('psixx', '0');
    %model.physics('w4').feature('init1').set('psixy', 'withsol(''sol_isolate_CW'',(comp1.Qxyxx-comp1.Qxxxy)*(x/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxxx+comp1.Qxyy)*(x*y/(x^2+y^2+epsreg)^(3/2)),setval(t,theta))');
    %model.physics('w4').feature('init1').set('psixx', 'A_Q_weight*(withsol(''sol_forward'',comp1.Qxx,setval(t,Tf))-withsol(''sol_isolate_CW'',comp1.Qxx,setval(t,theta)))');
    %model.physics('w4').feature('init1').set('psixy', 'A_Q_weight*(withsol(''sol_forward'',comp1.Qxy,setval(t,Tf))-withsol(''sol_isolate_CW'',comp1.Qxy,setval(t,theta)))');

    model.physics('w5').feature('init1').set('nux', ['B_U_weight*(withsol(''sol_forward_0'',comp1.ux,setval(t,Tf))-withsol(''sol_isolate_'  params.targetstr ''',comp1.ux,setval(t,theta)))']);
    model.physics('w5').feature('init1').set('nuy', ['B_U_weight*(withsol(''sol_forward_0'',comp1.uy,setval(t,Tf))-withsol(''sol_isolate_'  params.targetstr ''',comp1.uy,setval(t,theta)))']);

    model.physics('w4').feature('wfeq4').set('weak', {['C_Q_weight*test(psixx)*(withsol(''sol_forward_0'',comp1.Qxx,setval(t,t))-withsol(''sol_isolate_'  params.targetstr ''',comp1.Qxx,setval(t,t-Tf+theta)))'];...
        ['C_Q_weight*test(psixy)*(withsol(''sol_forward_0'',comp1.Qxy,setval(t,t))-withsol(''sol_isolate_'  params.targetstr ''',comp1.Qxy,setval(t,t-Tf+theta)))']});

    %%
    model.sol.create('sol_adjoint_0');
    model.sol('sol_adjoint_0').study('std_adjoint_0');
    model.sol('sol_adjoint_0').attach('std_adjoint_0');
    model.sol('sol_adjoint_0').create('st1', 'StudyStep');
    model.sol('sol_adjoint_0').create('v1', 'Variables');
    model.sol('sol_adjoint_0').create('t1', 'Time');
    model.sol('sol_adjoint_0').feature('t1').create('fc1', 'FullyCoupled');
    model.sol('sol_adjoint_0').feature('t1').feature.remove('fcDef');
    model.sol('sol_adjoint_0').feature('t1').feature('dDef').set('linsolver', 'pardiso');
    model.sol('sol_adjoint_0').feature('v1').set('notsolnum', 'auto');
    model.sol('sol_adjoint_0').feature('v1').set('notsolmethod', 'sol');
    model.sol('sol_adjoint_0').feature('v1').set('notsol', 'sol_forward_0');
    model.sol('sol_adjoint_0').feature('t1').set('tlist', 'range(Tf,-dt,0)');

    model.result.dataset.create('dset_adjoint_0', 'Solution');
    model.result.dataset('dset_adjoint_0').set('solution', 'sol_adjoint_0');

else

    model.result.dataset.remove('dset_adjoint');
    model.sol.remove('sol_adjoint');
    model.study.remove('std_adjoint');
    disp(['adjoint step :: ' num2str(ii) '... running']);

    model.study.create('std_adjoint');
    model.study('std_adjoint').create('time', 'Transient');
    model.study('std_adjoint').feature('time').set('activate', {'w' 'off' 'w2' 'off' 'w3' 'off' 'w4' 'on' 'w5' 'on' 'w6' 'on' 'w7' 'off' 'w8' 'off'});
    model.study('std_adjoint').label('Adjoint Step A');
    model.study('std_adjoint').feature('time').set('notstudy', 'std_forward');
    model.study('std_adjoint').feature('time').set('usesol', 'on');
    model.study('std_adjoint').feature('time').set('tlist', 'range(Tf,-dt,0)');
    model.study('std_adjoint').feature('time').set('useadvanceddisable', true);
    model.study('std_adjoint').feature('time').set('notsolnum', 'auto');

    switch lower(params.controltype)
        case 'torque'
            %model.study('std_adjoint_0').feature('time').set('disabledphysics', {'w4/wfeq2'});
            model.study('std_adjoint').feature('time').set('disabledphysics', {'w4/wfeq2'}); %fixed 2020.07.17

            if ii==1 && params.cont==0
                disp('index is 1, using sol_update_0 for active torque')
                model.physics('w4').feature('wfeq7').set('weak',...
                    {'test(psixx)*(withsol(''sol_update_0'',comp1.Gnew,setval(t,t))*psixy*(-1))';...
                    'test(psixy)*(withsol(''sol_update_0'',comp1.Gnew,setval(t,t))*psixx*(1))'});
            else
                if mod(ii,2)==0
                    disp('    index is even, using sol_update_b for active torque')
                    model.physics('w4').feature('wfeq7').set('weak',...
                        {'test(psixx)*(withsol(''sol_update_b'',comp1.Gnew,setval(t,t))*psixy*(-1))';...
                        'test(psixy)*(withsol(''sol_update_b'',comp1.Gnew,setval(t,t))*psixx*(1))'});
                elseif mod(ii,2)==1
                    disp('    index is odd, using sol_update_a for active torque')
                    model.physics('w4').feature('wfeq7').set('weak',...
                        {'test(psixx)*(withsol(''sol_update_a'',comp1.Gnew,setval(t,t))*psixy*(-1))';...
                        'test(psixy)*(withsol(''sol_update_a'',comp1.Gnew,setval(t,t))*psixx*(1))'});
                end
            end

        case 'stress'
            %model.study('std_adjoint_0').feature('time').set('disabledphysics', {'w4/wfeq6' 'w4/wfeq7'}); %disable torque and uniform stress in adjoint dynamics
            model.study('std_adjoint').feature('time').set('disabledphysics', {'w4/wfeq6' 'w4/wfeq7'}); %disable torque and uniform stress in adjoint dynamics


            if ii==1 && params.cont==0
                disp('index is 1, using sol_update_0 for active stress')
                model.physics('w4').feature('wfeq2').set('weak', {'test(psixx)*(withsol(''sol_update_0'',comp1.Anew,setval(t,t))*(nuyy-nuxx))';...
                    '-test(psixy)*(withsol(''sol_update_0'',comp1.Anew,setval(t,t))*(nuxy+nuyx))'});
            else
                if mod(ii,2)==0
                    disp('    index is even, using sol_update_b for active stress')
                    model.physics('w4').feature('wfeq2').set('weak', {'test(psixx)*(withsol(''sol_update_b'',comp1.Anew,setval(t,t))*(nuyy-nuxx))';...
                        '-test(psixy)*(withsol(''sol_update_b'',comp1.Anew,setval(t,t))*(nuxy+nuyx))'});
                elseif mod(ii,2)==1
                    disp('    index is odd, using sol_update_a for active stress')
                    model.physics('w4').feature('wfeq2').set('weak', {'test(psixx)*(withsol(''sol_update_a'',comp1.Anew,setval(t,t))*(nuyy-nuxx))';...
                        '-test(psixy)*(withsol(''sol_update_a'',comp1.Anew,setval(t,t))*(nuxy+nuyx))'});
                end
            end
    end

    model.study('std_adjoint').feature('time').set('notsolmethod', 'sol');
    model.study('std_adjoint').feature('time').set('rtol', '1e-8');

    model.physics('w4').feature('wfeq4').set('weak', {['C_Q_weight*test(psixx)*(withsol(''sol_forward'',comp1.Qxx,setval(t,t))-withsol(''sol_isolate_'  params.targetstr ''',comp1.Qxx,setval(t,t-Tf+theta)))'];...
        ['C_Q_weight*test(psixy)*(withsol(''sol_forward'',comp1.Qxy,setval(t,t))-withsol(''sol_isolate_'  params.targetstr ''',comp1.Qxy,setval(t,t-Tf+theta)))']});

    %% Adjoint Step: Initial Conditions
    model.physics('w4').feature('init1').set('psixx', ['A_Q_weight*(withsol(''sol_forward'',comp1.Qxx,setval(t,Tf))-withsol(''sol_isolate_'  params.targetstr ''',comp1.Qxx,setval(t,theta)))-radialcutoff(sqrt(x^2+y^2))*E_thetaforce_weight*2*(x*y/(x^2+y^2+epsreg)^(3/2))*(withsol(''sol_forward'',(comp1.Qxxx+comp1.Qxyy)*(-y/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxyx-comp1.Qxxy)*(x/(x^2+y^2+epsreg)^(1/2)),setval(t,Tf))-withsol(''sol_isolate_'  params.targetstr ''',(comp1.Qxxx+comp1.Qxyy)*(-y/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxyx-comp1.Qxxy)*(x/(x^2+y^2+epsreg)^(1/2)),setval(t,theta)))-radialcutoff(sqrt(x^2+y^2))*E_thetaforce_weight*(-y/(x^2+y^2+epsreg)^(1/2)*(withsol(''sol_forward'',(comp1.Qxxxx+comp1.Qxyxy)*(-y/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxyxx-comp1.Qxxxy)*(x/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxxx+comp1.Qxyy)*(x*y/(x^2+y^2+epsreg)^(3/2))+(comp1.Qxyx-comp1.Qxxy)*(1/(x^2+y^2+epsreg)^(1/2)-x^2/(x^2+y^2+epsreg)^(1/2)),setval(t,Tf))-withsol(''sol_isolate_'  params.targetstr ''',(comp1.Qxxxx+comp1.Qxyxy)*(-y/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxyxx-comp1.Qxxxy)*(x/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxxx+comp1.Qxyy)*(x*y/(x^2+y^2+epsreg)^(3/2))+(comp1.Qxyx-comp1.Qxxy)*(1/(x^2+y^2+epsreg)^(1/2)-x^2/(x^2+y^2+epsreg)^(1/2)),setval(t,theta)))-x/(x^2+y^2+epsreg)^(1/2)*(withsol(''sol_forward'',(comp1.Qxxxy+comp1.Qxyyy)*(-y/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxyxx-comp1.Qxxxy)*(x/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxxx+comp1.Qxyy)*(y^2/(x^2+y^2+epsreg)^(1/2)-1/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxyx-comp1.Qxxy)*(-x*y/(x^2+y^2+epsreg)^(3/2)),setval(t,Tf))-withsol(''sol_isolate_'  params.targetstr ''',(comp1.Qxxxy+comp1.Qxyyy)*(-y/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxyxx-comp1.Qxxxy)*(x/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxxx+comp1.Qxyy)*(y^2/(x^2+y^2+epsreg)^(1/2)-1/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxyx-comp1.Qxxy)*(-x*y/(x^2+y^2+epsreg)^(3/2)),setval(t,theta))))']);
    model.physics('w4').feature('init1').set('psixy', ['A_Q_weight*(withsol(''sol_forward'',comp1.Qxy,setval(t,Tf))-withsol(''sol_isolate_'  params.targetstr ''',comp1.Qxy,setval(t,theta)))-radialcutoff(sqrt(x^2+y^2))*E_thetaforce_weight*((y^2-x^2)/(x^2+y^2+epsreg)^(3/2))*(withsol(''sol_forward'',(comp1.Qxxx+comp1.Qxyy)*(-y/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxyx-comp1.Qxxy)*(x/(x^2+y^2+epsreg)^(1/2)),setval(t,Tf))-withsol(''sol_isolate_'  params.targetstr ''',(comp1.Qxxx+comp1.Qxyy)*(-y/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxyx-comp1.Qxxy)*(x/(x^2+y^2+epsreg)^(1/2)),setval(t,theta)))-radialcutoff(sqrt(x^2+y^2))*E_thetaforce_weight*(x/(x^2+y^2+epsreg)^(1/2)*(withsol(''sol_forward'',(comp1.Qxxxx+comp1.Qxyxy)*(-y/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxyxx-comp1.Qxxxy)*(x/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxxx+comp1.Qxyy)*(x*y/(x^2+y^2+epsreg)^(3/2))+(comp1.Qxyx-comp1.Qxxy)*(1/(x^2+y^2+epsreg)^(1/2)-x^2/(x^2+y^2+epsreg)^(1/2)),setval(t,Tf))-withsol(''sol_isolate_'  params.targetstr ''',(comp1.Qxxxx+comp1.Qxyxy)*(-y/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxyxx-comp1.Qxxxy)*(x/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxxx+comp1.Qxyy)*(x*y/(x^2+y^2+epsreg)^(3/2))+(comp1.Qxyx-comp1.Qxxy)*(1/(x^2+y^2+epsreg)^(1/2)-x^2/(x^2+y^2+epsreg)^(1/2)),setval(t,theta)))-y/(x^2+y^2+epsreg)^(1/2)*(withsol(''sol_forward'',(comp1.Qxxxy+comp1.Qxyyy)*(-y/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxyxx-comp1.Qxxxy)*(x/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxxx+comp1.Qxyy)*(y^2/(x^2+y^2+epsreg)^(1/2)-1/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxyx-comp1.Qxxy)*(-x*y/(x^2+y^2+epsreg)^(3/2)),setval(t,Tf))-withsol(''sol_isolate_'  params.targetstr ''',(comp1.Qxxxy+comp1.Qxyyy)*(-y/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxyxx-comp1.Qxxxy)*(x/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxxx+comp1.Qxyy)*(y^2/(x^2+y^2+epsreg)^(1/2)-1/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxyx-comp1.Qxxy)*(-x*y/(x^2+y^2+epsreg)^(3/2)),setval(t,theta))))']);

    %model.physics('w4').feature('init1').set('psixx', '0');
    %model.physics('w4').feature('init1').set('psixy', 'withsol(''sol_isolate_CW'',(comp1.Qxyxx-comp1.Qxxxy)*(x/(x^2+y^2+epsreg)^(1/2))+(comp1.Qxxx+comp1.Qxyy)*(x*y/(x^2+y^2+epsreg)^(3/2)),setval(t,theta))');
    %model.physics('w4').feature('init1').set('psixx', 'A_Q_weight*(withsol(''sol_forward'',comp1.Qxx,setval(t,Tf))-withsol(''sol_isolate_CW'',comp1.Qxx,setval(t,theta)))');
    %model.physics('w4').feature('init1').set('psixy', 'A_Q_weight*(withsol(''sol_forward'',comp1.Qxy,setval(t,Tf))-withsol(''sol_isolate_CW'',comp1.Qxy,setval(t,theta)))');

    model.physics('w5').feature('init1').set('nux', ['B_U_weight*(withsol(''sol_forward'',comp1.ux,setval(t,Tf))-withsol(''sol_isolate_'  params.targetstr ''',comp1.ux,setval(t,theta)))']);
    model.physics('w5').feature('init1').set('nuy', ['B_U_weight*(withsol(''sol_forward'',comp1.uy,setval(t,Tf))-withsol(''sol_isolate_'  params.targetstr ''',comp1.uy,setval(t,theta)))']);
    %% Adjoint Step: Run
    model.sol.create('sol_adjoint');
    model.sol('sol_adjoint').study('std_adjoint');
    model.sol('sol_adjoint').attach('std_adjoint');
    model.sol('sol_adjoint').create('st1', 'StudyStep');
    model.sol('sol_adjoint').create('v1', 'Variables');
    model.sol('sol_adjoint').create('t1', 'Time');
    model.sol('sol_adjoint').feature('t1').create('fc1', 'FullyCoupled');
    model.sol('sol_adjoint').feature('t1').feature.remove('fcDef');
    model.sol('sol_adjoint').feature('t1').feature('dDef').set('linsolver', 'pardiso');

    model.sol('sol_adjoint').feature('v1').set('notsolnum', 'auto');
    model.sol('sol_adjoint').feature('v1').set('notsolmethod', 'sol');
    model.sol('sol_adjoint').feature('v1').set('notsol', 'sol_forward');
    model.sol('sol_adjoint').feature('t1').set('tlist', 'range(Tf,-dt,0)');

    model.result.dataset.create('dset_adjoint', 'Solution');
    model.result.dataset('dset_adjoint').set('solution', 'sol_adjoint');

end

return
