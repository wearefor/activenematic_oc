function func_config_forward(ii,params)

global model

if ii==0

    model.study.create('std_forward_0');
    model.study('std_forward_0').create('time', 'Transient');
    model.study('std_forward_0').feature('time').set('activate', {'w' 'on' 'w2' 'on' 'w3' 'on' 'w4' 'off' 'w5' 'off'  'w6' 'off' 'w7' 'off' 'w8' 'off'});
    model.study('std_forward_0').label('Initial Forward Step');
    model.study('std_forward_0').feature('time').set('solnum', 'interp');
    model.study('std_forward_0').feature('time').set('useinitsol', 'on');
    model.study('std_forward_0').feature('time').set('rtol', '1e-8');
    model.study('std_forward_0').feature('time').set('useadvanceddisable', true);
    model.study('std_forward_0').feature('time').set('tlist', 'range(0,dt,Tf)');
    model.study('std_forward_0').feature('time').set('t', num2str(params.thetastart));
    %model.study('std_forward_0').feature('time').set('disabledphysics', {'w2/wfeq2'});
    %model.study('std_forward_0').feature('time').set('disabledvariables', {'var_Anew_0' 'var_Anew_a' 'var_Anew_b'});
    model.study('std_forward_0').feature('time').set('rtolactive', true);
    model.study('std_forward_0').feature('time').set('initstudy', ['std_isolate_' params.startstr]); %use stored solution for Initial Condtion
    model.study('std_forward_0').feature('time').set('initmethod', 'sol');

    model.sol.create('sol_forward_0');
    model.sol('sol_forward_0').study('std_forward_0');
    model.sol('sol_forward_0').attach('std_forward_0');
    model.sol('sol_forward_0').create('st1', 'StudyStep');
    model.sol('sol_forward_0').create('v1', 'Variables');
    model.sol('sol_forward_0').create('t1', 'Time');
    model.sol('sol_forward_0').feature('t1').create('fc1', 'FullyCoupled');
    model.sol('sol_forward_0').feature('t1').feature.remove('fcDef');

    model.result.dataset.create('dset_forward_0', 'Solution');
    model.result.dataset('dset_forward_0').set('solution', 'sol_forward_0');

    switch lower(params.controltype)
        case 'torque'
            disp('index is 0, using initial guess for active torque')
            model.study('std_forward_0').feature('time').set('disabledphysics', {'w2/wfeq4'});
            model.physics('w').feature('wfeq16').set('weak', {'test(Qxx)*Qxy*(G(t))'; 'test(Qxy)*Qxx*(-G(t))'});
        case 'stress'
            disp('index is 0, using initial guess for active stress')
            model.study('std_forward_0').feature('time').set('disabledphysics', {'w/wfeq16' 'w2/wfeq2'}); %disable torque and uniform stress in Q dynamics
            model.physics('w2').feature('wfeq4').set('weak', {'A(t)*(Qxxx+Qxyy)*test(ux)';'A(t)*(Qxyx-Qxxy)*test(uy)'});
    end
else

    if ii==1
        model.study.create('std_forward');
        model.study('std_forward').create('time', 'Transient');
        model.study('std_forward').feature('time').set('activate', {'w' 'on' 'w2' 'on' 'w3' 'on' 'w4' 'off' 'w5' 'off'  'w6' 'off' 'w7' 'off' 'w8' 'off'});
        model.study('std_forward').label('Forward Step A');
        model.study('std_forward').feature('time').set('solnum', 'interp');
        model.study('std_forward').feature('time').set('useinitsol', 'on');
        model.study('std_forward').feature('time').set('rtol', '1e-8');
        model.study('std_forward').feature('time').set('useadvanceddisable', true);
        model.study('std_forward').feature('time').set('tlist', 'range(0,dt,Tf)');
        model.study('std_forward').feature('time').set('t',  num2str(params.thetastart));
        %model.study('std_forward').feature('time').set('disabledphysics', {'w2/wfeq2'});
        %model.study('std_forward').feature('time').set('disabledvariables', {'var_Anew_0' 'var_Anew_a' 'var_Anew_b'});
        model.study('std_forward').feature('time').set('rtolactive', true);
        model.study('std_forward').feature('time').set('initstudy', ['std_isolate_' params.startstr]); %use stored solution for Initial Condtion
        model.study('std_forward').feature('time').set('initmethod', 'sol');

        model.sol.create('sol_forward');
        model.sol('sol_forward').study('std_forward');
        model.sol('sol_forward').attach('std_forward');
        model.sol('sol_forward').create('st1', 'StudyStep');
        model.sol('sol_forward').create('v1', 'Variables');
        model.sol('sol_forward').create('t1', 'Time');
        model.sol('sol_forward').feature('t1').create('fc1', 'FullyCoupled');
        model.sol('sol_forward').feature('t1').feature.remove('fcDef');
        model.sol('sol_forward').feature('t1').feature('dDef').set('linsolver', 'pardiso');

        model.result.dataset.create('dset_forward', 'Solution');
        model.result.dataset('dset_forward').set('solution', 'sol_forward');
    else
    end


    %turning these off at this stage just to avoid errors (w4 and w5 are
    %not being used!)
    model.physics('w4').feature('init1').set('psixx', '0');
    model.physics('w4').feature('init1').set('psixy', '0');
    model.physics('w5').feature('init1').set('nux', '0');
    model.physics('w5').feature('init1').set('nuy', '0');


    switch lower(params.controltype)
        case 'torque'
            model.study('std_forward').feature('time').set('disabledphysics', {'w2/wfeq4'}); %disable time dependent active stress and uniform active stress
            if ii==1 %%&& params.cont==0
                disp('index is 1, using sol_update_0 for active stress')
                model.physics('w').feature('wfeq16').set('weak', {'test(Qxx)*Qxy*withsol(''sol_update_0'',comp1.Gnew,setval(t,t))'; '(-1)*test(Qxy)*Qxx*withsol(''sol_update_0'',comp1.Gnew,setval(t,t))'});
            else
                if mod(ii,2)==0
                    disp('index is even, using sol_update_b for active stress')
                    model.physics('w').feature('wfeq16').set('weak', {'test(Qxx)*Qxy*withsol(''sol_update_b'',comp1.Gnew,setval(t,t))'; '(-1)*test(Qxy)*Qxx*withsol(''sol_update_b'',comp1.Gnew,setval(t,t))'});
                elseif mod(ii,2)==1
                    disp('index is odd, using sol_update_a for active stress')
                    model.physics('w').feature('wfeq16').set('weak', {'test(Qxx)*Qxy*withsol(''sol_update_a'',comp1.Gnew,setval(t,t))'; '(-1)*test(Qxy)*Qxx*withsol(''sol_update_a'',comp1.Gnew,setval(t,t))'});
                else
                end
            end
        case 'stress'
            model.study('std_forward').feature('time').set('disabledphysics', {'w2/wfeq2' 'w/wfeq16'}); %disable time dependent active torque

            if ii==1 %%&& params.cont==0
                disp('index is 1, using sol_update_0 for active stress')
                model.physics('w2').feature('wfeq4').set('weak', {'(withsol(''sol_update_0'',comp1.Anew,setval(t,t))*(Qxxx+Qxyy)+(Qxx*withsol(''sol_update_0'',comp1.Anewx,setval(t,t))+Qxy*withsol(''sol_update_0'',comp1.Anewy,setval(t,t))))*test(ux)';...
                    '(withsol(''sol_update_0'',comp1.Anew,setval(t,t))*(Qxyx-Qxxy)+(Qxy*withsol(''sol_update_0'',comp1.Anewx,setval(t,t))-Qxx*withsol(''sol_update_0'',comp1.Anewy,setval(t,t))))*test(uy)'});
            else
                if mod(ii,2)==0
                    disp('index is even, using sol_update_b for active stress')
                    model.physics('w2').feature('wfeq4').set('weak', {'(withsol(''sol_update_b'',comp1.Anew,setval(t,t))*(Qxxx+Qxyy)+(Qxx*withsol(''sol_update_b'',comp1.Anewx,setval(t,t))+Qxy*withsol(''sol_update_b'',comp1.Anewy,setval(t,t))))*test(ux)';...
                        '(withsol(''sol_update_b'',comp1.Anew,setval(t,t))*(Qxyx-Qxxy)+(Qxy*withsol(''sol_update_b'',comp1.Anewx,setval(t,t))-Qxx*withsol(''sol_update_b'',comp1.Anewy,setval(t,t))))*test(uy)'});
                elseif mod(ii,2)==1
                    disp('index is odd, using sol_update_a for active stress')
                    model.physics('w2').feature('wfeq4').set('weak', {'(withsol(''sol_update_a'',comp1.Anew,setval(t,t))*(Qxxx+Qxyy)+(Qxx*withsol(''sol_update_a'',comp1.Anewx,setval(t,t))+Qxy*withsol(''sol_update_a'',comp1.Anewy,setval(t,t))))*test(ux)';...
                        '(withsol(''sol_update_a'',comp1.Anew,setval(t,t))*(Qxyx-Qxxy)+(Qxy*withsol(''sol_update_a'',comp1.Anewx,setval(t,t))-Qxx*withsol(''sol_update_a'',comp1.Anewy,setval(t,t))))*test(uy)'});
                else
                end
            end

    end


end



return
