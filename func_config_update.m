function func_config_update(namestr,ii,params)

global model

%clean up
model.result.dataset.remove(['dset_update_' namestr]);
model.sol.remove(['sol_update_' namestr]);
model.study.remove(['std_update_' namestr]);


%configure
model.study.create(['std_update_' namestr]);
model.study(['std_update_' namestr]).create('time', 'Transient');

model.study(['std_update_' namestr]).label(['Update Step' namestr]);
model.study(['std_update_' namestr]).feature('time').set('solnum', 'interp');
model.study(['std_update_' namestr]).feature('time').set('rtol', '1e-8');
model.study(['std_update_' namestr]).feature('time').set('useadvanceddisable', true);
model.study(['std_update_' namestr]).feature('time').set('tlist', 'range(0,dt,Tf)');
model.study(['std_update_' namestr]).feature('time').set('rtolactive', true);

switch lower(params.controltype)

    case 'torque'

        model.study(['std_update_' namestr]).feature('time').set('activate', {'w' 'off' 'w2' 'off' 'w3' 'off' 'w4' 'off' 'w5' 'off'  'w6' 'off' 'w7' 'off' 'w8' 'on'});

        switch lower(namestr)
            %
            case '0'
                disp('index is 0, using initial guess for active torque')
                model.study(['std_update_' namestr]).feature('time').set('disabledphysics', {'w8/wfeq2' 'w8/wfeq3'});
                model.physics('w8').feature('wfeq1').set('weak', {'(G(t)-gradstep*Ggrad-Gnew)*test(Gnew)';...
                    'test(Ggrad)*(withsol(''sol_forward_0'',comp1.Qxy,setval(t,t))*withsol(''sol_adjoint_0'',comp1.psixx,setval(t,t))-withsol(''sol_forward_0'',comp1.Qxx,setval(t,t))*withsol(''sol_adjoint_0'',comp1.psixy,setval(t,t))+G(t)-Ggrad)'});
            case 'a'
                model.study(['std_update_' namestr]).feature('time').set('disabledphysics', {'w8/wfeq1' 'w8/wfeq3'});
                disp('index is even, using sol_update_b for active torque')
                model.physics('w8').feature('wfeq2').set('weak', {'((withsol(''sol_update_b'',comp1.Gnew,setval(t,t))-gradstep*Ggrad)-Gnew)*test(Gnew)';...
                    'test(Ggrad)*(withsol(''sol_forward'',comp1.Qxy,setval(t,t))*withsol(''sol_adjoint'',comp1.psixx,setval(t,t))-withsol(''sol_forward'',comp1.Qxx,setval(t,t))*withsol(''sol_adjoint'',comp1.psixy,setval(t,t))-Gamma_g*withsol(''sol_update_b'',comp1.Gnewxx+comp1.Gnewyy,setval(t,t))+withsol(''sol_update_b'',comp1.Gnew,setval(t,t))-Ggrad)'});
            case 'b'
                model.study(['std_update_' namestr]).feature('time').set('disabledphysics', {'w8/wfeq1' 'w8/wfeq2'});
                if ii==2
                    disp('index is 2, using sol_update_0 for active torque')
                    model.physics('w8').feature('wfeq3').set('weak', {'((withsol(''sol_update_0'',comp1.Gnew,setval(t,t))-gradstep*Ggrad)-Gnew)*test(Gnew)';...
                        'test(Ggrad)*(withsol(''sol_forward'',comp1.Qxy,setval(t,t))*withsol(''sol_adjoint'',comp1.psixx,setval(t,t))-withsol(''sol_forward'',comp1.Qxx,setval(t,t))*withsol(''sol_adjoint'',comp1.psixy,setval(t,t))-Gamma_g*withsol(''sol_update_0'',comp1.Gnewxx+comp1.Gnewyy,setval(t,t))+withsol(''sol_update_0'',comp1.Gnew,setval(t,t))-Ggrad)'});
                else
                    disp('index is even, using sol_update_a for active torque')
                    model.physics('w8').feature('wfeq3').set('weak', {'((withsol(''sol_update_a'',comp1.Gnew,setval(t,t))-gradstep*Ggrad)-Gnew)*test(Gnew)';...
                        'test(Ggrad)*(withsol(''sol_forward'',comp1.Qxy,setval(t,t))*withsol(''sol_adjoint'',comp1.psixx,setval(t,t))-withsol(''sol_forward'',comp1.Qxx,setval(t,t))*withsol(''sol_adjoint'',comp1.psixy,setval(t,t))-Gamma_g*withsol(''sol_update_a'',comp1.Gnewxx+comp1.Gnewyy,setval(t,t))+withsol(''sol_update_a'',comp1.Gnew,setval(t,t))-Ggrad)'});
                end
            end

    case 'stress'

        model.study(['std_update_' namestr]).feature('time').set('activate', {'w' 'off' 'w2' 'off' 'w3' 'off' 'w4' 'off' 'w5' 'off'  'w6' 'off' 'w7' 'on' 'w8' 'off'});

        switch lower(namestr)

            case '0'
                %previous solution is guess A(t)
                disp('index is 0, using initial guess for active stress')
                model.study(['std_update_' namestr]).feature('time').set('disabledphysics', {'w7/wfeq2' 'w7/wfeq3'});
                %model.physics('w7').feature('wfeq1').set('weak', {'(controlcutoff(alpha0-gradstep*Agrad)-Anew)*test(Anew)';...
                %    'test(Agrad)*(Agrad-((A(t)-alpha0)-(withsol(''sol_forward_0'',comp1.Qxx,setval(t,t))*(withsol(''sol_adjoint_0'',comp1.nuxx,setval(t,t))-withsol(''sol_adjoint_0'',comp1.nuyy,setval(t,t)))+withsol(''sol_forward_0'',comp1.Qxy,setval(t,t))*(withsol(''sol_adjoint_0'',comp1.nuxy,setval(t,t))+withsol(''sol_adjoint_0'',comp1.nuyx,setval(t,t))))+0))'});
                model.physics('w7').feature('wfeq1').set('weak', {'(alpha0-gradstep*Agrad-Anew)*test(Anew)';...
                    'test(Agrad)*((A(t)-alpha0)-control_max_weight*(A(t)-control_max_soft)^(-2)*((A(t)-control_max_soft)^(-1)-(alpha0-control_max_soft)^(-1))^(control_max_power-1)-(withsol(''sol_forward_0'',comp1.Qxx,setval(t,t))*withsol(''sol_adjoint_0'',comp1.nuxx-comp1.nuyy,setval(t,t))+withsol(''sol_forward_0'',comp1.Qxy,setval(t,t))*withsol(''sol_adjoint_0'',comp1.nuxy+comp1.nuyx,setval(t,t)))-Agrad)'});

            case 'a'
                %previous control solution is sol_update_b
                model.study(['std_update_' namestr]).feature('time').set('disabledphysics', {'w7/wfeq1' 'w7/wfeq3'});

                disp('index is odd, using sol_update_b for active stress')
                model.physics('w7').feature('wfeq2').set('weak', {'(withsol(''sol_update_b'',comp1.Anew,setval(t,t))-gradstep*Agrad-Anew)*test(Anew)';...
                    'test(Agrad)*(Agrad-((withsol(''sol_update_b'',comp1.Anew,setval(t,t))-alpha0)-control_max_weight*(withsol(''sol_update_b'',comp1.Anew,setval(t,t))-control_max_soft)^(-2)*((withsol(''sol_update_b'',comp1.Anew,setval(t,t))-control_max_soft)^(-1)-(alpha0-control_max_soft)^(-1))^(control_max_power-1)-Gamma_alpha*withsol(''sol_update_b'',comp1.Anewxx+comp1.Anewyy,setval(t,t))-(withsol(''sol_forward'',comp1.Qxx,setval(t,t))*(withsol(''sol_adjoint'',comp1.nuxx,setval(t,t))-withsol(''sol_adjoint'',comp1.nuyy,setval(t,t)))+withsol(''sol_forward'',comp1.Qxy,setval(t,t))*(withsol(''sol_adjoint'',comp1.nuxy,setval(t,t))+withsol(''sol_adjoint'',comp1.nuyx,setval(t,t))))+0))'});

            case 'b'
                model.study(['std_update_' namestr]).feature('time').set('disabledphysics', {'w7/wfeq1' 'w7/wfeq2'});
                if ii==2 %&& params.cont==0
                    %previous control solution is sol_update_0
                    disp('index is 2, using sol_update_0 for active stress')

                    model.physics('w7').feature('wfeq3').set('weak', {'(withsol(''sol_update_0'',comp1.Anew,setval(t,t))-gradstep*Agrad-Anew)*test(Anew)';...
                        'test(Agrad)*(Agrad-((withsol(''sol_update_0'',comp1.Anew,setval(t,t))-alpha0)-control_max_weight*(withsol(''sol_update_0'',comp1.Anew,setval(t,t))-control_max_soft)^(-2)*((withsol(''sol_update_0'',comp1.Anew,setval(t,t))-control_max_soft)^(-1)-(alpha0-control_max_soft)^(-1))^(control_max_power-1)-Gamma_alpha*withsol(''sol_update_0'',comp1.Anewxx+comp1.Anewyy,setval(t,t))-(withsol(''sol_forward'',comp1.Qxx,setval(t,t))*(withsol(''sol_adjoint'',comp1.nuxx,setval(t,t))-withsol(''sol_adjoint'',comp1.nuyy,setval(t,t)))+withsol(''sol_forward'',comp1.Qxy,setval(t,t))*(withsol(''sol_adjoint'',comp1.nuxy,setval(t,t))+withsol(''sol_adjoint'',comp1.nuyx,setval(t,t))))+0))'});

                else
                    %previous control solution is sol_update_a
                    disp('index is even, using sol_update_a for active stress')
                    model.physics('w7').feature('wfeq3').set('weak', {'(withsol(''sol_update_a'',comp1.Anew,setval(t,t))-gradstep*Agrad-Anew)*test(Anew)';...
                        'test(Agrad)*(Agrad-((withsol(''sol_update_a'',comp1.Anew,setval(t,t))-alpha0)-control_max_weight*(withsol(''sol_update_a'',comp1.Anew,setval(t,t))-control_max_soft)^(-2)*((withsol(''sol_update_a'',comp1.Anew,setval(t,t))-control_max_soft)^(-1)-(alpha0-control_max_soft)^(-1))^(control_max_power-1)-Gamma_alpha*withsol(''sol_update_a'',comp1.Anewxx+comp1.Anewyy,setval(t,t))-(withsol(''sol_forward'',comp1.Qxx,setval(t,t))*(withsol(''sol_adjoint'',comp1.nuxx,setval(t,t))-withsol(''sol_adjoint'',comp1.nuyy,setval(t,t)))+withsol(''sol_forward'',comp1.Qxy,setval(t,t))*(withsol(''sol_adjoint'',comp1.nuxy,setval(t,t))+withsol(''sol_adjoint'',comp1.nuyx,setval(t,t))))+0))'});

                end

        end

end

switch lower(params.optimalphase)
    case 'true'
        if strcmp('0',namestr)==1
            dJdtheta_xint=mphint2(model,{['(-1)*C_Q_weight*((withsol(''sol_forward_0'',comp1.Qxy,setval(t,t))-withsol(''sol_isolate_'  params.targetstr ''',comp1.Qxy,setval(t,t-Tf+theta)))*withsol(''sol_isolate_'  params.targetstr ''',comp1.Qxyt,setval(t,t-Tf+theta))+(withsol(''sol_forward_0'',comp1.Qxx,setval(t,t))-withsol(''sol_isolate_'  params.targetstr ''',comp1.Qxx,setval(t,t-Tf+theta)))*withsol(''sol_isolate_'  params.targetstr ''',comp1.Qxxt,setval(t,t-Tf+theta)))']},'surface','dataset','dset_forward_0','t',params.tlist,'intorder',2);

            dJdtheta_xtint=trapz(params.tlist,dJdtheta_xint);
        else
            dJdtheta_xint=mphint2(model,{['(-1)*C_Q_weight*((withsol(''sol_forward'',comp1.Qxy,setval(t,t))-withsol(''sol_isolate_'  params.targetstr ''',comp1.Qxy,setval(t,t-Tf+theta)))*withsol(''sol_isolate_'  params.targetstr ''',comp1.Qxyt,setval(t,t-Tf+theta))+(withsol(''sol_forward'',comp1.Qxx,setval(t,t))-withsol(''sol_isolate_'  params.targetstr ''',comp1.Qxx,setval(t,t-Tf+theta)))*withsol(''sol_isolate_'  params.targetstr ''',comp1.Qxxt,setval(t,t-Tf+theta)))']},'surface','dataset','dset_forward','t',params.tlist,'intorder',2);

            dJdtheta_xtint=trapz(params.tlist,dJdtheta_xint);

        end
        disp('----------------------------------------------------------------')
        disp(['     plotting dJdtheta_xint, scale = ' num2str(max(dJdtheta_xint))])
        asciiplot(params.tlist,dJdtheta_xint);
        disp('----------------------------------------------------------------')
        model.param.set('dJdtheta', num2str(dJdtheta_xtint));
        disp(['     dJdtheta set'])
    case 'false'
end

model.sol.create(['sol_update_' namestr]);
model.sol(['sol_update_' namestr]).study(['std_update_' namestr]);
model.sol(['sol_update_' namestr]).attach(['std_update_' namestr]);
model.sol(['sol_update_' namestr]).create('st1', 'StudyStep');
model.sol(['sol_update_' namestr]).create('v1', 'Variables');
model.sol(['sol_update_' namestr]).create('t1', 'Time');
model.sol(['sol_update_' namestr]).feature('t1').create('fc1', 'FullyCoupled');
model.sol(['sol_update_' namestr]).feature('t1').feature.remove('fcDef');
model.sol(['sol_update_' namestr]).feature('t1').feature('dDef').set('linsolver', 'pardiso');

model.result.dataset.create(['dset_update_' namestr], 'Solution');
model.result.dataset(['dset_update_' namestr]).set('solution', ['sol_update_' namestr]);

disp(['  solver for updating alpha(x,t) configured...'])
disp(['     study name : ' 'std_update_' namestr])
disp(['     solution name : ' 'sol_update_' namestr])
disp(['     dataset name : ' 'dset_update_' namestr])

return
