function [z,Q_cost_terminal_error]=func_cost(ii,params)

%evaluate cost function for the current control (before updating):
% if ii==1          : alpha is alpha_0 (or w/e the initial guess was)
% if ii==2          : alpha comes from sol_upate_0
% if ii==4,6,7      : alpha comes from sol_update_b
% if ii==3,5,7...   : alpha comes from sol_update_a

global model

%pseudo code:
%1. calculate cost function:
%1.a Stage Control Cost
%1.b Stage State Cost
%1.c Terminal State Cost on Q
%1.d Terminal State Cost on Div Q dot e_theta

%E_thetaforce_weight
%A_Q_weight
%C_Q_weight


thetaforce_str='((comp1.Qxxx+comp1.Qxyy)*(-y)+(comp1.Qxyx-comp1.Qxxy)*(x))/sqrt(x^2+y^2+epsreg)';


switch lower(params.controltype)

case 'torque'
    disp(['        **using active torque g(t,x) in cost function**']);
    %% stage control costs
    if ii==1
        G_cost_int=mphint2(model,{'(0.5)*(G(t))^2'},'surface','dataset','dset_forward_0','t',params.tlist,'intorder',4);
        G_cost_tint=trapz(params.tlist,G_cost_int);
        Ggrad_cost_tint=0;
        %2.b
    elseif ii==2
        [G_cost_int,Ggrad_cost_int]=mphint2(model,{'(0.5)*(Gnew)^2','(0.5)*Gamma_g*(Gnewx^2+Gnewy^2)'},'surface','dataset','dset_update_0','t',params.tlist,'intorder',4);
        G_cost_tint=trapz(params.tlist,G_cost_int);
        Ggrad_cost_tint=trapz(params.tlist,Ggrad_cost_int);
    else
        if mod(ii,2)==1                     % odd ii
            [G_cost_int,Ggrad_cost_int]=mphint2(model,{'(0.5)*(Gnew)^2','(0.5)*Gamma_g*(Gnewx^2+Gnewy^2)'},'surface','dataset','dset_update_b','t',params.tlist,'intorder',4);
            G_cost_tint=trapz(params.tlist,G_cost_int);
            Ggrad_cost_tint=trapz(params.tlist,Ggrad_cost_int);
        elseif mod(ii,2)==0                % even ii
            [G_cost_int,Ggrad_cost_int]=mphint2(model,{'(0.5)*(Gnew)^2','(0.5)*Gamma_g*(Gnewx^2+Gnewy^2)'},'surface','dataset','dset_update_a','t',params.tlist,'intorder',4);
            G_cost_tint=trapz(params.tlist,G_cost_int);
            Ggrad_cost_tint=trapz(params.tlist,Ggrad_cost_int);
        end
    end
    actuation_cost_tint=G_cost_tint;
    actuationgrad_cost_tint=Ggrad_cost_tint;
case 'stress'
        disp(['        **using active stress alpha(t,x) in cost function**']);
%% stage control costs

if ii==1
    Alpha_cost_int=mphint2(model,{'(0.5)*(A(t)-alpha0)^2+control_max_weight*(1/control_max_power)*(1/(A(t)-control_max_soft)-1/(alpha0-control_max_soft))^control_max_power'},'surface','dataset','dset_forward_0','t',params.tlist,'intorder',4);
    Alpha_cost_tint=trapz(params.tlist,Alpha_cost_int);
    Alphagrad_cost_tint=0;
    %2.b
elseif ii==2
    [Alpha_cost_int,Alphagrad_cost_int]=mphint2(model,{'(0.5)*(Anew-alpha0)^2+control_max_weight*(1/control_max_power)*(1/(Anew-control_max_soft)-1/(alpha0-control_max_soft))^control_max_power','(0.5)*Gamma_alpha*(Anewx^2+Anewy^2)'},'surface','dataset','dset_update_0','t',params.tlist,'intorder',4);
    Alpha_cost_tint=trapz(params.tlist,Alpha_cost_int);
    Alphagrad_cost_tint=trapz(params.tlist,Alphagrad_cost_int);
else
    if mod(ii,2)==1                     % odd ii
        [Alpha_cost_int,Alphagrad_cost_int]=mphint2(model,{'(0.5)*(Anew-alpha0)^2+control_max_weight*(1/control_max_power)*(1/(Anew-control_max_soft)-1/(alpha0-control_max_soft))^control_max_power','(0.5)*Gamma_alpha*(Anewx^2+Anewy^2)'},'surface','dataset','dset_update_b','t',params.tlist,'intorder',4);
        Alpha_cost_tint=trapz(params.tlist,Alpha_cost_int);
        %Alpha_cost_tint=trapz(params.tlist,Alpha_cost_int);
        Alphagrad_cost_tint=trapz(params.tlist,Alphagrad_cost_int);
    elseif mod(ii,2)==0                 % even ii
        [Alpha_cost_int,Alphagrad_cost_int]=mphint2(model,{'(0.5)*(Anew-alpha0)^2+control_max_weight*(1/control_max_power)*(1/(Anew-control_max_soft)-1/(alpha0-control_max_soft))^control_max_power','(0.5)*Gamma_alpha*(Anewx^2+Anewy^2)'},'surface','dataset','dset_update_a','t',params.tlist,'intorder',4);
        Alpha_cost_tint=trapz(params.tlist,Alpha_cost_int);
        %Alpha_cost_tint=trapz(params.tlist,Alpha_cost_int);
        Alphagrad_cost_tint=trapz(params.tlist,Alphagrad_cost_int);
    end
end
actuation_cost_tint=Alpha_cost_tint;
actuationgrad_cost_tint=Alphagrad_cost_tint;
end


%% stage and terminal state costs
if ii==1
    Q_error_stage_int=mphint2(model,{['(0.5)*((withsol(''sol_forward_0'',comp1.Qxy,setval(t,t))-withsol(''sol_isolate_'  params.targetstr ''',comp1.Qxy,setval(t,t-Tf+theta)))^2+(withsol(''sol_forward_0'',comp1.Qxx,setval(t,t))-withsol(''sol_isolate_'  params.targetstr ''',comp1.Qxx,setval(t,t-Tf+theta)))^2)']},'surface','dataset','dset_forward_0','t',params.tlist,'intorder',4);
    Q_error_stage=trapz(params.tlist,Q_error_stage_int);
    Q_cost_stage=Q_error_stage*params.C_Q_weight;
    Q_cost_terminal_error=mphint2(model,{['(0.5)*((withsol(''sol_isolate_'  params.targetstr ''',comp1.Qxy,setval(t,theta))-withsol(''sol_forward_0'',comp1.Qxy,setval(t,theta)))^2+(withsol(''sol_isolate_'  params.targetstr ''',comp1.Qxx,setval(t,theta))-withsol(''sol_forward_0'',comp1.Qxx,setval(t,Tf)))^2)']},'surface','dataset','dset_forward_0','t',params.Tf,'intorder',4);
    Q_cost_terminal=Q_cost_terminal_error*params.A_Q_weight;
    Qgrad_cost_terminal=mphint2(model,{['(0.5)*radialcutoff(sqrt(x^2+y^2))*E_thetaforce_weight*(withsol(''sol_isolate_'  params.targetstr ''',' thetaforce_str ',setval(t,theta))-withsol(''sol_forward_0'',' thetaforce_str ',setval(t,Tf)))^2']},'surface','dataset','dset_forward_0','t',params.Tf,'intorder',4);
else
    Q_error_stage_int=mphint2(model,{['(0.5)*((withsol(''sol_forward'',comp1.Qxy,setval(t,t))-withsol(''sol_isolate_'  params.targetstr ''',comp1.Qxy,setval(t,t-Tf+theta)))^2+(withsol(''sol_forward'',comp1.Qxx,setval(t,t))-withsol(''sol_isolate_'  params.targetstr ''',comp1.Qxx,setval(t,t-Tf+theta)))^2)']},'surface','dataset','dset_forward','t',params.tlist,'intorder',4);
    Q_error_stage=trapz(params.tlist,Q_error_stage_int);
    Q_cost_stage=Q_error_stage*params.C_Q_weight;
    Q_cost_terminal_error=mphint2(model,{['(0.5)*((withsol(''sol_isolate_'  params.targetstr ''',comp1.Qxy,setval(t,theta))-withsol(''sol_forward'',comp1.Qxy,setval(t,theta)))^2+(withsol(''sol_isolate_'  params.targetstr ''',comp1.Qxx,setval(t,theta))-withsol(''sol_forward'',comp1.Qxx,setval(t,Tf)))^2)']},'surface','dataset','dset_forward','t',params.Tf,'intorder',4);
    Q_cost_terminal=Q_cost_terminal_error*params.A_Q_weight;
    Qgrad_cost_terminal=mphint2(model,{['(0.5)*radialcutoff(sqrt(x^2+y^2))*E_thetaforce_weight*(withsol(''sol_isolate_'  params.targetstr ''',' thetaforce_str ',setval(t,theta))-withsol(''sol_forward'',' thetaforce_str ',setval(t,Tf)))^2']},'surface','dataset','dset_forward','t',params.Tf,'intorder',4);
end

%% sum
z=actuationgrad_cost_tint+actuation_cost_tint+Q_cost_stage+Q_cost_terminal+Qgrad_cost_terminal;

disp(['            * actuation cost               : ' num2str(actuation_cost_tint)]);
disp(['            * actuation gradient cost      : ' num2str(actuationgrad_cost_tint)]);
disp(['            * Q stage error                : ' num2str(Q_error_stage)]);
disp(['            * Q stage cost                 : ' num2str(Q_cost_stage)]);
disp(['            * Q terminal error             : ' num2str(Q_cost_terminal_error)]);
disp(['            * Q terminal cost              : ' num2str(Q_cost_terminal)]);
disp(['            * chiral (gradQ terminal) cost : ' num2str(Qgrad_cost_terminal)]);
disp(['        cost total _______________________ : ' num2str(z)]);

return
