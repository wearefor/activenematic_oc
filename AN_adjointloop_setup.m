% XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
% Adjoint Looping Script for Active Nematic Control
% INITIALIZIATION
% (C) 2019-Present, Michael M. Norton
% Brandeis University, Physics
% Pennsylvania State University, Center for Neural Engineering
% XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
close all;
clear all;
clc;

compstr='XXXXXXXX';
filenamestr=datestr(now,'yyyymmdd_HHMM');
new=input(['new session? new (1), old (0): ']);

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
%% Start Comsol and Initiate Model
if new==1
    mphstart
else
end

import com.comsol.model.*
import com.comsol.model.util.*
%ModelUtil.showProgress(true);

model = ModelUtil.create('Model8');
model.modelPath(pathstr);
model.label(['LimitCycles_' filenamestr '.mph']);
model.comments(['Confined Active Nematic']);
%% Define Parameters

model.param.set('lam1', '1');   %flow alignment
model.param.set('rho0', '1.6'); %nematic density
model.param.set('S0', 'sqrt(2*(rho0-1)/(rho0+1))');
model.param.set('nx0', 'sqrt(1/2)');
model.param.set('Smax', 'sqrt(2)');
model.param.set('DomRad', '6.5', 'domain radius');
model.param.set('Ea', '3', 'anchoring energy');
model.param.set('pf', '0');
model.param.set('C', 'rho0');
model.param.set('cubic_A', '0.0445'); %initial condition for Q parameter
model.param.set('cubic_B', '0.765');  %initial condition for Q parameter
model.param.set('beta1', 'C-1'); %isotropic-nematic transition parameters
model.param.set('beta2', '(C+1)/C^2'); %isotropic-nematic transition parameters
model.param.set('rey', '0');
model.param.set('gradstep', '0');
model.param.set('A_Q_weight', '0');
model.param.set('B_U_weight', '0');
model.param.set('C_Q_weight', '0');
model.param.set('D_U_weight', '0');
model.param.set('E_thetaforce_weight', '0');
model.param.set('epsreg', '1e-8');
model.param.set('Gamma_alpha', '0.1');
model.param.set('Gamma_g', '0.1');
model.param.set('alpha0', '5');
model.param.set('G0', '-0.5');

%% Define Functions, Geometry, etc..

model.modelNode.create('comp1');
model.geom.create('geom1', 2);

model.func.create('rn2', 'Random');
model.func.create('an2', 'Analytic');
model.func.create('an6', 'Analytic');
model.func.create('an7', 'Analytic');
model.func.create('an8', 'Analytic');
model.func.create('an9', 'Analytic');
model.func.create('an10', 'Analytic');
model.func.create('an11', 'Analytic');
model.func.create('an12', 'Analytic');
model.func.create('pw1', 'Piecewise');
model.func.create('pw2', 'Piecewise');
model.func('rn2').label('mean0');
model.func('rn2').set('uniformrange', '0.05');
model.func('rn2').set('nargs', '2');
model.func('an6').set('args', {'x' 'y'});
model.func('an6').set('expr', 'rho0+1.75*rn2(x,y)');
model.func('an6').set('plotargs', {});
model.func('an7').set('args', {'x' 'y'});
model.func('an7').set('expr', '1/sqrt(1+(3*cubic_A*x^2-cubic_B)^2)');
model.func('an7').set('plotargs', {'x' '-1' '1'; 'y' '-1' '1'});
model.func('an7').set('funcname', 'nx00');
model.func('an8').set('args', {'x' 'y'});
model.func('an8').set('expr', '(3*cubic_A*x^2-cubic_B)/sqrt(1+(3*cubic_A*x^2-cubic_B)^2)');
model.func('an8').set('plotargs', {'x' '-1' '1'; 'y' '-1' '1'});
model.func('an8').set('funcname', 'ny00');
model.func('an9').set('args', {'x' 'y'});
model.func('an9').set('expr', 'rho0+rn2(x,y)');
model.func('an9').set('plotargs', {'x' '-1' '1'; 'y' '-1' '1'});
model.func('an9').set('funcname', 'C0');
model.func('an10').set('args', {'x' 'y'});
model.func('an10').set('expr', 'S0*C0(x,y)*(nx00(x,y)^2-1/2)');
model.func('an10').set('plotargs', {'x' '0' '1'; 'y' '0' '1'});
model.func('an10').set('funcname', 'QxxIC');
model.func('an11').set('args', {'x' 'y'});
model.func('an11').set('expr', 'S0*C0(x,y)*(nx00(x,y)*ny00(x,y))');
model.func('an11').set('plotargs', {'x' '0' '1'; 'y' '0' '1'});
model.func('an11').set('funcname', 'QxyIC');

%model.func('an12').set('args', {'t'});
%model.func('an12').set('expr', 'tanh((t-2)*1.25)/2+1/2');
%model.func('an12').set('plotargs', {'x' '0' 'Tf'});
%model.func('an12').set('funcname', 'rampforcing');

model.func('pw1').set('pieces', {'0' '5' 'alpha0*(1-x/5)'; '5' '10' 'alpha0*((x-5)/5)'});
model.func('pw1').set('smoothzone', '0.4');
model.func('pw1').set('smooth', 'contd2');
model.func('pw1').set('funcname', 'A');

model.func('pw2').set('pieces', {'0' '20' 'G0'});
model.func('pw2').set('smoothzone', '0.4');
model.func('pw2').set('smooth', 'contd2');
model.func('pw2').set('funcname', 'G');

model.mesh.create('mesh1', 'geom1');

model.geom('geom1').create('c1', 'Circle');
model.geom('geom1').feature('c1').set('r', 'DomRad');
model.geom('geom1').create('sq1', 'Square');
model.geom('geom1').feature('sq1').active(false);
model.geom('geom1').feature('sq1').set('size', 'DomRad');
model.geom('geom1').create('pt1', 'Point');
model.geom('geom1').run;

% XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
% XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
%% Define Spatial Integration Function (used for cost function evaluation)

model.cpl.create('intop1', 'Integration', 'geom1');
model.cpl('intop1').selection.all;
model.cpl('intop1').label('spaceint');
model.cpl('intop1').set('opname', 'func_spaceint');

% XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
% XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
%% Define Physics
model.physics.create('w', 'WeakFormPDE', 'geom1');
model.physics('w').field('dimensionless').component({'Qxx' 'Qxy'});
model.physics('w').create('wfeq15', 'WeakFormPDE', 2);
model.physics('w').feature('wfeq15').selection.set([1]);
model.physics('w').create('wfeq7', 'WeakFormPDE', 2);
model.physics('w').feature('wfeq7').selection.set([1]);
model.physics('w').create('wfeq8', 'WeakFormPDE', 2);
model.physics('w').feature('wfeq8').selection.set([1]);
model.physics('w').create('wfeq2', 'WeakFormPDE', 2);
model.physics('w').feature('wfeq2').selection.set([1]);
model.physics('w').create('wfeq12', 'WeakFormPDE', 2);
model.physics('w').feature('wfeq12').selection.all;
model.physics('w').create('wfeq3', 'WeakFormPDE', 2);
model.physics('w').feature('wfeq3').selection.set([1]);
model.physics('w').create('wfeq4', 'WeakFormPDE', 2);
model.physics('w').feature('wfeq4').selection.set([1]);
model.physics('w').create('wfeq5', 'WeakFormPDE', 2);
model.physics('w').feature('wfeq5').selection.set([1]);
model.physics('w').create('wfeq6', 'WeakFormPDE', 2);
model.physics('w').feature('wfeq6').selection.set([1]);
model.physics('w').create('dir2', 'DirichletBoundary', 1);
model.physics('w').feature('dir2').selection.set([1 2 3 4]);
model.physics('w').create('weak1', 'WeakContribution', 1);
model.physics('w').feature('weak1').selection.set([1 2 3 4]);
model.physics('w').create('weak2', 'WeakContribution', 1);
model.physics('w').feature('weak2').selection.set([1 2 3 4]);
model.physics('w').create('wfeq16', 'WeakFormPDE', 2);
model.physics('w').feature('wfeq16').selection.set([1]);

model.physics.create('w2', 'WeakFormPDE', 'geom1');
model.physics('w2').field('dimensionless').component({'ux' 'uy'});
model.physics('w2').create('wfeq3', 'WeakFormPDE', 2);
model.physics('w2').feature('wfeq3').selection.set([1]);
model.physics('w2').create('dir2', 'DirichletBoundary', 1);
model.physics('w2').feature('dir2').selection.set([1 2 3 4]);
model.physics('w2').create('wfeq2', 'WeakFormPDE', 2);
model.physics('w2').feature('wfeq2').selection.set([1]);
%model.physics('w2').create('wfeq5', 'WeakFormPDE', 2);
%model.physics('w2').feature('wfeq5').selection.set([1]);
model.physics('w2').create('wfeq4', 'WeakFormPDE', 2);
model.physics('w2').feature('wfeq4').selection.set([1]);
%model.physics('w2').create('wfeq7', 'WeakFormPDE', 2);
%model.physics('w2').feature('wfeq7').selection.set([1]);
model.physics('w2').create('wfeq6', 'WeakFormPDE', 2);
model.physics('w2').feature('wfeq6').selection.set([1]);
model.physics.create('w3', 'WeakFormPDE', 'geom1');
model.physics('w3').field('dimensionless').field('p');
model.physics('w3').field('dimensionless').component({'p'});
model.physics('w3').create('constr1', 'PointwiseConstraint', 0);
model.physics('w3').feature('constr1').selection.set([3]);

model.physics.create('w4', 'WeakFormPDE', 'geom1');
model.physics('w4').field('dimensionless').component({'psixx' 'psixy'});
model.physics('w4').create('dir1', 'DirichletBoundary', 1);
model.physics('w4').feature('dir1').selection.set([1 2 3 4]);
model.physics('w4').create('wfeq2', 'WeakFormPDE', 2);
model.physics('w4').feature('wfeq2').selection.set([1]);
%model.physics('w4').create('wfeq3', 'WeakFormPDE', 2);
%model.physics('w4').feature('wfeq3').selection.set([1]);
%model.physics('w4').create('wfeq5', 'WeakFormPDE', 2);
%model.physics('w4').feature('wfeq5').selection.set([1]);
model.physics('w4').create('wfeq4', 'WeakFormPDE', 2);
model.physics('w4').feature('wfeq4').selection.set([1]);
model.physics('w4').create('wfeq6', 'WeakFormPDE', 2);
model.physics('w4').feature('wfeq6').selection.set([1]);
model.physics('w4').create('wfeq7', 'WeakFormPDE', 2);
model.physics('w4').feature('wfeq7').selection.set([1]);


model.physics.create('w5', 'WeakFormPDE', 'geom1');
model.physics('w5').field('dimensionless').component({'nux' 'nuy'});
model.physics('w5').create('dir1', 'DirichletBoundary', 1);
model.physics('w5').feature('dir1').selection.set([1 2 3 4]);

model.physics('w5').create('wfeq2', 'WeakFormPDE', 2);
model.physics('w5').feature('wfeq2').selection.set([1]);

model.physics.create('w6', 'WeakFormPDE', 'geom1');
model.physics('w6').field('dimensionless').component({'phi'});
model.physics('w6').create('constr2', 'PointwiseConstraint', 0);
model.physics('w6').feature('constr2').selection.set([3]);

%create active stress update PDE
model.physics.create('w7', 'WeakFormPDE', 'geom1');
model.physics('w7').field('dimensionless').component({'Anew' 'Agrad'});
model.physics('w7').create('wfeq2', 'WeakFormPDE', 2);
model.physics('w7').feature('wfeq2').selection.set([1]);
model.physics('w7').create('wfeq3', 'WeakFormPDE', 2);
model.physics('w7').feature('wfeq3').selection.set([1]);

%create active torque update PDE
model.physics.create('w8', 'WeakFormPDE', 'geom1');
model.physics('w8').field('dimensionless').component({'Gnew' 'Ggrad'});
model.physics('w8').create('wfeq2', 'WeakFormPDE', 2);
model.physics('w8').feature('wfeq2').selection.all;
model.physics('w8').create('wfeq3', 'WeakFormPDE', 2);
model.physics('w8').feature('wfeq3').selection.all;

%% Define Mesh
model.mesh('mesh1').create('map1', 'Map');
model.mesh('mesh1').create('ftri1', 'FreeTri');
model.mesh('mesh1').feature('map1').selection.geom('geom1', 2);
model.mesh('mesh1').feature('map1').selection.set([1]);
model.mesh('mesh1').feature('ftri1').selection.geom('geom1', 2);
model.mesh('mesh1').feature('ftri1').selection.set([1]);
%% Define Q-Field Dynamics
model.physics('w').label('Weak Form PDE : Q and density');
model.physics('w').prop('ShapeProperty').set('valueType', 'real');
model.physics('w').prop('Units').set('CustomSourceTermUnit', '1');
model.physics('w').feature('wfeq1').set('weak', {'test(Qxx)*(Qxxt)'; 'test(Qxy)*(Qxyt)'});
model.physics('w').feature('wfeq1').label('Time Derivatives');
model.physics('w').feature('init1').set('Qxx', 'S0*C*(nx00(x,y)^2-1/2)');
model.physics('w').feature('init1').set('Qxy', 'S0*C*(nx00(x,y)*ny00(x,y))');
model.physics('w').feature('init1').label('Initial Conditions');
model.physics('w').feature('wfeq15').set('weak', {'-Qxx*test(Qxx)*(beta1-beta2*(2*(Qxx^2)+2*(Qxy^2)))'; '-Qxy*test(Qxy)*(beta1-beta2*(2*(Qxx^2)+2*(Qxy^2)))'});
model.physics('w').feature('wfeq15').label('Propto DR 1');
model.physics('w').feature('wfeq7').set('weak', {'2*(test(Qxxx)*Qxxx+test(Qxxy)*Qxxy)'; '2*(test(Qxyx)*Qxyx+test(Qxyy)*Qxyy)'});
model.physics('w').feature('wfeq7').label('Propto DE');
model.physics('w').feature('wfeq8').set('weak', {'0'; '0'});
model.physics('w').feature('wfeq8').label('Propto D');
model.physics('w').feature('wfeq2').set('weak', {'0'; '0'});
model.physics('w').feature('wfeq2').label('Propto DQ');
model.physics('w').feature('wfeq12').set('weak', {'0'; '0'});
model.physics('w').feature('wfeq12').label('Propto DQ 2');

model.physics('w').feature('wfeq3').set('weak', {'(ux*Qxxx+uy*Qxxy+(uxx+uyy)*Qxx)*test(Qxx)'; '(ux*Qxyx+uy*Qxyy+(uxx+uyy)*Qxy)*test(Qxy)'});
model.physics('w').feature('wfeq3').label('Propto LamC');
model.physics('w').feature('wfeq4').set('weak', {'test(Qxx)*Qxy*(uyx-uxy)'; 'test(Qxy)*Qxx*(uxy-uyx)'});
model.physics('w').feature('wfeq4').label('Propto LamR');
model.physics('w').feature('wfeq5').set('weak', {'-test(Qxx)*lam1*uxx'; '-test(Qxy)*lam1*(uxy+uyx)/2'});
model.physics('w').feature('wfeq5').label('Propto Lam1');
model.physics('w').feature('wfeq6').set('weak', {'0'; '0'});
model.physics('w').feature('wfeq6').label('Propto Lam2');

model.physics('w').feature('wfeq16').set('weak', {'test(Qxx)*Qxy*(G(t))'; 'test(Qxy)*Qxx*(-G(t))'});
model.physics('w').feature('wfeq16').label('Active Torque');

model.physics('w').feature('dir2').set('r', {'((tx^2-1/2))*Smax*C'; '((tx*ty))*Smax*C'});
model.physics('w').feature('weak1').set('weakExpression', '(Qxx-((tx^2-1/2))*Smax*C)*Ea*test(Qxx)');
model.physics('w').feature('weak1').active(false);
model.physics('w').feature('weak1').label('anchoring energy 1');
model.physics('w').feature('weak2').set('weakExpression', '(Qxy-((tx*ty))*Smax*C)*Ea*test(Qxy)');
model.physics('w').feature('weak2').active(false);
model.physics('w').feature('weak2').label('anchoring energy 2');
%% Define u-field Dynamics
model.physics('w2').label('Weak Form PDE : Hydrodynamics');
model.physics('w2').prop('ShapeProperty').set('order', '3');
model.physics('w2').feature('wfeq1').set('weak', {'(test(uxx)*uxx+test(uxy)*uxy)+px*test(ux)'; '(test(uyx)*uyx+test(uyy)*uyy)+py*test(uy)'});
%model.physics('w2').feature('wfeq3').set('weak', {'(f*C*ux)*test(ux)'; '(f*C*uy)*test(uy)'});
%model.physics('w2').feature('wfeq3').label('Bulk Drag');
% Define Active Stress Fields
% XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
% XXXXXXXXXXXXXXXXXXX DEFINE FORWARD ACTIVE STRESS FIELDS XXXXXXXXXXXXXXXXXXXXXXX
% XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

% w2/wfeq2: uniform (use for generating initial conditions and target solution)
% w2/wfeq5: ramp up ramp down (initial guess)
% w2/wfeq4: use a previous solution A
% w2/wfeq7: use a previous solution B

model.physics('w2').feature('wfeq2').set('weak', {'alpha0*(Qxxx+Qxyy)*test(ux)'; 'alpha0*(Qxyx-Qxxy)*test(uy)'});
model.physics('w2').feature('wfeq2').label('Active Stress - Uniform');

%model.physics('w2').feature('wfeq5').set('weak', {'(A(t)*(Qxxx+Qxyy))*test(ux)'; '(A(t)*(Qxyx-Qxxy))*test(uy)'});
%model.physics('w2').feature('wfeq5').label('Active Stress - Time Varying - Initial Guess - No Gradients');

model.physics('w2').feature('wfeq4').set('weak', {'(withsol(''sol7'',comp1.Anew_a,setval(t,t))*(Qxxx+Qxyy)+(Qxx*d(withsol(''sol7'',comp1.Anew_a,setval(t,t)),x)+Qxy*d(withsol(''sol7'',comp1.Anew_a,setval(t,t)),y)))*test(ux)'; '(withsol(''sol7'',comp1.Anew_a,setval(t,t))*(Qxyx-Qxxy)+(Qxy*d(withsol(''sol7'',comp1.Anew_a,setval(t,t)),x)-Qxx*d(withsol(''sol7'',comp1.Anew_a,setval(t,t)),y)))*test(uy)'});
model.physics('w2').feature('wfeq4').label('Active Stress - Time Varying');

%model.physics('w2').feature('wfeq7').set('weak', {'(withsol(''sol7'',comp1.Anew_b,setval(t,t))*(Qxxx+Qxyy)+(Qxx*d(withsol(''sol7'',comp1.Anew_b,setval(t,t)),x)+Qxy*d(withsol(''sol7'',comp1.Anew_b,setval(t,t)),y)))*test(ux)'; '(withsol(''sol7'',comp1.Anew_b,setval(t,t))*(Qxyx-Qxxy)+(Qxy*d(withsol(''sol7'',comp1.Anew_b,setval(t,t)),x)-Qxx*d(withsol(''sol7'',comp1.Anew_b,setval(t,t)),y)))*test(uy)'});
%model.physics('w2').feature('wfeq7').label('Active Stress - Time Varying - b');

% XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
% XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

model.physics('w2').feature('wfeq6').label('time derivative');
model.physics('w2').feature('wfeq6').set('weak', {'test(ux)*uxt*rey'; 'test(uy)*uyt*rey'});
model.physics('w3').label('Weak Form PDE : Continuity');
model.physics('w3').feature('wfeq1').set('weak', 'test(p)*(uxx+uyy+pf*p)');
model.physics('w3').feature('constr1').set('constraintExpression', 'p');
model.physics('w3').feature('constr1').set('constraintMethod', 'nodal');
%% Define Adjoint Dynamics


model.physics('w4').label('Weak Form PDE: Adjoint Dynamics: psi');
model.physics('w4').prop('ShapeProperty').set('order', '3');
model.physics('w4').feature('wfeq1').set('weak', {'test(psixx)*(-(psixxt+ux*psixxx+uy*psixxy))+test(psixx)*(psixy*(uxy-uyx))+test(psixx)*(4*Qxx*beta2*(Qxx*psixx+Qxy*psixy)-psixx*(beta1-2*beta2*(Qxx^2+Qxy^2)))+2*(test(psixxx)*psixxx+test(psixxy)*psixxy)';...
                                                  'test(psixy)*(-(psixyt+ux*psixyx+uy*psixyy))+test(psixy)*(psixx*(uyx-uxy))+test(psixy)*(4*Qxy*beta2*(Qxx*psixx+Qxy*psixy)-psixy*(beta1-2*beta2*(Qxx^2+Qxy^2)))+2*(test(psixyx)*psixyx+test(psixyy)*psixyy)'});


% XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
% XXXXXXXXXXXXXXXXXX DEFINE BACKWARD ACTIVE STRESS FIELDS XXXXXXXXXXXXXXXXXXXXXXX
% XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

% w4/feq2 : Use initial ramp A(t)
% w4/feq3 : Uses Anew_a
% w4/feq5 : Uses Anew_b
% w4/feq4 : target solution selection

model.physics('w4').feature('wfeq7').set('weak', {'(-1)*test(psixx)*psixy*G(t)'; 'test(psixy)*psixx*G(t)'});
model.physics('w4').feature('wfeq7').label('activetorque');

model.physics('w4').feature('wfeq6').set('weak', {'test(psixx)*(alpha0*(nuyy-nuxx))'; 'test(psixy)*(-alpha0*(nuxy+nuyx))'});
model.physics('w4').feature('wfeq6').label('activeterm_uniform');

model.physics('w4').feature('wfeq2').set('weak', {'test(psixx)*(A(t)*(nuyy-nuxx))';...
                                                  'test(psixy)*(-A(t)*(nuxy+nuyx))'});
model.physics('w4').feature('wfeq2').label('activeterm_timedependent');

%model.physics('w4').feature('wfeq3').set('weak', {'test(psixx)*(withsol(''sol7'',comp1.Anew_a,setval(t,t))*(nuyy-nuxx))';...
%                                                  'test(psixy)*(-withsol(''sol7'',comp1.Anew_a,setval(t,t))*(nuxy+nuyx))'});
%model.physics('w4').feature('wfeq3').label('activeterm_iteration_a');

%model.physics('w4').feature('wfeq5').set('weak', {'test(psixx)*(withsol(''sol7'',comp1.Anew_b,setval(t,t))*(nuyy-nuxx))';...
%                                                  'test(psixy)*(-withsol(''sol7'',comp1.Anew_b,setval(t,t))*(nuxy+nuyx))'});
%model.physics('w4').feature('wfeq5').label('activeterm_iteration_b');

% XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
% XXXXXXXXXXXXXXXXXXXXXXXXXXXXX CONTROL PENALTY XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
% XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

%model.physics('w4').feature('wfeq4').set('weak', {'Q0weight*test(psixx)*(Qxx-withsol(''sol_isolate_CW'',comp1.Qxx,setval(t,t)))'; 'Q0weight*test(psixy)*(Qxy-withsol(''sol_isolate_CW'',comp1.Qxy,setval(t,t)))'});
model.physics('w4').feature('wfeq4').set('weak', {'C_Q_weight*test(psixx)*(Qxx-withsol(''sol_isolate_CW'',comp1.Qxx,setval(t,t-Tf+theta)))';...
                                                  'C_Q_weight*test(psixy)*(Qxy-withsol(''sol_isolate_CW'',comp1.Qxy,setval(t,t-Tf+theta)))'});
model.physics('w4').feature('wfeq4').label('target penalty');

% XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
% XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

model.physics('w5').label('Weak Form PDE: Adjoint Dynamics: nu');
model.physics('w5').prop('ShapeProperty').set('order', '3');
%model.physics('w5').feature('wfeq1').set('weak', {'-(test(nuxx)*nuxx+test(nuxy)*nuxy)+test(nux)*(-phix)+test(nux)*((psixx*Qxxx+psixy*Qxyx)+(psixy*Qxxy+Qxx*psixyy-psixx*Qxyy-Qxy*psixxy)+lam1*(psixxx+0.5*psixyy))+test(nux)*nuxt*rey'; '-(test(nuyx)*nuyx+test(nuyy)*nuyy)+test(nuy)*(-phiy)+test(nuy)*((psixx*Qxxy+psixy*Qxyy)+(psixx*Qxyx+Qxy*psixxx-psixy*Qxxx-Qxx*psixyx)+lam1*(0.5*psixyx))+test(nuy)*nuyt*rey'});

model.physics('w5').feature('wfeq1').set('weak', {'-test(nux)*nuxt*rey+(test(nuxx)*nuxx+test(nuxy)*nuxy)+test(nux)*(-phix)+test(nux)*((psixx*Qxxx+psixy*Qxyx)+(-psixy*Qxxy-Qxx*psixyy+psixx*Qxyy+Qxy*psixxy)+lam1*(psixxx+0.5*psixyy))';...
                                                  '-test(nuy)*nuyt*rey+(test(nuyx)*nuyx+test(nuyy)*nuyy)+test(nuy)*(-phiy)+test(nuy)*((psixx*Qxxy+psixy*Qxyy)+(-psixx*Qxyx-Qxy*psixxx+psixy*Qxxx+Qxx*psixyx)+lam1*(0.5*psixyx))'});


%model.physics('w5').feature('wfeq2').set('weak', {'U0weight*test(nux)*(ux-withsol(''sol_isolate_CW'',comp1.ux,setval(t,t)))'; 'U0weight*test(nuy)*(uy-withsol(''sol_isolate_CW'',comp1.uy,setval(t,t)))'});
model.physics('w5').feature('wfeq2').set('weak', {'D_U_weight*test(nux)*(ux-withsol(''sol_isolate_CW'',comp1.ux,setval(t,t-Tf+theta)))';...
                                                  'D_U_weight*test(nuy)*(uy-withsol(''sol_isolate_CW'',comp1.uy,setval(t,t-Tf+theta)))'});
model.physics('w5').feature('wfeq2').label('target penalty velocity');

model.physics('w6').label('Weak Form PDE: Adjoint Dynamics phi');
model.physics('w6').feature('wfeq1').set('weak', 'test(phi)*(nuxx+nuyy)');
model.physics('w6').feature('constr2').set('constraintExpression', 'phi');
model.physics('w6').feature('constr2').set('constraintMethod', 'nodal');
%% Adjoint Initial Conditions
% XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
% XXXXXXXXXXXXXXXXXXXXXXXXX ADJOINT INITIAL CONDITIONS XXXXXXXXXXXXXXXXXXXXXXXXXX
% XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
%model.physics('w4').feature('init1').set('psixx', 'A_Q_weight*(withsol(''sol_forward_0'',comp1.Qxx,setval(t,Tf))-withsol(''sol_isolate_CW'',comp1.Qxx,setval(t,10)))');
%model.physics('w4').feature('init1').set('psixy', 'A_Q_weight*(withsol(''sol_forward_0'',comp1.Qxy,setval(t,Tf))-withsol(''sol_isolate_CW'',comp1.Qxy,setval(t,10)))');
%model.physics('w5').feature('init1').set('nux', 'B_U_weight*(withsol(''sol_forward_0'',comp1.ux,setval(t,Tf))-withsol(''sol_isolate_CW'',comp1.ux,setval(t,10)))');
%model.physics('w5').feature('init1').set('nuy', 'B_U_weight*(withsol(''sol_forward_0'',comp1.uy,setval(t,Tf))-withsol(''sol_isolate_CW'',comp1.uy,setval(t,10)))');

model.physics('w4').feature('init1').set('psixx', '0');
model.physics('w4').feature('init1').set('psixy', '0');
model.physics('w5').feature('init1').set('nux', '0');
model.physics('w5').feature('init1').set('nuy', '0');

% XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
% XXXXXXXXXXXXXXXXXXXXXXXX DEFINE ACTIVE STRESS UPDATE XXXXXXXXXXXXXXXXXXXXXXXXXX
% XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

model.physics('w7').label('Update Active Stress');
model.physics('w7').feature('wfeq1').set('weak', {'(A(t)-gradstep*Agrad-Anew)*test(Anew)'; 'test(Agrad)*(Agrad-((A(t)-alpha0)-(withsol(''sol_forward_0'',comp1.Qxx,setval(t,t))*(withsol(''sol_adjoint_0'',comp1.nuxx,setval(t,t))-withsol(''sol_adjoint_0'',comp1.nuyy,setval(t,t)))+withsol(''sol_forward_0'',comp1.Qxy,setval(t,t))*(withsol(''sol_adjoint_0'',comp1.nuxy,setval(t,t))+withsol(''sol_adjoint_0'',comp1.nuyx,setval(t,t))))+0))'});
model.physics('w7').feature('wfeq1').label('Update 0');

model.physics('w7').feature('wfeq2').set('weak', {'(withsol(''sol_update_b'',comp1.Anew,setval(t,t))-gradstep*Agrad-Anew)*test(Anew)'; 'test(Agrad)*(Agrad-((withsol(''sol_update_b'',comp1.Anew,setval(t,t))-alpha0)-(withsol(''sol_forward'',comp1.Qxx,setval(t,t))*(withsol(''sol_adjoint'',comp1.nuxx,setval(t,t))-withsol(''sol_adjoint'',comp1.nuyy,setval(t,t)))+withsol(''sol_forward'',comp1.Qxy,setval(t,t))*(withsol(''sol_adjoint'',comp1.nuxy,setval(t,t))+withsol(''sol_adjoint'',comp1.nuyx,setval(t,t))))+0))'});
model.physics('w7').feature('wfeq2').label('Update A');

model.physics('w7').feature('wfeq3').set('weak', {'(withsol(''sol_update_a'',comp1.Anew,setval(t,t))-gradstep*Agrad-Anew)*test(Anew)'; 'test(Agrad)*(Agrad-((withsol(''sol_update_a'',comp1.Anew,setval(t,t))-alpha0)+withsol(''sol_forward'',comp1.Qxx,setval(t,t))*(withsol(''sol_adjoint'',comp1.nuxx,setval(t,t))-withsol(''sol_adjoint'',comp1.nuyy,setval(t,t)))+withsol(''sol_forward'',comp1.Qxy,setval(t,t))*(withsol(''sol_adjoint'',comp1.nuxy,setval(t,t))+withsol(''sol_adjoint'',comp1.nuyx,setval(t,t)))))'});
model.physics('w7').feature('wfeq3').label('Update B');

% XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
% XXXXXXXXXXXXXXXXXXXXXXXX DEFINE ACTIVE TORQUE UPDATE XXXXXXXXXXXXXXXXXXXXXXXXXX
% XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

model.physics('w8').label('Update Active Torque');
model.physics('w8').feature('wfeq1').set('weak', {'(G(t)-gradstep*Ggrad-Gnew)*test(Gnew)'; 'test(Ggrad)*(Ggrad-(G(t)-0))'});
model.physics('w8').feature('wfeq1').label('Update_G_0');

model.physics('w8').feature('wfeq2').set('weak', {'(G(t)-gradstep*Ggrad-Gnew)*test(Gnew)'; 'test(Ggrad)*(Ggrad-(G(t)-0))'});
model.physics('w8').feature('wfeq2').label('Update_G_A');

model.physics('w8').feature('wfeq3').set('weak', {'(G(t)-gradstep*Ggrad-Gnew)*test(Gnew)'; 'test(Ggrad)*(Ggrad-(G(t)-0))'});
model.physics('w8').feature('wfeq3').label('Update_G_B');

%
%% Build Mesh
% XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
% XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

model.mesh('mesh1').feature('size').set('hauto', 3);
model.mesh('mesh1').feature('size').set('custom', 'on');
model.mesh('mesh1').feature('size').set('hmax', '0.25');
model.mesh('mesh1').feature('size').set('hmin', '2.5E-4');
model.mesh('mesh1').feature('map1').active(false);
model.mesh('mesh1').run;
%% Create Studies and Solutions
% XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
% XXXXXXXXXXXXXXXXXXXXXXXXXXXX CREATE  STUDIES XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
% XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

model.study.create('std_initialize_CW');
model.study('std_initialize_CW').create('time', 'Transient');
model.study('std_initialize_CW').feature('time').set('activate', {'w' 'on' 'w2' 'on' 'w3' 'on' 'w4' 'off' 'w5' 'off' 'w6' 'off' 'w7' 'off' 'w8' 'off'});
model.study('std_initialize_CW').label('Step 1: Initialization (find steady state circulation): CW');
model.study('std_initialize_CW').feature('time').set('rtol', '1e-8');
model.study('std_initialize_CW').feature('time').set('useadvanceddisable', true);
model.study('std_initialize_CW').feature('time').set('tlist', 'range(0,0.05,25)');
model.study('std_initialize_CW').feature('time').set('disabledphysics', {'w2/wfeq4' 'w/wfeq16'});
%model.study('std_initialize_CW').feature('time').set('disabledvariables', {'var_Anew_0' 'var_Anew_a' 'var_Anew_b'});
model.study('std_initialize_CW').feature('time').set('rtolactive', true);

model.study.create('std_initialize_CCW');
model.study('std_initialize_CCW').create('time', 'Transient');
model.study('std_initialize_CCW').feature('time').set('activate', {'w' 'on' 'w2' 'on' 'w3' 'on' 'w4' 'off' 'w5' 'off' 'w6' 'off' 'w7' 'off' 'w8' 'off'});
model.study('std_initialize_CCW').label('Step 1: Initialization (find steady state circulation): CCW');
model.study('std_initialize_CCW').feature('time').set('rtol', '1e-8');
model.study('std_initialize_CCW').feature('time').set('useadvanceddisable', true);
model.study('std_initialize_CCW').feature('time').set('tlist', 'range(0,0.05,25)');
model.study('std_initialize_CCW').feature('time').set('disabledphysics', {'w2/wfeq4' 'w/wfeq16'});
%model.study('std_initialize_CCW').feature('time').set('disabledvariables', {'var_Anew_0' 'var_Anew_a' 'var_Anew_b'});
model.study('std_initialize_CCW').feature('time').set('rtolactive', true);

model.study.create('std_isolate_CW');
model.study('std_isolate_CW').create('time', 'Transient');
model.study('std_isolate_CW').feature('time').set('activate', {'w' 'on' 'w2' 'on' 'w3' 'on' 'w4' 'off' 'w5' 'off'  'w6' 'off'  'w7' 'off' 'w8' 'off'});
model.study('std_isolate_CW').label('Step 2: Isolate Limit Cycle: CW');
model.study('std_isolate_CW').feature('time').set('solnum', 'interp');
model.study('std_isolate_CW').feature('time').set('useinitsol', 'on');
model.study('std_isolate_CW').feature('time').set('rtol', '1e-8');
model.study('std_isolate_CW').feature('time').set('useadvanceddisable', true);
model.study('std_isolate_CW').feature('time').set('tlist', 'range(0,0.05,15)');
model.study('std_isolate_CW').feature('time').set('t', '25');
model.study('std_isolate_CW').feature('time').set('disabledphysics', {'w2/wfeq4' 'w/wfeq16'});
%model.study('std_isolate_CW').feature('time').set('disabledvariables', {'var_Anew_0' 'var_Anew_a' 'var_Anew_b'});
model.study('std_isolate_CW').feature('time').set('rtolactive', true);
model.study('std_isolate_CW').feature('time').set('initstudy', 'std_initialize_CW');
model.study('std_isolate_CW').feature('time').set('initmethod', 'sol');


model.study.create('std_isolate_CCW');
model.study('std_isolate_CCW').create('time', 'Transient');
model.study('std_isolate_CCW').feature('time').set('activate', {'w' 'on' 'w2' 'on' 'w3' 'on' 'w4' 'off' 'w5' 'off'  'w6' 'off'  'w7' 'off' 'w8' 'off'});
model.study('std_isolate_CCW').label('Step 2: Isolate Limit Cycle: CCW');
model.study('std_isolate_CCW').feature('time').set('solnum', 'interp');
model.study('std_isolate_CCW').feature('time').set('useinitsol', 'on');
model.study('std_isolate_CCW').feature('time').set('rtol', '1e-8');
model.study('std_isolate_CCW').feature('time').set('useadvanceddisable', true);
model.study('std_isolate_CCW').feature('time').set('tlist', 'range(0,0.05,15)');
model.study('std_isolate_CCW').feature('time').set('t', '25');
model.study('std_isolate_CCW').feature('time').set('disabledphysics', {'w2/wfeq4' 'w/wfeq16'});
%model.study('std_isolate_CCW').feature('time').set('disabledvariables', {'var_Anew_0' 'var_Anew_a' 'var_Anew_b'});
model.study('std_isolate_CCW').feature('time').set('rtolactive', true);
model.study('std_isolate_CCW').feature('time').set('initstudy', 'std_initialize_CCW');
model.study('std_isolate_CCW').feature('time').set('initmethod', 'sol');

% XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
% XXXXXXXXXXXXXXXXXXXXXXXXXX CREATE  SOLUTIONS XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
% XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

model.sol.create('sol_initialize_CW');
model.sol('sol_initialize_CW').study('std_initialize_CW');
model.sol('sol_initialize_CW').attach('std_initialize_CW');
model.sol('sol_initialize_CW').create('st1', 'StudyStep');
model.sol('sol_initialize_CW').create('v1', 'Variables');
model.sol('sol_initialize_CW').create('t1', 'Time');
model.sol('sol_initialize_CW').feature('t1').create('fc1', 'FullyCoupled');
model.sol('sol_initialize_CW').feature('t1').feature.remove('fcDef');

model.sol.create('sol_initialize_CCW');
model.sol('sol_initialize_CCW').study('std_initialize_CCW');
model.sol('sol_initialize_CCW').attach('std_initialize_CCW');
model.sol('sol_initialize_CCW').create('st1', 'StudyStep');
model.sol('sol_initialize_CCW').create('v1', 'Variables');
model.sol('sol_initialize_CCW').create('t1', 'Time');
model.sol('sol_initialize_CCW').feature('t1').create('fc1', 'FullyCoupled');
model.sol('sol_initialize_CCW').feature('t1').feature.remove('fcDef');

model.sol.create('sol_isolate_CW');
model.sol('sol_isolate_CW').study('std_isolate_CW');
model.sol('sol_isolate_CW').attach('std_isolate_CW');
model.sol('sol_isolate_CW').create('st1', 'StudyStep');
model.sol('sol_isolate_CW').create('v1', 'Variables');
model.sol('sol_isolate_CW').create('t1', 'Time');
model.sol('sol_isolate_CW').feature('t1').create('fc1', 'FullyCoupled');
model.sol('sol_isolate_CW').feature('t1').feature.remove('fcDef');

model.sol.create('sol_isolate_CCW');
model.sol('sol_isolate_CCW').study('std_isolate_CCW');
model.sol('sol_isolate_CCW').attach('std_isolate_CCW');
model.sol('sol_isolate_CCW').create('st1', 'StudyStep');
model.sol('sol_isolate_CCW').create('v1', 'Variables');
model.sol('sol_isolate_CCW').create('t1', 'Time');
model.sol('sol_isolate_CCW').feature('t1').create('fc1', 'FullyCoupled');
model.sol('sol_isolate_CCW').feature('t1').feature.remove('fcDef');
%% Create Datasets and Run
% XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
% XXXXXXXXXXXXXXXXXXXXXXXXXXX CREATE  DATASETS XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
% XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

% Initialize Clockwise and Counter Clockwise States

model.result.dataset.create('dset_initialize_CW', 'Solution');
model.result.dataset('dset_initialize_CW').set('solution', 'sol_initialize_CW');
model.physics('w').feature('init1').set('Qxx', 'S0*C*(nx00(x,y)^2-1/2)');
model.physics('w').feature('init1').set('Qxy', 'S0*C*(nx00(x,y)*ny00(x,y))');
model.sol('sol_initialize_CW').runAll;

model.result.dataset.create('dset_initialize_CCW', 'Solution');
model.result.dataset('dset_initialize_CCW').set('solution', 'sol_initialize_CCW');
model.physics('w').feature('init1').set('Qxx', 'S0*C*(nx00(x,y)^2-1/2)');
model.physics('w').feature('init1').set('Qxy', '-S0*C*(nx00(x,y)*ny00(x,y))');
model.sol('sol_initialize_CCW').runAll;

% Isolate Circulating States

model.result.dataset.create('dset_CW', 'Solution');
model.result.dataset('dset_CW').set('solution', 'sol_initialize_CW');
model.sol('sol_isolate_CW').runAll;

model.result.dataset.create('dset_CCW', 'Solution');
model.result.dataset('dset_CCW').set('solution', 'sol_initialize_CCW');
model.sol('sol_isolate_CCW').runAll;

% Remove Initialization Data Sets and Solutions

model.sol.remove('sol_initialize_CW');
model.sol.remove('sol_initialize_CCW');
model.result.dataset.remove('dset_initialize_CW');
model.result.dataset.remove('dset_initialize_CCW');

mphsave(model, ['LimitCycles_' filenamestr '.mph'])
