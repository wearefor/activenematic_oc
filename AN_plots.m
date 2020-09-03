% XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
% Active Nematic Optimal Control
% Plotting Results
% (C) 2020 Michael M. Norton
% Brandeis University, Physics
% Pennsylvania State University, Center for Neural Engineering
% XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

close all;
clear all;
clc;

%%
pathsetstr='full path here'
filestr='mph file name'
model=mphload([pathsetstr '/' filestr]);

dsetnamestr='dsetname for Q u p';
dsetnamestr_control='dsetname for control';
%mphnavigator(model); %if data set name string is unknown: 1) use mphnavigator or 2) open mph file in Comsol to help navigate the study/solution/data trees
%*note that data sets containing {Q,u,p} dynamics and control solutions will be different!*

params.N=100; %spatial resolution for interpolating solution
params.alpha0=str2num(model.param.get('alpha0'));
params.DomRad=str2num(model.param.get('DomRad'));
params.rho0=str2num(model.param.get('rho0'));
xcoord=linspace(-params.DomRad,params.DomRad,params.N);
ycoord=linspace(-params.DomRad,params.DomRad,params.N);

[x_grid,y_grid]=meshgrid(xcoord,ycoord);
xlist=reshape(x_grid,params.N^2,1);
ylist=reshape(y_grid,params.N^2,1);
coord=[xlist'; ylist'];
time=1;

[Qxx,Qxy,ux,uy,vort]=mphinterp(model,{'Qxx','Qxy','ux','uy','uyx-uxy'},'coord',coord,'dataset',dsetnamestr,'t',time);
[controlfield]=mphinterp(model,{'Anew'},'coord',coord,'dataset',dsetnamestr_control,'t',time);

figure; func_plot_director(Qxx,Qxy,x_grid,y_grid,params);
figure; func_plot_velocity(ux,uy,vort,x_grid,y_grid,params);
figure; func_plot_controlfield(controlfield,x_grid,y_grid,params);
