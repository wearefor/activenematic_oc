function func_plot_director(Qxx,Qxy,x_grid,y_grid,params)

S=2*(Qxx.^2+Qxy.^2).^(1/2)./params.rho0;
nx=sqrt(Qxx./sqrt(2.*(2*Qxx.^2+2*Qxy.^2))+1/2);
ny=sqrt(1-(sqrt(Qxx./sqrt(2*(2*Qxx.^2+2*Qxy.^2))+1/2)).^2).*sign(Qxy);

S_grid=reshape(S,params.N,params.N);
nx_grid=reshape(nx,params.N,params.N);
ny_grid=reshape(ny,params.N,params.N);

% remove "seam" artifacts from director streamline plot
theta=atan2(ny_grid,nx_grid);
[theta_x,theta_y]=gradient(theta);
theta_grad_mag=theta_x.^2+theta_y.^2;
nx_plot=nx_grid;
ny_plot=ny_grid;
nx_plot(theta_grad_mag>0.05)=NaN;
ny_plot(theta_grad_mag>0.05)=NaN;

contourf(x_grid,y_grid,S_grid,20,'LineColor','none');
colorbar
hold on
axis square

h_slice=streamslice(x_grid,y_grid,nx_plot,ny_plot,1,'noarrows');
set(h_slice,'Color','k','LineWidth',0.8);

phi=linspace(0,2*pi,300);
plot(cos(phi)*params.DomRad,sin(phi)*params.DomRad,'k','Linewidth',2);

return
