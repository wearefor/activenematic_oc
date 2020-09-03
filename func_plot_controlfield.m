function func_plot_controlfield(control,x_grid,y_grid,params)

    control_grid=reshape(control,params.N,params.N);

    contourf(x_grid,y_grid,control_grid,20,'LineColor','none');
    colorbar

    theta=linspace(0,2*pi,300);
    plot(cos(theta)*params.DomRad,sin(theta)*params.DomRad,'k','Linewidth',2);

return
