function func_plot_flowfield(ux,uy,vort,x_grid,y_grid,params)

    ux_grid=reshape(ux,params.N,params.N);
    uy_grid=reshape(uy,params.N,params.N);
    vort_grid=reshape(vort,params.N,params.N);

    contourf(x_grid,y_grid,vort_grid,20,'LineColor','none');
    colorbar

    h_slice=streamslice(x_grid,y_grid,ux_grid,uy_grid,1);
    set(h_slice,'Color','k','LineWidth',0.8);

    theta=linspace(0,2*pi,300);
    plot(cos(theta)*params.DomRad,sin(theta)*params.DomRad,'k','Linewidth',2);

return
