function func_plotcontrol(ii,params)

global model

if ii==0

    switch lower(params.controltype)
        case 'torque'
            [con]=mphmax(model,{'abs(G(t))'},'surface','dataset','dset_forward_0','t',params.tlist);
        case 'stress'
            [con]=mphmax(model,{'abs(A(t))'},'surface','dataset','dset_forward_0','t',params.tlist);
    end

elseif ii==1

    switch lower(params.controltype)
        case 'torque'
            [con]=mphmax(model,{'abs(Gnew)'},'surface','dataset','dset_update_0','t',params.tlist);
        case 'stress'
            [con]=mphmax(model,{'abs(Anew)'},'surface','dataset','dset_update_0','t',params.tlist);
    end

else
    if mod(ii,2)==0
        switch lower(params.controltype)
            case 'torque'
                [con]=mphmax(model,{'abs(Gnew)'},'surface','dataset','dset_update_b','t',params.tlist);
            case 'stress'
                [con]=mphmax(model,{'abs(Anew)'},'surface','dataset','dset_update_b','t',params.tlist);
        end
    elseif mod(ii,2)==1
        switch lower(params.controltype)
            case 'torque'
                [con]=mphmax(model,{'abs(Gnew)'},'surface','dataset','dset_update_a','t',params.tlist);
            case 'stress'
                [con]=mphmax(model,{'abs(Anew)'},'surface','dataset','dset_update_a','t',params.tlist);
        end
    else

    end

end


disp('----------------------------------------------------------------')
disp(['     plotting max(abs(control)), scale = ' num2str(max(con))])
asciiplot(params.tlist,con);
disp('----------------------------------------------------------------')



return
