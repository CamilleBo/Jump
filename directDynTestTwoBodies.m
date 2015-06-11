function directDynTest%(m, Qinit)
run('./S2M_Lib/loadS2MLib_pwd.m')
persistent m

    try %#ok<TRYNC>
        S2M_rbdl('delete', m);
    end
    clc; clear; close all;

    tau = [0 0 0 0 0 0]';
    m = S2M_rbdl('new', './example2.s2mMod');
    n = S2M_rbdl('new', './example2.s2mMod');
    xinitm = [0; 0.00000; 0; 0; 0; 0; 0; 0; 0; zeros(9,1)];
    xinitn = [0; 0.00000; pi; 0; 0; 0; 0; 0; 0; zeros(9,1)];
    figure 
    view([0 0])
    xlim([-1 1]);
    ylim([-1 1]);
    zlim([-1 1]);
    h = S2M_rbdl_ShowModel(m,convertQ(xinitm(1:end/2)));
    hold on
    k = S2M_rbdl_ShowModel(n, convertQ(xinitn(1:end/2)));
    Xintm = ode45(@dirDyn, [0 5], xinitm);
    Xintn = ode45(@dirDyn, [0 5], xinitn);
    
    function dx = dirDyn(t,x)
        Q = x(1:end/2);
        Qdot = x(end/2+1:end);
        [Qddot, F] = S2M_rbdl('forwardDynamics', m, convertQ(Q), convertQ(Qdot), convertTAU(tau), true);
        dx = [Qdot; convertQ(Qddot, true)];
        F
%         tau1 = S2M_rbdl('inversedynamics', m, convertQ(Q), convertQ(Qdot), convertQ(Qddot));
%         tau = convertTAU(tau1(4:end), true)
%        Qddot;
%         h = S2M_rbdl_ShowModel(m, convertQ(Q),h);
%         drawnow
    end


    Qintm = Xintm.y(1:end/2,:);
    Qintn = Xintn.y(1:end/2,:);
    S2M_rbdl_AnimateModel(m,convertQ(Qintm));
    hold on
    S2M_rbdl_AnimateModel(n,convertQ(Qintn));
end