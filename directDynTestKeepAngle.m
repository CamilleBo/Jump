function directDynTest%(m, Qinit)
run('./S2M_Lib/loadS2MLib_pwd.m')
persistent m

    try %#ok<TRYNC>
        S2M_rbdl('delete', m);
    end
    clc; clear; close all;

    tau = [0 0 0 0 0 0]';
    m = S2M_rbdl('new', './example2.s2mMod');
   % n = S2M_rbdl('new', './example2.s2mMod');
    xinit = [0; 0.00000; 0; 0.00; 0;0;pi/6;0; 0;zeros(9,1)];
    figure 
    view([0 0])
    xlim([-1 1]);
    ylim([-1 1]);
    zlim([-1 3]);
    h = S2M_rbdl_ShowModel(m,convertQ(xinit(1:end/2)));
    Xint = ode45(@dirDyn, [0 2], xinit);
        

    
    function dx = dirDyn(t,x)
        Q = x(1:end/2);
        Qdot = x(end/2+1:end);
%        Qddot1 = zeros(size(x));
%        tau1 = S2M_rbdl('inversedynamics', m, convertQ(Q), convertQ(Qdot), convertQ(Qddot1));
%        tau = convertTAU(tau1(4:end), true);
       [Qddot, F] = S2M_rbdl('forwardDynamics', m, convertQ(Q), convertQ(Qdot), convertTAU(tau), true);
       Qdot(7) = 0;
       Qddot(7) = 0;
       dx = [Qdot; convertQ(Qddot, true)];
     
%         h = S2M_rbdl_ShowModel(m, convertQ(Q),h);
%         drawnow
    end


    Qint = Xint.y(1:end/2,:);
    S2M_rbdl_AnimateModel(m,convertQ(Qint));
end