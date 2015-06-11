%  %%                Two Bodies Interacting in ONE DIFFERENTIAL EQUATION WHAAAAA?
%                        <<       wah woh won't work :(      >>
% 
%     function dx = dirDynTwoBodies(t,x)
%         
%         Q1 = x(1:end/4);                                                                          % Stores position values
%         Qdot1 = x(end/4+1:end/2);                                                                   % Stores velocity values
%         Q2 = x(end/2+1:3*end/4);                                                                          % Stores initial postion
%         Qdot2 = x(3*end/4+1:end);                                                                   % Stores initial velocities
%         [Qddot1, F] = S2M_rbdl('forwardDynamics', m1, Q1, Qdot1, tau, true);            % Calculates final forces exherted on the contact point by body one using final state of m1
%         dx1 = [Qdot1; Qddot1];                                                                      % Returns velocities and accelerations in a format useful for ODE 45
%         S2M_rbdl('gravity', m2, [0 0 0]');                                                       % Sets gravity of m2 to zero
%         taus = S2M_rbdl('inverseDynamics', m2, Q2, Qdot2,[0;0;0;0], [0; 0; 0; F(1); 0; F(2)]);     % Calculates impact of forces on a stationary m2 in vaccuum
%         S2M_rbdl('gravity', m2, [0 0 -9.81]');                                                   % Sets gravity of m2 back to -9.81
% 
%         TauProp = tau+taus;                                                                      % Adds initial value tau to torques created by the forces from m1
%         Qddot2 = S2M_rbdl('forwardDynamics', m2, Q2, Qdot2, TauProp);                              % Calculates accelerations
%         dx = [dx1(1:end/2); Qdot2; dx1(end/2+1:end); Qddot2];                                                                    % Returns velocities and accelerations in a format compatible with ODE45
%     end 