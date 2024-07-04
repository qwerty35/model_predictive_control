function sdot = car_dynamics(t, state, params)
    V = params.V;
    u = params.u;
    
    theta = state(3); % state = [x,y,theta]
        
    sdot = [V * cos(theta);
            V * sin(theta);
            u];
end