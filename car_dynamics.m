function sdot = car_dynamics(t, s, params)
    % s = [x,y,theta]
    
    V = params.V;
    u = params.u;
    
    theta = s(3);
        
    sdot = [V * cos(theta);
            V * sin(theta);
            u];
end