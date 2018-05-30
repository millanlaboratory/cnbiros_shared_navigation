function velocity = dynamic_velocity_linear(obs_distance, obs_min_distance, obs_max_distance, vel_minimum, vel_maximum, strength)

    if length(obs_distance) > 1
        error('chk:length', 'Provide just a obstacle distance value at the time');
    end

    x1 = obs_min_distance;
    x2 = obs_max_distance;
    y1 = vel_minimum;
    y2 = vel_maximum;
    
    x = obs_distance;
    
    m = (1/(1+strength))*(y2 - y1)./(x2 - x1);
    b = y1 - m*x1;
    
    
    
    if(obs_distance < obs_min_distance)
        y = m*x + b -2*vel_minimum;
    else 
        y = m*x + b;
    end
    
    if(y > vel_maximum)
        y = vel_maximum;
    elseif(y < -vel_maximum)
        y = -vel_maximum;
    end
    
    
    velocity = y;

end