function size = get_robot_size(robot_size, type)
% size = get_robot_size(robot_size, type)
%
% Function that returns the requested robot size (type), from the 4
% elements vector robot_size. robot_size vector has the following element:
% [front_size back_size right_size left_size]. Possible value for type are:
% 'front', 'back', 'right', 'left', 'front-back', 'right-left'

    if(length(robot_size) ~= 4)
        error('chk:size', 'robot_size must be 4 element vector: [front_size back_size right_size left_size]');
    end
    
    switch(type)
        case 'front'
            size = robot_size(1);
        case 'back'
            size = robot_size(2);
        case 'right'
            size = robot_size(3);
        case 'left'
            size = robot_size(4);
        case 'front-back'
            size = robot_size(1) + robot_size(2);
        case 'right-left'
            size = robot_size(3) + robot_size(4);
        otherwise
            error('chk:type', ['Unknown requested type: ' type ' . Possible types are: front, back, right, left']);
    end

end