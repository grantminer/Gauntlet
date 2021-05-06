function positions = gauntlet_drive_overhaul(x, y, fx, fy)

lambda = 1;
steps = 0;
speed = 0.1;

pubvel = rospublisher('/raw_vel/');
sub_bump = rossubscriber('/bump');
message = rosmessage(pubvel);

positions = [0; 0];
current_heading = [1; 0];

message.Data = [0 0];
send(pubvel, message);

placeNeato(positions(1), positions(2),current_heading(1),current_heading(2))

pause(2);

while 1
    
    current_x = floor((positions(1, end) - min(x, [], 'all'))/0.07)+1;
    current_y = floor((positions(2, end) - min(y, [], 'all'))/0.07)+1;
    
    current_gradient = -1 * [fx(current_y, current_x); fy(current_y, current_x)];
    
    if steps > 10000
        break
    end
    
    crossvec = cross([current_heading;0], [current_gradient;0]);
    
    turn_angle = asin(norm(crossvec)/(norm(current_heading)*norm(current_gradient)));
    
    time = double(turn_angle) / speed;
    
    turn = rostic;
    
    if crossvec(3) > 0
        message.Data = [-1*speed*0.1175, speed*0.1175];
    elseif crossvec(3) < 0
        message.Data = [speed*0.1175, -1*speed*0.1175];
    end
    
    send(pubvel, message)
    
    while 1
        elapsed = rostoc(turn);
        
        if elapsed > time
            break
        end
        pause(0.05);
    end
    
    current_heading = current_gradient;
    
    message.Data = [speed, speed];
    
    send(pubvel, message);
    
    drive = rostic;
    
    drive_distance = norm(lambda * current_gradient);
    
    while 1
        drive_elapse = rostoc(drive);
        
        bumpMessage = receive(sub_bump);
        % check if any of the bump sensors are set to 1 (meaning triggered)
        if any(bumpMessage.Data)
            message.Data = [0.0, 0.0];
            send(pubvel, message);
            positions = [positions, (positions(:, end)+ (drive_elapse*speed/drive_distance)*lambda*current_gradient)];
            break;
        end
        
        if drive_elapse > (drive_distance/speed)
            message.Data = [0, 0];
            send(pubvel, message)
            break
        end
    end
    
    

    
    positions = [positions, (positions(:, end) + lambda * current_gradient)];
    
    lambda = 0.9*lambda;
    
    steps = steps + 1;
end

end