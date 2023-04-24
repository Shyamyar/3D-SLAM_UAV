function quad = Draw_Quad(phi, theta, psi , x, y, z, col, width)

w = 1; %width of quad box
AC = [1, 0, 0; 0, 1, 0; 0, 0, 1]; 
%v1B to v13B are in Body Frames
v1B = [w/2; w/2; w/2]; 
v2B = [w/2; -w/2; w/2]; 
v3B = [w/2; -w/2; -w/2]; 
v4B = [w/2; w/2; -w/2]; 
v5B = [-w/2; w/2; w/2]; 
v6B = [-w/2; -w/2; w/2]; 
v7B = [-w/2; -w/2; -w/2]; 
v8B = [-w/2; w/2; -w/2]; 
v9B = [0; 0; w/2]; 
v10B = [3 * w/2; 0; w/2]; 
v11B = [0; 3 * w/2; w/2]; 
v12B = [-3 * w/2; 0; w/2]; 
v13B = [0; -3 * w/2; w/2]; 

posN = [x; y; z]; %Position in Inertial Frame

% RB2N = RM_body2inertial(phi, theta, psi); %DCM to convert from body to inertial
RB2N = C(3, psi) * C(2, theta) * C(1, phi);    % 3-2-1 Rotation matrix for Body to Inertial

%v1N to v13N are in Inertial Frame
v1N = AC * (RB2N * v1B + posN); 
v2N = AC * (RB2N * v2B + posN); 
v3N = AC * (RB2N * v3B + posN); 
v4N = AC * (RB2N * v4B + posN); 
v5N = AC * (RB2N * v5B + posN); 
v6N = AC * (RB2N * v6B + posN); 
v7N = AC * (RB2N * v7B + posN); 
v8N = AC * (RB2N * v8B + posN); 
v9N = AC * (RB2N * v9B + posN); 
v10N = AC * (RB2N * v10B + posN); 
v11N = AC * (RB2N * v11B + posN); 
v12N = AC * (RB2N * v12B + posN); 
v13N = AC * (RB2N * v13B + posN); 

quad = plot3([v1N(1), v2N(1), v3N(1), v4N(1), v1N(1)], [v1N(2), v2N(2), v3N(2), v4N(2), v1N(2)], [v1N(3), v2N(3), v3N(3), v4N(3), v1N(3)], col...
    , [v3N(1), v4N(1), v8N(1), v7N(1), v3N(1)], [v3N(2), v4N(2), v8N(2), v7N(2), v3N(2)], [v3N(3), v4N(3), v8N(3), v7N(3), v3N(3)], col...
    , [v8N(1), v7N(1), v6N(1), v5N(1), v8N(1)], [v8N(2), v7N(2), v6N(2), v5N(2), v8N(2)], [v8N(3), v7N(3), v6N(3), v5N(3), v8N(3)], col...
    , [v6N(1), v5N(1), v1N(1), v2N(1), v6N(1)], [v6N(2), v5N(2), v1N(2), v2N(2), v6N(2)], [v6N(3), v5N(3), v1N(3), v2N(3), v6N(3)], col...
    , [v9N(1), v10N(1)], [v9N(2), v10N(2)], [v9N(3), v10N(3)], col...
    , [v9N(1), v11N(1)], [v9N(2), v11N(2)], [v9N(3), v11N(3)], col...
    , [v9N(1), v12N(1)], [v9N(2), v12N(2)], [v9N(3), v12N(3)], col...
    , [v9N(1), v13N(1)], [v9N(2), v13N(2)], [v9N(3), v13N(3)], col); 

quad(5).LineWidth = width; 
quad(5).Marker = "."; 
quad(5).MarkerIndices = 2; 
quad(5).MarkerSize = 10; 
quad(6).LineWidth = width; 
quad(6).Marker = "."; 
quad(6).MarkerIndices = 2; 
quad(6).MarkerSize = 10; 
quad(7).LineWidth = width; 
quad(7).Marker = "."; 
quad(7).MarkerIndices = 2; 
quad(7).MarkerSize = 10; 
quad(8).LineWidth = width; 
quad(8).Marker = "."; 
quad(8).MarkerIndices = 2; 
quad(8).MarkerSize = 10; 

end