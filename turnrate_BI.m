function turnrate_tf = turnrate_BI(phi,theta)

turnrate_tf = [1,sin(phi)*tan(theta),cos(phi)*tan(theta);
        0,cos(phi),-sin(phi);
        0,sin(phi)/cos(theta),cos(phi)/cos(theta)];