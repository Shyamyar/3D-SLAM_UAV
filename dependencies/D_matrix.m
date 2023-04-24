function D = D_matrix(phi, theta)

D = (1/cos(theta)) * [1, sin(phi) * sin(theta),  cos(phi) * sin(theta);
                      0, cos(phi) * cos(theta), -sin(phi) * cos(theta);
                      0,              sin(phi),               cos(phi)];