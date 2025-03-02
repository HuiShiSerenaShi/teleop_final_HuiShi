% Analytical Jacobian
% dh: dh table of the robot
% q: joints position

function Ja = Jacobian_a(dh, q, tcp_offset)
    x = kinematics(dh, q, tcp_offset);

    phi = x(4);
    theta = x(5);
    
    Transformation = [0 -sin(phi) cos(phi)*sin(theta); ...
                      0  cos(phi) sin(phi)*sin(theta); ...
                      1  0        cos(theta)];

    Ta = [eye(3) zeros(3); zeros(3) Transformation];

    if abs(det(Ta)) <= 1e-3
       disp("Representation singularity");
    end

    Ja = inv(Ta) * Jacobian(dh, q, tcp_offset, length(q));
end