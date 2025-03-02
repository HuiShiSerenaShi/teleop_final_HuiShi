% loading kinematics parameters of the robot
dh_definition;
% optional transformation between flange and end-effector
flange2ee_vec = [0 0 0]';

% initial configuration of the joints
q0 = zeros(7,1);
dq0 = zeros(7,1);

% forward kinematics (by default the orientation is expressed with euler
% angles using ZYZ convention)
kinematics(dh,q0,flange2ee_vec);

% inverse kinematics is not available, for any convertion between cartesian
% space and generalized coordinates use the pseudo-inverse of the jacobian
% or the transpose of the jacobian

% geometric jacobian
J = Jacobian(dh,q0,flange2ee_vec);

% analytical jacobian
Ja = Jacobian_a(dh,q0,flange2ee_vec);