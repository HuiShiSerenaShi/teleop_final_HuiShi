% Forward kinematics
function x = kinematics(dh, q, tcp_offset)
    if ~isvector(tcp_offset)
        disp('dimension error')
        return
    end
    T = getTransformation(0, dh.dof, dh, q);
    T_tcp = eye(4);
    T_tcp(1:3,4) = tcp_offset'; 
    T = T * T_tcp;

    p = T(1:3,4);
    R = T(1:3,1:3);
    
    angles = rotm2eul(R,'ZYZ');
    
    x = [p; angles'];
end



