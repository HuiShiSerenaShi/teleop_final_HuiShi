% Geometric Jacobian
% dh: dh table of the robot 
% q: joints position

function J = Jacobian(dh, q, tcp_offset, index)
    if ~isvector(tcp_offset)
        disp('errore di dimensione')
        return
    end
    if nargin == 4
        T = getTransformation(0, index, dh, q);
        tcp_offset = [tcp_offset' 1]';
        pe = T * tcp_offset;
        pe = pe(1:3,:);
        n = index;
    else
        T = getTransformation(0, length(q), dh, q);
        tcp_offset = [tcp_offset' 1]';
        pe = T * tcp_offset;
        pe = pe(1:3,:);
        n = length(q);
        
    end

    J = zeros(6,length(q));
    zi_m_1 = [0 0 1]';
    pi_m_1 = [0 0 0]';
    
    J(1:3, 1) = cross(zi_m_1, (pe-pi_m_1));
    J(4:6, 1) = zi_m_1;
   
    for i=2:n
        if dh.modified == true
            t_index = i;
        else
            t_index = i-1;
        end

        Ti_m_1 = getTransformation(0, t_index, dh, q);
        
        zi_m_1 = Ti_m_1(1:3, 3);
        pi_m_1 = Ti_m_1(1:3,4);
        
        J(1:3, i) = cross(zi_m_1, (pe - pi_m_1));
        J(4:6, i) = zi_m_1;
    end
end