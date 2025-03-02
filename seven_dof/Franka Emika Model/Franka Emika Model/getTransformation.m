% Calculates Transformation matrix
function  T = getTransformation(from, to, dh, qc)
    % Transformation from one joint to another joint
    % 0<=from<N_DOFS
    % 0<to<=N_DOFS    
    T = eye(4);
    N_DOFS = length(qc);
    % Sanity check
    if (from >= N_DOFS) || (from < 0) || (to <= 0) || (to >  N_DOFS)
        return;
    end 
    for i = from+1:to
        ct = cos(qc(i));
        st = sin(qc(i));

        if dh.modified == false
            ca = cos(dh.alpha(i));
            sa = sin(dh.alpha(i));
            a = dh.a(i);           
            T = T * [ ct    -st*ca   st*sa     a*ct ; ...
                      st    ct*ca    -ct*sa    a*st ; ...
                      0     sa       ca        dh.d(i); ...
                      0     0        0         1];
        else
            ca = cos(dh.alpha(i));
            sa = sin(dh.alpha(i));
            a = dh.a(i);
            T = T * [ ct    -st     0   a; ...                 
                      st*ca ct*ca   -sa -dh.d(i)*sa;...
                      st*sa ct*sa   ca  dh.d(i)*ca;...
                      0     0       0   1];
        end
    end
end