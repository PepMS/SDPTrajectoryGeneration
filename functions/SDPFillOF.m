function F_matrices = SDPFillOF(A, b, C, d, nVars, nJoints, isIneq)
    for jj=1:nVars+1
        if jj==1 % F_0
            F_matrices{1,jj} = -eye(size(A,1)+1);
            F_matrices{1,jj}(1, 1) = 0;
        elseif jj<=nVars
            F_matrices{1,jj} = [-b(jj-1), A(:,jj-1)'; A(:,jj-1), zeros(size(A,1))];
        elseif jj==nVars+1
            F_matrices{1,jj} = zeros(size(A,1)+1);
            F_matrices{1,jj}(1,1) = 1;
        end
    end
    % In case there there is an inequality task
    if (isIneq)
        for jj=1:nVars+1
            if jj==1 % F_0
                F_matrices{2,jj} = -diag(d);
            elseif jj<=nJoints+1
                F_matrices{2,jj} = -diag(C(:,jj-1));
            elseif (jj>nJoints+1 && jj < nVars+1)
                kk = jj - (nJoints+1);
                F_matrices{2,jj} = zeros(size(C,1));
                F_matrices{2,jj}(kk,kk) = 1;
            elseif jj == nVars+1
                F_matrices{2,jj} = zeros(size(C,1));
            end
        end
    end
end
