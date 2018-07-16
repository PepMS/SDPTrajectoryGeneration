function F_matrices = SDPFillConst(A_c, b_c, C_c, d_c, nVars, nJoint, ineqDim, Fold)   
    F_matrices = Fold;
    if ~isempty(A_c)
        kk = 2+1*(ineqDim~=0);
        for jj=1:nVars+1
            if jj==1 % F_0
                F_matrices{kk,jj}   = -diag(b_c);
                F_matrices{kk+1,jj} =  diag(b_c);
            elseif jj<=nJoint+1
                F_matrices{kk,jj}   = -diag(A_c(:,jj-1));
                F_matrices{kk+1,jj} =  diag(A_c(:,jj-1));
            elseif jj>nJoint+1
                F_matrices{kk,jj}   = zeros(ineqDim);
                F_matrices{kk+1,jj} = zeros(ineqDim);
            end
        end
    end
    % Inequality Constraints
    if ~isempty(C_c)
        kk = 2+1*(ineqDim~=0)+2*(~isempty(A_c));
        for jj=1:nVars+1
            if jj==1 % F_0
                F_matrices{kk,jj}   = -diag(d_c);
            elseif jj<=nJoint+1
                F_matrices{kk,jj}   = -diag(C_c(:,jj-1));
            elseif jj>nJoint+1
                F_matrices{kk,jj}   = zeros(ineqDim);
            end
        end
    end
end