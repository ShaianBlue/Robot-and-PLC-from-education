function Q = global2dof(Q,r,J,P)

pos = num2cell(Q);

f = matlabFunction(r);
df = matlabFunction(J);

for i=1:size(P,2)

    while norm(f(pos{:})-P(:,i))>1e-10
    
        pos = mod( cell2mat(pos) - df(pos{:})\(f(pos{:})-P(:,i)) , 2*pi);
    
        if any(pos>pi)
            pos(pos>pi) = pos(pos>pi) - 2*pi;
        end
    
        pos = num2cell(pos);
    
    end
    
    Q(:,i+1) = cell2mat(pos);

end

end