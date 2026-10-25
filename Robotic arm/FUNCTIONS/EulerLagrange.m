function [M,L] = EulerLagrange(q,dq,ddq,D,G,m,rc,E)

if ~exist("E","var")
    E = 0;
end

n = length(q);
K = dq'*D*dq/2;
P = 0;
for i=1:n
    P = P - m(i)*G*rc(:,:,i);
end

L = simplify(K - P + E);

M = simplify(hessian(L,dq)*ddq + jacobian(jacobian(L,dq)',q)*dq - jacobian(L,q)');

end