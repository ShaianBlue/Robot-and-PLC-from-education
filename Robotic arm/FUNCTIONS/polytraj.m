function Z = polytraj(P,V,A)

if ~exist("A","var")
    A = [];
end
if ~exist("V","var")
    V = [];
end

Y = [P(:,2); V(:,2); A(:,2)];

n = size(Y,1);
p = flip(0:n-1);
po = ones(1,n);

if ~isempty(V)
    dpo = polyder(po);
    dpo = kron(dpo,ones(size(V,1),1));
    V = dpo.*(V(:,1).^p(2:end));
end

if ~isempty(A)
    ddpo = polyder(polyder(po));
    ddpo = kron(ddpo,ones(size(A,1),1));
    A = ddpo.*(A(:,1).^p(3:end));
end

X = [P(:,1).^p; V zeros(size(V,1),1); A zeros(size(A,1),2)];

Z = X\Y;

end