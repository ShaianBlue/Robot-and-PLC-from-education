function w = omega(DH,T,q)

n = size(T,3);

v = jacobian([DH(:,1); DH(:,4)],q);

H = eye(3);
w = zeros(3,length(q));

for i=1:n
    w = w + H*[v(i+n,:); zeros(1,length(q)); v(i,:)];
    H = H*T(1:3,1:3,i);
end

end