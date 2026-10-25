function D = massmatrix(T,Jc,l1,l2,l3,m)

I(1,1,:) = (l1.^2 + l2.^2).*m/12;
I(2,2,:) = (l3.^2 + l2.^2).*m/12;
I(3,3,:) = (l1.^2 + l3.^2).*m/12;

I = sym(I);

for i=1:size(I,3)
    I(:,:,i) = transpose(T(1:3,1:3,i))*I(:,:,i)*T(1:3,1:3,i);
end

D = zeros(size(Jc,2));

for i=1:size(I,3)
    D = D + transpose(Jc(1:3,:,i))*Jc(1:3,:,i)*m(i) + transpose(Jc(4:6,:,i))*I(:,:,i)*Jc(4:6,:,i);
end

D = simplify(D);

end