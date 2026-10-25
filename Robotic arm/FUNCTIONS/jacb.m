function [J,Jc] = jacb(T,rc,r,var)

n = size(T,3);

J = jacobian(r,var);

Jc = sym(zeros(3,length(var),n));

Jw = reshape(T(1:3,3,:),[3 n]);

for i=1:n
    Jc(1:3,:,i) = jacobian(rc(:,:,i),var);
    Jc(4:6,1:i,i) = Jw(:,1:i);
end

end