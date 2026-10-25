function [T,H,rc] = tmatrix(DH,CM)

len = size(DH,1);
T = sym(zeros(4,4,len));
rc = sym(zeros(3,1,len));
H = eye(4);

for i=1:len

    so = sin(DH(i,1));
    co = cos(DH(i,1));
    sa = sin(DH(i,4));
    ca = cos(DH(i,4));

    T(:,:,i) = [co -so.*ca so.*sa DH(i,3).*co
                so co.*ca -co.*sa DH(i,3).*so
                0 sa ca DH(i,2)
                0 0 0 1];

    H = H*T(:,:,i);

    rc(:,:,i) = H(1:3,:)*CM(:,i);

end

H = simplify(H);
rc = simplify(rc);

end