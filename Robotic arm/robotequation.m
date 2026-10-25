function [h,J,r] = robotequation()

n = 4;
syms q dq ddq [n 1] real

DH = [q1 0.0925 0 pi/2
      q2+pi/2 0 0.095 0
      q3 0 0.095 0
      q4 0 0.065 0];

CM = [0 -0.05 0; -0.03 0 0; -0.03 0 0; -0.07 0.01 0]';
CM(4,:) = 1;

Tend = eye(4);
Tend(2,4) = 0.022;

Tcam = eye(4);
Tcam(1,4) = -0.04;
Tcam(2,4) = 0.045;

[T,H,rc] = tmatrix(DH,CM);

Hcam = H*Tcam;
h = matlabFunction(Hcam);

H = H*Tend;
r = H(1:3,4);
w = H(1:3,1);
r(4) = [0 0 1]*w;

[J,~] = jacb(T,rc,r,q);

end