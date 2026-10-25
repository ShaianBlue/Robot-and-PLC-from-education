function P = pix2glob(Q,H)

K = [729.0639, 0, 336.3515; 0, 728.0372, 246.7135; 0, 0, 1];

Q = transpose(Q);
Q(3,:) = 1;

Pcam = K\Q*H(3,4);
Pcam(4,:) = 1;
Pcam([1 3],:) = Pcam([3 1],:);
Pcam(2,:) = -Pcam(2,:);

P = H(1:3,:)*Pcam;

end