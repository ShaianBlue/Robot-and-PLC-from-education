Q = Q2(:,end);

for DXL_ID=DXL_IDS
    write2ByteTxRx(port_num, PROTOCOL_VER, DXL_ID, ADDR_MX_GOAL_POSITION, Q(DXL_ID))
end
pause(1.5)

image = takeimage();

[h,J,r] = robotequation();

Q = [0 0 -pi/2 -pi/2]';

coor = num2cell(Q);
P = RedSmartiesDetector(image,h(coor{:}));

len = size(P,2);
if len ~= 1
    P(:,end + len - 1) = 0;
    P(:,1:2:end) = P(:,1:len);
    P(:,2:2:end) = (P(:,1:2:end-2) + P(:,3:2:end))/2;
    P(3,2:2:end) = 0.05;
    P(4,:) = -1;
end

Q = [0 0 -pi/2 -pi/2]'-0.1;
Q = global2dof(Q,r,J,P);
Q = [Q(:,2:end) Q(:,1)];
Q(:,end) = [0 0 -pi/2 -pi/2];
Q2 = rad2bites(Q);

pause

for i=1:size(Q2,2)
    coordinates = Q2(:,i);
    for DXL_ID=DXL_IDS
        write2ByteTxRx(port_num, PROTOCOL_VER, DXL_ID, ADDR_MX_GOAL_POSITION, coordinates(DXL_ID))
    end
    pause(1.5)
end










