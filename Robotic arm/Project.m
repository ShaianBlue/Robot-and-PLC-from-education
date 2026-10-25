clear
clc

addpath FUNCTIONS\

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Part 1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Problem I

n = 4;
syms q dq ddq [n 1] real

DH = [q1 0.05 0 pi/2
      q2+pi/2 0 0.093 0
      q3 0 0.093 0
      q4 0 0.05 0];

CM = [0 -0.02 0; -0.03 0 0; -0.03 0 0; -0.025 0.015 0]';
CM(4,:) = 1;

Tcam = eye(4);
Tcam(1,4) = -0.015;
Tcam(2,4) = 0.045;

[T,H,rc] = tmatrix(DH,CM);

Hcam = H*Tcam;

%% Problem II

p = [0 0 0 1]';
r = H(1:3,:)*p;
w = H(1:3,1); % orientation of the end efector
% for the end effector to be horizontal we impose that w must be 
% orthogonal to the global Z axis (0,0,1) meaning that
r(4) = [0 0 1]*w;

f = matlabFunction(r);

[J,Jc] = jacb(T,rc,r,q);

df = matlabFunction(J);

%% Problem III
t = 37;
phi = linspace(0,2*pi,t);

P = [150; 0; 120]/1000 + 32/1000*[zeros(1,t); cos(phi); sin(phi)];
P(4,:) = zeros(1,t);

Q = [0.2 -0.5 -0.5 0]';

Q = global2dof(Q,r,J,P);
Q(:,1) = [];

%% Problem IV

rcam = Hcam(1:3,:)*p;
wcam = Hcam(1:3,1); % orientation of the camera
% for the camera to be horizontal we impose that w must be 
% orthogonal to the global Z axis (0,0,1) meaning that
rcam(4) = [0 0 1]*wcam;

fcam = matlabFunction(rcam);

[Jcam,~] = jacb(T,rc,rcam,q);

dfcam = matlabFunction(Jcam);
Jsol = zeros(2*n,n,n);

for i=0:n-1
    m = 9*i+1;
    Jsol(:,:,i+1) = [df(Q(1,m),Q(2,m),Q(3,m),Q(4,m))
                     dfcam(Q(1,m),Q(2,m),Q(3,m),Q(4,m))];
end

%% Problem V

v = [0 -0.003 0 0]';

dq90 = df(Q(1,10),Q(2,10),Q(3,10),Q(4,10))\v;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Part 2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Problem VI

df = matlabFunction(J(1:3,:));

pos = Q(:,1:9:end);

dr = zeros(3,5);
dr(2,2) = -0.027;
dr(3,3) = -0.027;
dr(2,4) = 0.027;

vel = zeros(size(pos));

for i=1:size(pos,2)
    var = num2cell(pos(:,i));
    vel(:,i) = df(var{:})\dr(:,i);
end

acc = vel*0;

for j=1:size(pos,2)-1
    for i=1:n
        P = [0 pos(i,j); 2 pos(i,j+1)];
        V = [0 vel(i,j); 2 vel(i,j+1)];
        A = [0 acc(i,j); 2 acc(i,j+1)];
        Z6(:,i,j) = polytraj(P,V,A);
    end
end

%% Problem VII

color = [1 0 0; 0 1 0; 0 0 1; 0 0 0];

figure
hold on
grid on

x = linspace(0,8,37);
for i = 1:4
    plot(x,rad2deg(Q(i,1:end)),LineWidth=1,Color=color(i,:))
end

pq6 = [];
x = [];
t = linspace(0,2,7);

for j=1:size(Z6,3)
    pq1 = [polyval((Z6(:,1,j)),t) 
           polyval((Z6(:,2,j)),t)  
           polyval((Z6(:,3,j)),t) 
           polyval((Z6(:,4,j)),t)];
    pq6 = [ pq6 pq1 ];
    x = [ x t+2*(j-1)];
end

for i = 1:4
    plot(x,rad2deg(pq6(i,:)),'o',LineWidth=2,Color=color(i,:))
    % plot(x,rad2deg(pq(i,:)),'--',LineWidth=1,Color=color(i,:))
end

tmax = 4;
tmin = -4;

V = [tmin 0; tmax 0];
A = [tmin 0; tmax 0];

clear P

for i=1:4
    P(:,2) = Q(i,1:4:end);
    P(:,1) = linspace(tmin,tmax,size(P,1));
    Z(:,i) = polytraj(P,V,A);
end

t = linspace(tmin,tmax,25);

for i = 1:4
    pq = polyval(Z(:,i),t);
    plot(t-tmin,rad2deg(pq),'x','MarkerSize',10,LineWidth=2,Color=color(i,:))
    % plot(t-tmin,rad2deg(pq),'-',LineWidth=1,Color=color(i,:))
end

xlabel('Time (s)', 'Interpreter', 'latex');
ylabel('Angle/time ($^\circ$)', 'Interpreter', 'latex');
title('Trajectory Approximation');
legend({' ', ' ', ' ', ' ', 'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', ' ', ' ', ' ', ' '},...
    'Orientation','horizontal',NumColumns=4,Location='best');

% dim = [0.2 0.5 0.3 0.3];
% str = {'Actual Path','2 pts 2 s 4 stps Approx','10 pts 8 s 1 stp Approx'};
% annotation('textbox',dim,'String',str,'FitBoxToText','on');

fontsize(16,"points")
yticks(-120:10:40)
xticks(0:0.5:8)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Part 3 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Problem VIII

for i=1:size(Q,2)
    x = num2cell(Q(:,i));
    cond_prob3(i) = cond(df(x{:}));
end

for i=1:length(pq6)
    x = num2cell([pq6(:,i)]');
    cond_prob6(i) = cond(df(x{:}));
end

%% Problem IX
m = [];
G = [0 0 -9.81];
m(1,1,:) = [60 80 80 40]/1000;
l1 = ones(1,1,n)*0.03;
l2 = l1;
l3(1,1,:) = sqrt( DH(1:end,2).^2 + DH(1:end,3).^2 );

D = massmatrix(T,Jc,l1,l2,l3,m*0);

E = [0 0 -1 0]*r;

[M,~] = EulerLagrange(q,dq,ddq,D,G,m*0,rc,E);

torq = matlabFunction(M);

tau9 = zeros(4,size(Q,2));

for i=1:size(Q,2)
    tau9(:,i) = torq(Q(2,i)',Q(3,i)',Q(4,i)');
end

x = linspace(0,2*pi,37);
plot(x,tau9)

%% Problem X
% this is the torq of the electric motors for the best aproximation done
% in exercise VII

D = massmatrix(T,Jc,l1,l2,l3,m);

[M,L] = EulerLagrange(q,dq,ddq,D,G,m,rc,0);

torq = matlabFunction(M);

t = linspace(-4,4,9*4);

var = [polyval(Z(:,1),t)
       polyval(Z(:,2),t)
       polyval(Z(:,3),t)
       polyval(Z(:,4),t)
       polyval(polyder(Z(:,1)),t)
       polyval(polyder(Z(:,2)),t)
       polyval(polyder(Z(:,3)),t)
       polyval(polyder(Z(:,4)),t)
       polyval(polyder(polyder(Z(:,1))),t)
       polyval(polyder(polyder(Z(:,2))),t)
       polyval(polyder(polyder(Z(:,3))),t)
       polyval(polyder(polyder(Z(:,4))),t)];

var = num2cell(var);

for i=1:size(var,2)
    tau7(:,i) = torq(var{:,i});
end

figure

x = linspace(0,2*pi,size(tau7,2));

for i = 1:4
    plot(x,tau7(i,:),'-',LineWidth=2,Color=color(i,:))
end

hold on

% this is the torq of the electric motors for the trajectory calculated
% in exercise VI

pq = [];
x = [];
t = linspace(0,2,9);

for j=1:size(Z6,3)
    pq1 = [polyval(Z6(:,1,j),t) 
           polyval(Z6(:,2,j),t)  
           polyval(Z6(:,3,j),t) 
           polyval(Z6(:,4,j),t)
           polyval(polyder(Z6(:,1)),t)
           polyval(polyder(Z6(:,2)),t)
           polyval(polyder(Z6(:,3)),t)
           polyval(polyder(Z6(:,4)),t)
           polyval(polyder(polyder(Z6(:,1))),t)
           polyval(polyder(polyder(Z6(:,2))),t)
           polyval(polyder(polyder(Z6(:,3))),t)
           polyval(polyder(polyder(Z6(:,4))),t)];
    pq = [ pq pq1 ];
end

pq = num2cell(pq);

for i=1:size(pq,2)
    tau6(:,i) = torq(pq{:,i});
end

x = linspace(0,2*pi,size(tau6,2));

for i = 1:4
    plot(x,tau6(i,:),'--',LineWidth=2,Color=color(i,:))
end












