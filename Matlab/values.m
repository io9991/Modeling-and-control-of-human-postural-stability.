%% PARAMETERS OF THE MODEL 

%H = height of the person 
H = 1.60;            %[m]
%mass
m = 52;              %[kg]
%gravity
g = 9.81;            %[N/kg]
%antropometric constant computed by Winter
K = 0.448;

%h = H * 0.567 is the height of COM
h = H * 0.567; 
%moment of inertia of the ankle
Is = m * (K*H).^2;
%viscosity
beta = 3.5; 
Tmus = 0.1;

%% INITIAL CONDITIONS

% theta = 0 (vertical), omega = 0 , tau_a = 0 (muscole relaxed)
x0 = [0 0 0]'; 
u0 = 0;
y0 = x0;
idx = [1];

%% EQUILIBRIUM POINT

%punti di equilibrio
[xeq,ueq] = trim('MathematicalModel',x0,u0,y0,idx,[],[])

%linearizzazione del modello attorno al punto di equilibrio
[A,B,C,D] = linmod('MathematicalModel',xeq,ueq)

%state space representation 
sys = ss(A,B,C,D)

%% EIGENVALUES and EIGENVECTORS

eigA = eig(A)

[V,de] = eig(A)

%% STRUCTURAL PROPERTIES

%controllability
R = ctrb(A,B)

%computation of the rank 
RankR = rank(R)

%observability matrix
O = obsv(A,C)

%Rank of O in order to chek the full observability 
RankO = rank(O)

%% IMPULSE RESPOSE OPEN-LOOP

%creation of plot

figure; 
impulse(sys); 
grid on; 

%% CONTROLLER

K_ack = -acker(A,B, [-2.5 -3 -10])
eigen_ack = eig(A + B*K_ack)
sim("Dynamical System theory\Progettino\Matlab\ControllerFSF.slx")

%impulse response after controller
figure;
sys_cl = ss(A +B*K_ack, B, C, D)
eig_2_1 = eig(A + B*K_ack)
impulse(sys_cl)
grid on;

%if the system in open loop is reachable, the closed loop will also be, but
%the observability could be lost, so check also the obs
R_cl = ctrb(A +B*K_ack,B)
O_cl = obsv(A +B*K_ack,C) 
rank_r_cl = rank(R_cl)
rank_o_cl = rank(O_cl)




%% OBSERVER

%Assume that our matrix C only measure the first state (angle)

C = [1 0 0]; 
D = 0; 

%check the olbs matrix

O = obsv(A,C); 
rO = rank(O); 

%design the observer 
%define the poles 
polesObs = [-15 -20 -30]; 

L = acker(A', C', polesObs);
L = L';

eigO = eig(A-L*C); 

Ao = A - L*C;
Bo = [B L]; 
Co = eye(size(A));
Do = zeros(size(A,1),2); 


modelSim = sim('ControllerObserver.slx'); 