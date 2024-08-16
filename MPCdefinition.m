%Define the MPC object with Yalmip:

%Sample time for the MPC controller (Please use this name for sample time): 
Ts = 0.1;

%Prediction Horizon (Please use this name prediction horizon)
Hp = 5;

%window Horizon (Please use this name for window horizon)
Hw = 1;

%Control Horizon (Please use this name for control horizon)
Hu = Hp;

Xprev = zeros(2,1);

Uold = zeros(2,1);

%system matrix
    A = [eye(2)] % system matrix

    B = [Ts , 0; 0, Ts] % input matrix

    C = [eye(2)] % output matrix

    D = [0, 0; 0, 0] % feedthrough matrix
    

refht=sdpvar(2,Hp);     % P_{ref}: The window containing the pos-reference
P0=sdpvar(2,1);         % P(k):    The current state
Uprev=sdpvar(2,1);      % U(k-1):  The previous input command.
Udelta=sdpvar(2*Hu,1);    % U_{delta}: The window containing the input command

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PLACE YOUR CODE HERE (START)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PLACE YOUR CODE HERE 

P = objective(A,B,C, Hp, Hu, P0, Uprev, Udelta);

K_m = ones(5, 5);

K = [ones(5,1) tril(K_m)];

K = kron(K, eye(2));

U = K*[Uprev; Udelta];

Q = eye(Hp*2);

R = eye(Hp*2);

Z = P;

Tau = reshape(refht, [Hp*2,1]); % not sure

Cost = (Z - Tau)'*Q*(Z - Tau) + (Udelta)'*R*(Udelta);

%G_vec = ones(Hp, 1)*-0.6;
%G_i = kron(eye(Hp), [0, -1]);

%G = [G_i G_vec];

%now with constaints for x as well
G_vec = zeros(Hp*2, 1);

for i = 1:Hp*2
    if rem(i, 2) == 0
        G_vec(i) = -0.6; % y
    else
        G_vec(i) = -1.2; % x
    end
end

G_i = eye(Hp*2)*-1;

G = [G_i G_vec]

F_vec = ones(Hu*2*2, 1)*-0.22;

F_i = kron(eye(Hp*2), [-1; 1]);

F = [F_i F_vec]

Constraints = [G*[Z;1]<=0, F*[U;1]<=0];

% (END)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% The Yalmip optimizer-object used for simulation and TurtleBot3 control
options = sdpsettings('solver','quadprog');

MPCobj=optimizer(Constraints,Cost,options,{P0,Uprev,refht},{U, P})
% U: u*: The optimal control window
% P: P*: Optimal states in prediction window

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% YOU CAN PLACE YOUR OWN FUNCTIONS HERE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function P = objective(A,B,C, Hp, Hu, X, U, Udelta)

    Alpha = zeros(2*Hp,2);

    Beta = zeros(2*Hp,2); 

    BetaDelta = zeros(2*Hp, 2*Hu); 

    %DeltaU = 
    %Fill Alpha
    for i = 1:2:2*Hp %starter i = 1 indtil Hp = 5
            Alpha(i:i+1,:) = A^(i);
    end

    %Fill Beta
    for i = 1:2:2*Hp %starter i = 1 
        if i == 1
            Beta(i:i+1,:) = B ;           
        else
            for j = 1:i-1 %summeringen i B
                if j == 1
                    Beta(i:i+1,:) = A^(j)*B;
                else
                    Beta(i:i+1,:) = Beta(i:i+1,:)+A^(j)*B;
                end
            end
        end
    end

    
    for i = 1:2:Hu*2 %this forloops over colonms (we take all rows, and I is the column)
        BetaDelta(1:2*Hp,i:i+1) = [zeros(i-1, size(Beta, 2)); Beta(1:end-i+1, :)];
    end

    P = Alpha*X + Beta*U + BetaDelta*Udelta;
    
end
