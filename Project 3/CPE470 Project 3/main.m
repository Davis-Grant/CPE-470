%Grant Davis
%CPE470
%Project 3: Consensus Filters for a Multi-robot System

clc
close all
clear

%code by Jim La
num_nodes = 50; %number of nodes
r = .4; %communication range <==========================Change r here
n = 2; % number of dimensions
delta_t_update = .008; 
nodes = rand(num_nodes, n); %uncomment to generate new node set
%save('Nodes.txt', 'nodes');
%nodes = importdata('Nodes.txt');
F = 50;
%nodes = nodes * 4;
%Add measurement for each node
M = F * ones(num_nodes, 1) + 1 * randn(num_nodes, 1);
m_i = M; % Save initial measurement
[Nei_agent, A] = findneighbors(nodes, r, n, delta_t_update);

cv = .01;
W = zeros(num_nodes, num_nodes);%Weights initialized to 0
% generate design factor
c1_w = ((2 * cv) / (r * r * (num_nodes - 1))).* rand(1,1);
c2_w = (cv / (r * r)) .*rand(1,1);
%generate V
V = zeros(num_nodes, 1);
q_bar = mean(nodes);
%Q = transpose(q_bar);
for i = 1:num_nodes
    e_d = norm(nodes(i,:) - q_bar);
    V(i) = (e_d + cv) / (r * r);
end
%Select Weight Design
str = 'CPE470 Project 3:';
str = [str newline '1. Weight Design 1'];
str = [str newline  '2. Weight Design 2'];
str = [str newline '3. Metropolis Design'];
str = [str newline '4. Maximum Degree Design'];
str = [str newline 'Choice : '];
choice = input(str);
switch choice
    case 1 % Weight Design 1 set L to 500
        for i = 1:num_nodes % for each node
            for j = 1:num_nodes %for each weight
                for k = 1:size(Nei_agent{i})%for each neighbor
                    TEST = Nei_agent{i}(k);
                    if j == TEST
                        if i ~= j
                            W(i,j) = c1_w / (V(i) + V(j));
                        end
                    end
                end
            end
            for j = 1:num_nodes % for each weight where i equals j
                if i == j
                    W(i,j) = 1 - sum(W(i));
                end
            end
        end   
    case 2 % Weight Design 2 set r to .5 and L to 50
        for i = 1:num_nodes %for each node
             W(i,i) = c2_w / V(i);
        end
        for i = 1:num_nodes % for each node
            for j = 1:num_nodes %for each weight
                m = size(Nei_agent{i});
                for k = 1:size(Nei_agent{i})%for each neighbor
                    TEST = Nei_agent{i}(k);
                    if j == TEST
                        if i ~= j
                            W(i,j) = (1 - W(i,i)) / m(:,1);
                        end
                    end
                end
            end
        end
    case 3 % Metropolis Design set L to 15
        for i = 1:num_nodes %for each node
            for j = 1:num_nodes %for each weight
                for k = 1:size(Nei_agent{i})%for each neighbor
                    TEST = Nei_agent{i}(k);
                    if j == TEST
                        m = size(Nei_agent{i});
                        n = size(Nei_agent{j});
                        W(i,j) = 1 / (max(m(:,1), n(:,1)) + 1);
                    end
                end
            end
            for j = 1:num_nodes %for each weight
                if i == j
                    
                    W(i,j) = 1 - sum(W(i,Nei_agent{i}));
                end
            end
        end
    case 4 % Maximum Degree Design set L to 50
        for i = 1:num_nodes %for each node
            for j = 1:num_nodes%for each weight
                if i == j
                    m = size(Nei_agent{i});
                    W(i,j) = 1 - (m(:,1) / num_nodes);
                end
            end
            for j = 1:num_nodes %for each weight
                for k = 1:size(Nei_agent{i})%for each neighbor
                    TEST = Nei_agent{i}(k);
                    if j == TEST
                        W(i,j) = 1 /num_nodes;
                    end
                end
            end
        end
    otherwise % Terminate program
            disp('No Design selected\n');
            quit;
end

L = 100;%<=====================================Change L here
Val = zeros(9,1);
X = zeros(L, num_nodes);
X(1,:) = m_i;% first iteration
for j = 2:L %Weighted Average Consensus
    for i = 1:num_nodes
        temp1 = transpose(X((j - 1), Nei_agent{i}));
        temp2 = W(i,Nei_agent{i});
        Val = temp2 * temp1;
        X(j,i) = W(i,i) * X((j - 1),i) + Val;
    end
end

j = L;
C = zeros(L,num_nodes);

for i = 1:L %calculating convergence C
    for k = 1:num_nodes
        C(i,k) = (X(j,k) - X(1,k));
        if floor(k/2) == k/2
            C(i,k) = C(i,k) * -1;%flip every other node
        end 
    end
    j = j - 1;
end
%plotting node network
figure(1), plot(nodes(:,1), nodes(:,2), 'm>', 'Linewidth', .2, ...
                    'MarkerEdgeColor', 'm', ...
                    'MarkerSize',5)
hold on
for i = 1:num_nodes
    %Line the neighbors together
    tmp = nodes(Nei_agent{i},:);
    for j = 1:size(nodes(Nei_agent{i},1))
        line([nodes(i,1), tmp(j,1)],[nodes(i,2),tmp(j,2)])
    end
end

%plotting Convergence
figure(2), plot(C)
%plotting final vs. intial measurements
figure(3), plot(C(1,:), '--ro')
hold on
plot(C(L,:), '--bs')