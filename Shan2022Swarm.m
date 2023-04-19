close all
clear
clc
rng(0)
% Narray=[5     7       9       11      13      15      17      19]
% Earray [8.33  16.14   18.60   24.41   29.13   33.18   30.25   34.85] %cm
%% Deploy Nodes
N=5; %number of nodes
Dmax=20;
locx=Dmax*rand(N,1);
locy=Dmax*rand(N,1);
nodes=cell(N,1);
distance=zeros(N,N);
for n=1:N
    for m=1:N
        distance(n,m)=sqrt((locx(n)-locx(m))^2+(locy(n)-locy(m))^2);
    end
end
% flight times in ns (distance/speed * 10^9)
flight_times = distance / (3 * 1e8) * 1e9;
% max flight time (dmax*sqrt(2))/ speed of light. 20sqrt(2)/ 3e8 = 94.5 ns

%% initialize nodes table and info
for m=1:N %nodes
    info=cell(N,1); %data structures
    for n=1:N
        info{n}.Tp=0;
        info{n}.Rp=0;
        info{n}.Tr=0;
        info{n}.Rr=0;
        info{n}.Tf=0;
        info{n}.Rf=0;
        info{n}.Re=0;
        info{n}.distance=0;
        info{n}.RrSeqNum=0;
        info{n}.RfSeqNum=0;        
    end
    
    stamps=cell(3,1);
    for ii = 1:10 % store 10 timestamps
        stamps{ii}.sourceId = 0;
        stamps{ii}.seqNum = 0;
        stamps{ii}.recTime = 0;
    end
    
    nodes{m}.table = info;
    nodes{m}.timeStamps = stamps;
    
    nodes{m}.seqNum = 0;
    nodes{m}.prevMessTime = 0;
    nodes{m}.xPos = locx(m);
    nodes{m}.yPos = locy(m);
    
end
pRange = 0.05:0.05:0.5;
% pRange = 1/N;
T = 10000;
mse=zeros(1, length(pRange)); % mse distance error
transmitting=zeros(N,T);
% The clock drift error of the Qorvo DW1000 module according to the
%datasheet, the module's integrated clock has a typical accuracy of
%±10 ppm (parts per million) at 25°C, which translates to about %±0.01%
%deviation from the ideal clock frequency.

% clock drift, a random value from the normal distribution with 0 mean
% 10 ppm variance. approx 1 nanoseonds missmatch
clockDrift = (normrnd(0,10e-6,[1,5]));
%% farkli p degerleri icin simulasyon
for pIndex=1:length(pRange)
    p = pRange(pIndex);
    collisionCntr = 0;
    idleCntr = 0;
    lossCntr = 0;
    distCalculationCntr = 0;
    distanceError = 0;

    %% run
    for t=1:T
        transmitting(:,t)=rand(N,1)<p; %ileti yapan dugumler
        
        %% collisions
        %1: simple
        %2: SNR based
        % determine receviers
        %which receivers received which transmitters signal?
        if sum(transmitting(:,t)) == 0 %idle
            idleCntr = idleCntr+1;
            continue;
        elseif sum(transmitting(:,t)) > 1 %collision
            collisionCntr = collisionCntr + 1;
            continue;
        end
        idx = find(transmitting(:,t) == 1);
        
        %% SentPacket from idx
        % messPacket contains following
        idx;
        nodes{idx}.seqNum;
        nodes{idx}.prevMessTime;
        timestepsSeries = nodes{idx}.timeStamps;
        
        %% update Tf in all neighbors table
        for ii=1:N %idx nolu (mesajgönderen) düğümün ii no'lu (komşu) düğümleri zaman damgası
            if(idx == ii)
                continue;
            end
            nodes{idx}.table{ii}.Tf = t;
        end
        
        %% process the received signals for each node
        % update data structures if packet is not lost
        % packet loss for indivual node with probalility 1/1000
        lossVector = rand(1,N)<0.001;
        
        for n=1:N
            if(idx == n)
                continue
            elseif(lossVector(n) == 1)
                lossCntr = lossCntr + 1;
                continue
            end
            
            %% recieve edince Mx ekle
            % todo: if full handle this
            for ii=1:10
                if(nodes{n}.timeStamps{ii}.recTime == 0 && nodes{n}.timeStamps{ii}.seqNum == 0)
                    % if empty
                    nodes{n}.timeStamps{ii}.sourceId = idx;
                    nodes{n}.timeStamps{ii}.seqNum = nodes{idx}.seqNum;
                    nodes{n}.timeStamps{ii}.recTime = t + flight_times(idx, n); % maybe mean?
                    break;
                end
            end
            %% add recieved data (MX) to tables
            for ii=1:10
                if (nodes{idx}.timeStamps{ii}.sourceId == n)
                    nodes{n}.table{idx}.Rf = nodes{idx}.timeStamps{ii}.recTime;
                    nodes{n}.table{idx}.RfSeqNum = nodes{idx}.timeStamps{ii}.seqNum;
                end
            end
            
            nodes{n}.table{idx}.Tr = nodes{idx}.prevMessTime; % nodes{idx}
            nodes{n}.table{idx}.Re = t + flight_times(idx, n);
            
            %% if enough timestamps are collected, calculate distance.
            
            % check packet missmatch
            % case2: Rf is empty other is full.-> Rr = Re
            % case3: Tr ve Rr seq uyumsuz -> Rp = Rf, TP = Tf, Rr = Re,
            if(nodes{n}.table{idx}.Rf == 0)
                nodes{n}.table{idx}.Rr = nodes{n}.table{idx}.Re;
                nodes{n}.table{idx}.RrSeqNum = nodes{idx}.seqNum;
                
                nodes{n}.table{idx}.Rf = 0;
                nodes{n}.table{idx}.RfSeqNum=0;
                
                nodes{n}.table{idx}.Tf = 0;
                nodes{n}.table{idx}.Re = 0;
                nodes{n}.table{idx}.Tr = 0;
                
            elseif(nodes{idx}.seqNum - nodes{n}.table{idx}.RrSeqNum > 1)
                nodes{n}.table{idx}.Rr = nodes{n}.table{idx}.Re;
                nodes{n}.table{idx}.RrSeqNum = nodes{idx}.seqNum;
                nodes{n}.table{idx}.Rp = nodes{n}.table{idx}.Rf;
                nodes{n}.table{idx}.Tp = nodes{n}.table{idx}.Tf;
                
                nodes{n}.table{idx}.Rf = 0;
                nodes{n}.table{idx}.RfSeqNum = 0;
                nodes{n}.table{idx}.Tf = 0;
                nodes{n}.table{idx}.Re = 0;
                nodes{n}.table{idx}.Tr = 0;
                
            elseif (nodes{n}.seqNum - nodes{n}.table{idx}.RfSeqNum > 1)
                nodes{n}.table{idx}.Rf = 0;
                nodes{n}.table{idx}.RfSeqNum = 0;
                
            end
            
            % calculate distance
            if ((nodes{n}.table{idx}.Rp ~= 0) && (nodes{n}.table{idx}.Tp ~= 0) && (nodes{n}.table{idx}.Tr ~= 0) && (nodes{n}.table{idx}.Rr ~= 0) && (nodes{n}.table{idx}.Rf ~= 0) && (nodes{n}.table{idx}.Tf ~= 0) && (nodes{n}.table{idx}.Re ~= 0))
                ad = nodes{n}.table{idx}.Rr - nodes{n}.table{idx}.Tp;
                bp = nodes{n}.table{idx}.Tr - nodes{n}.table{idx}.Rp;
                bd = nodes{n}.table{idx}.Rf - nodes{n}.table{idx}.Tr;
                ap = nodes{n}.table{idx}.Tf - nodes{n}.table{idx}.Rr;
                
                tof = (ad * bd - ap * bp) / (ad + bd + ap + bp);
                e1 = clockDrift(n);
                e2 = clockDrift(idx);
                tof_e = (e1 + e2 + 2 * e1 * e2) / (2 + e1 + e2) * tof + tof;
                nodes{n}.table{idx}.distance = tof_e * 3e-1;
                
                distanceError = distanceError + (nodes{n}.table{idx}.distance - distance(n,idx))^2;
                distCalculationCntr = distCalculationCntr + 1;
                
                nodes{n}.table{idx}.Rr = nodes{n}.table{idx}.Re;
                nodes{n}.table{idx}.RrSeqNum = nodes{idx}.seqNum;
                nodes{n}.table{idx}.Rp = nodes{n}.table{idx}.Rf;
                nodes{n}.table{idx}.Tp = nodes{n}.table{idx}.Tf;
                
                nodes{n}.table{idx}.Rf = 0;
                nodes{n}.table{idx}.Tf = 0;
                nodes{n}.table{idx}.Re = 0;
                nodes{n}.table{idx}.Tr = 0;
            end
            
        end
        %% after sending a packet clear Mx and update seqNum and prevMessTime
        for ii=1:10
            nodes{idx}.timeStamps{ii}.sourceId = 0;
            nodes{idx}.timeStamps{ii}.seqNum = 0;
            nodes{idx}.timeStamps{ii}.recTime =  0;
        end
        nodes{idx}.seqNum = nodes{idx}.seqNum + 1;
        nodes{idx}.prevMessTime = t;
        
    end
    
    %% results
    % for n=1:N
    %     for m=1:N
    %         fprintf("distance %d to %d is %.2f, real dist is %.2f\n", n, m, nodes{n}.table{m}.distance, distance(n,m))
    %     end
    % end
    
    meanError = sqrt(distanceError / distCalculationCntr);
    mse(pIndex) = meanError;
    
    %     fprintf("\nProbability that a node can transmit packet (p) is %.2f\n", p);
    %     fprintf("Collision occured in %d out of %d\n", collisionCntr, T);
    %     fprintf("Idle channel occured in %d out of %d\n", idleCntr, T);
    fprintf("Packet loss in %d out of %d\n", lossCntr, T);
    fprintf("Percantage of time that is avaliable for succesfull packet transmisson %.2f\n",100-100*(collisionCntr + idleCntr + lossCntr)/T );
    fprintf("Number of distance calculation made is %d\n", distCalculationCntr);
    fprintf("Average mean square error is %.2f meter (%.2f cm) \n\n", meanError, meanError * 100);
    clockDrift
    
end

mse = mse * 100 % m to cm

figure(1)
plot(pRange, mse, 'LineWidth', 1);
xlabel('p');
ylabel('Distance Error (cm)');
title('Probability - Distance Error Graph');
