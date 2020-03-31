% close all;
figure(1); 
Voltage = ScopeData{2}.Values.Data;
Current = ScopeData{1}.Values.Data;

first = 1;
for i=2:length(Current)
    if Current(i) < 0
        % no assignin of "last" here, because we need "first" for the next
        % loop. The first current elements can be erroneous, so we have to
        % do the second loop (starting from "first") to find "last".
        break;
    end
    if Current(i) > Current(i-1)
        first = i;
        % no break here!
    end
end
if first > 10
    first = 10;
end

last = length(Current);
for i=first:length(Current)
    if Current(i) < 0
        last = i;
        break
    end
end


V = Voltage(first:last);
I = Current(first:last);
P = V.*I;
[Pmp, idx_mpp] = max(P);
Vmp = V(idx_mpp);
Imp = I(idx_mpp);
Voc = V(end);
Isc = I(1);

subplot(2,1,1);
plot(V,I); grid on; hold on;
plot(Vmp,Imp,'ro');
xlabel('Voltage (V)'); ylabel('Current (A)');
title(['I-V Characteristics, V_m_p = ' num2str(Vmp*1000) ' mV']);
% title({['V_o_c = ' num2str(V(end)*1000) ' mV'], ...
%     ['I_s_c = ' num2str(I(1)*1000) ' mA']});

subplot(2,1,2);
plot(V,P); grid on; hold on;
plot(Vmp,Pmp,'ro');
xlabel('Voltage (V)'); ylabel('Power (W)');
title(['P-V Characteristics, V_m_p = ' num2str(Vmp*1000) ' mV']);
% title({['V_m_p_p = ' num2str(Vmp*1000) ' mV'],...
%     ['I_m_p = ' num2str(Imp*1000) ' mA']});

Voc = Voc*1000;
Isc = Isc*1000;
Vmp = Vmp*1000;
Imp = Imp*1000;
fprintf('Voc = %.2f\nIsc = %.2f\nVmp = %.2f\nImp = %.2f\n',Voc,Isc,Vmp,Imp);

