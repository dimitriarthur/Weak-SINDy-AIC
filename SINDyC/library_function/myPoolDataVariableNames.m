function yout = myPoolDataVariableNames(yin,ahat,nVars,polyorder,useFourier)
n = size(yin,1);

ind = 1;
% poly order 0
yout{ind,1} = ['1'];
ind = ind+1;

% poly order 1
for i=1:nVars
    yout(ind,1) = yin(i);
    ind = ind+1;
end

if(polyorder>=2)
    % poly order 2
    for i=1:nVars
        for j=i:nVars
            yout{ind,1} = [yin{i},yin{j}];
            ind = ind+1;
        end
    end
end

if(polyorder>=3)
    % poly order 3
    for i=1:nVars
        for j=i:nVars
            for k=j:nVars
                yout{ind,1} = [yin{i},yin{j},yin{k}];
                ind = ind+1;
            end
        end
    end
end

if(useFourier)
    for i=1:size(yin,2)
        yout{ind,1} = ['sin(',yin{i},')'];
        ind = ind + 1;
        yout{ind,1} = ['cos(',yin{i},')'];
        ind = ind + 1;
end

end
