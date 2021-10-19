% generates the library function for computing sparse regression
function yout= myPoolData(yin,nVars,polyorder,useFourier) 
%yin - vector concat; nVars - number of states; polyorder - degree;
%useFourier - flag for trig terms.
            
    n = size(yin,1);
    ind = 1;
    % poly order 0
    yout(:,ind) = ones(n,1);
    ind = ind+1;
    % poly order 1
    for i=1:nVars
        yout(:,ind) = yin(:,i);
        ind = ind+1;
    end
    
    if(polyorder>=2)
        % poly order 2
        for i=1:nVars
            for j=i:nVars
                yout(:,ind) = yin(:,i).*yin(:,j);
                ind = ind+1;
            end
        end
    end
    if(polyorder>=3)
        % poly order 3
        for i=1:nVars
            for j=i:nVars
                for k=j:nVars
                    yout(:,ind) = yin(:,i).*yin(:,j).*yin(:,k);
                    ind = ind+1;
                end
            end
        end
    end
    if(useFourier)
            yout = [yout sin(yin) cos(yin)];
    end
end