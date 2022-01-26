function u0 = horizonte(u)
    u0 = [u(:,2:size(u,2)) u(:,size(u,2))];
end

