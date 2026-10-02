function yes = find_consecutive(logical_array, N)
    count = 0;
    yes = false;
    for i = 1:length(logical_array)
        if logical_array(i)
            count = count + 1;
            if count >= N
                yes = true;
                return
            end
        else
            count = 0;
        end
    end
end
