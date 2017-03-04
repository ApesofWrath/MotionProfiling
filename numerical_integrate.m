function output = numerical_integrate(input, dt, initial_condition)
    output(1) = initial_condition + input(1) * dt;
    for i = 2:length(input)
        output(i,1) = output(i-1) + input(i) * dt;
    end
end
