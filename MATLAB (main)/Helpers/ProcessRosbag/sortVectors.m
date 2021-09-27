function [flag,t_sorted,occurrences] = sortVectors(t,t1,t2)

% Concatenate vectors and then sort them in ascending order by timestamp
t_sorted = sort([t t1 t2]);
% Create logic vector that maps the sampling times of each sensor
occurrences_t = ismembertol(t_sorted,t,eps); % added eps tolerance
occurrences_t1 = ismembertol(t_sorted,t1,eps);
occurrences_t2 = ismembertol(t_sorted,t2,eps);
%Filter
t_new = t_sorted(occurrences_t);
t1_new = t_sorted(occurrences_t1);
t2_new = t_sorted(occurrences_t2);
if(norm(t-t_new) == 0 && norm(t1-t1_new) == 0 && norm(t2-t2_new) == 0)
    flag = 1;
else
    flag = 0;
end

% Concatenate occurrences
occurrences = [occurrences_t; occurrences_t1; occurrences_t2];

% Correct offset to start at t=0
t_sorted = t_sorted - t_sorted(1); %is this okay?

end

