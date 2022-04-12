function y = jacobian_A(x)
    y = [zeros(3,9) -x(1:3,1:3) zeros(3,9); ...
        wedge_se3([0 0 9.81]) zeros(3,6) -wedge_se3(x(1:3,4))*x(1:3,1:3) -x(1:3,1:3) zeros(3,6); ...
        zeros(3) eye(3) zeros(3) -wedge_se3(x(1:3,5))*x(1:3,1:3) zeros(3,9); ...
        zeros(12,9) zeros(12)]; 
end
