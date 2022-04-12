function y = jacobian_B(x)
    y = [ adjoint_se23(x) zeros(9,12); ...
        zeros(12,9) eye(12)];
end

