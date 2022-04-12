function y = wedge_se23(x)
    y = [wedge_se3(x(1:3)) x(4:6) x(7:9); ....
        zeros(2,5)];
end

