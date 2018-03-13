function inc_test()

persistent x;
x = 0;

for n = 0:10
    a = increment(x)
end

end

function out = increment(x)

out = x + 2;
x = x + 1;

end