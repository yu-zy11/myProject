import Pkg;
Pkg.activate(@__DIR__);
Pkg.instantiate();

using LinearAlgebra
using ForwardDiff
using Plots
using Rotations
using Debugger
msg = "Hello World"
println(msg)
## q
a = 1
bb = 1
@bp

## 
b = [2 3 3 4 5]
print

##
x = 1:10;
y = rand(10); # These are the plotting data
plot(x, y, label="my label")