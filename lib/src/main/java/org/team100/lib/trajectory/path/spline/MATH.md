# math

https://en.wikipedia.org/wiki/Curvature#Curvature_vector

We'll use an arbitrary parameterization over parameter $s$  for each point, $\boldsymbol{P}$ on the curve:

```math
\boldsymbol{P}
=
\boldsymbol{\gamma}(s)
```

The _unit_ tangent vector, $\boldsymbol{T}$, is
the normalized derivative:

```math
\boldsymbol{T}
=
\frac
    {\boldsymbol{\gamma}'(s)}
    {\left\| \boldsymbol{\gamma}'(s) \right\| }
```

The curvature vector, $\boldsymbol{K}$, is the
derivative of the unit tangent vector with respect
to arc length, which is just the second derivative
of position, but only the
component perpendicular to $\boldsymbol{T}$,
so subtract the parallel component:
```math
\boldsymbol{K}
=
  \frac 
    {\boldsymbol{\gamma}''(s)}
    {\left\| \boldsymbol{\gamma}'(s) \right\|^2}
- 
  \boldsymbol{T}
\left (
  \boldsymbol{T}
  \cdot 
  \frac 
    {\boldsymbol{\gamma}''(s)}
    {\left\| \boldsymbol{\gamma}'(s) \right\|^2}
\right)
```

