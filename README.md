# geom.h

A small and simple geometry library for 3D graphics.

This won't be suitable for use in your projects! I recommend [GLM](https://github.com/g-truc/glm) instead; it has many more useful features (and is inevitably more rigorously tested).

The advantage of geom.h, though, is that it is not heavily templated like GLM, and so provides easier reference.

## Use

geom.h is just the one header. All you need to do is:
`#include "geom.h"` for C files that need it

```
#define GEOM_CPP
#include "geom.h"
``` for files that use the C++ API

```
#define GEOM_IMPL
#include "geom.h"
#undef GEOM_IMPL
``` from ONE file exactly (again, with the extra `#define GEOM_CPP` if you use the C++ API)

To compile C, you will need the flags `-std=c99` and `-lm`.

## Conventions

All matrices are in column-major order, i.e., geom::mat3(a, b, c, d, ... ) will store those values top-to-bottom, then left-to-right, like this:

```
a d ..
b . .
c .  .
```

This is the same convention OpenGL uses for its matrices.

Quaternions are stored <x, y, z, w>, i.e., the three imaginary parts first, the real part last.
'axisAngle's, geom_aa or geom::axisAngle, are just an axis and an angle, stored consecutively in that order.