# geom.h

A small and simple geometry library for 3D graphics.

This won't be suitable for use in your projects! I recommend [GLM](https://github.com/g-truc/glm) instead; it has many more useful features (and is inevitably more rigorously tested).

The advantage of geom.h, though, is that it is not heavily templated like GLM, and so provides easier reference.

## Use

geom.h is just the one header; all you need to do is `#include` it.

In one of the files that includes geom.h, you will also need to `#define GEOM_IMPL`. This will include the implementation of functions. This shouldn't be done in more than one file, since it will lead to linker errors from redefinition of functions.

geom.h should work for both C and C++. To use the C++ API (the classes geom::vec3, geom::vec4, etc.), you need to `#define GEOM_CPP`.

## Conventions

All matrices are in column-major order, i.e., geom::mat3(a, b, c, d, ... ) will store those values top-to-bottom, then left-to-right, like this:

```
a d ..
b . .
c .  .
```

This is the same convention OpenGL uses for its matrices.

Quaternions are stored <x, y, z, w>, with the real part last.
'axisAngle's, geom_aa or geom::axisAngle, are just an axis and an angle, stored consecutively in that order.