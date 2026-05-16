# CPU OBJ/MTL Renderer

A small standalone C11 ray tracer that loads Wavefront `.obj` meshes plus `.mtl` materials and renders a PPM image entirely on the CPU.

## Features

- Wavefront OBJ loader: `v`, `vn`, `f`, `mtllib`, and `usemtl`.
- MTL loader: `newmtl`, `Kd`, `Ks`, `Ns`, and a custom optional `refl`/`reflectivity` scalar.
- Triangle ray tracing with a BVH acceleration structure.
- Diffuse + Phong highlights, hard shadows, recursive mirror reflections, and sky fallback.
- Supersampling anti-aliasing.
- No third-party dependencies; only a C compiler and `libm` are required.

## Build

```sh
make -C cpu_obj_renderer
```

## Run

```sh
./cpu_obj_renderer/objrt cpu_obj_renderer/examples/mirror_room.obj output.ppm 800 450 --samples 4 --bounces 3
```

Open `output.ppm` with an image viewer that supports PPM, or convert it with ImageMagick:

```sh
convert output.ppm output.png
```

## Usage

```text
objrt scene.obj output.ppm [width height] [--samples N] [--bounces N]
```

The camera is currently a fixed view suitable for the bundled example: position `(0, 1.2, 4.2)`, looking at `(0, 0.7, 0)`, with a 55-degree vertical field of view. This keeps the renderer compact while leaving the core OBJ/MTL/BVH/reflection code easy to extend.

## Material reflection

Reflection can be controlled in two ways:

1. Add a custom scalar to an MTL material, e.g. `refl 0.75` or `reflectivity 0.75`.
2. If no custom scalar is present, the renderer derives a modest reflectivity from the brightest `Ks` channel.

## Notes

- Faces with more than three vertices are triangulated as a fan.
- Texture coordinates are parsed only as part of face syntax and are ignored for shading.
- BVH splitting uses the widest centroid axis and median partitioning for fast build times.
