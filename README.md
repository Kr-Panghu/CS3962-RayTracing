## Script Running Guide

Compilation:

1. run `mkdir build`
2. run `cd ./build`
3. run `cmake .. && make`
4. run `mv RayTracing ..`

Running options:

1. run `./RayTracing check`, if you want to run in check mode
2. run `./RayTracing`, if you want to run with original BVH structure (**Split by count**, which is the functionality implemented by the original code)
3. run `/RayTracing SAH`, if you want to run my proposed BVH structure (**Split by SAH**)

---

## Notes

This is the repo for CS3962 计算机图形学 course project at Shanghai Jiao Tong University.