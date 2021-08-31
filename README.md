# Newsched Wifi Module

This module implements parts of [gr-ieee802-11](https://github.com/bastibl/gr-ieee802-11) for [newsched](https://github.com/gnuradio/newsched) as an Out of Tree Module (OOT)

Also, an out of tree "scheduler" is included - a rudimentary threadpool for parallelizing the decode chain

## Dependencies

```
meson
ninja
CUDA (>=10.1) (for cuda implementations)
[cusp](https://github.com/gnuradio/cusp) (if CUDA enabled)
```

## Building

For Ubuntu 20.04 with CUDA installed via `apt-get`
```
CUDA_ROOT=/usr/ meson setup --debug build --buildtype=debugoptimized --prefix=[PREFIX] -Denable_cuda=true
cd build
ninja
```
where `[PREFIX]` is the newsched installation prefix

