# mvp_localization
sensor fusion based on gtsam for MVP framework


- GTSAM 4.2

```
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DGTSAM_BUILD_TESTS=OFF \
  -DGTSAM_BUILD_EXAMPLES=OFF \
  -DGTSAM_WITH_TBB=ON \
  -DGTSAM_USE_SYSTEM_EIGEN=ON
```

```
sudo make install
```