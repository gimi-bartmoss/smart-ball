## smart ball configuration

1. BUFFER_SIZE (SensorModule.h)
2. impactThreshold = 35.0f (SensorNOdule.h)

## IMUSample

```c
struct IMUSample {
    float ax, ay, az;
    float gx, gy, gz;
    float temp;
};
```

28 bytes for one record