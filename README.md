# SmartBall

## Configuration

The SmartBall firmware exposes several compile-time configuration
parameters located in the sensor and networking modules.

- **BUFFER_SIZE** (in `SensorModule.h`)  
  Size of the circular IMU ring buffer.

- **impactThreshold = 35.0f** (in `SensorModule.h`)  
  Default acceleration-magnitude threshold for impact detection.

---

## IMU Sample Format

```c
struct IMUSample {
    float ax, ay, az;
    float gx, gy, gz;
    float temp;
};
```

A single IMU record occupies **28 bytes** (7 floats × 4 bytes).

---

## TCP Command Protocol

The SmartBall device communicates over TCP using plain ASCII commands.  
Every command must terminate with a newline (`\n`).

### Commands

#### `SET THRESH=<value>`

Sets the impact detection threshold (acceleration magnitude in m/s²).

**Parameter**  
`<value>`: Floating-point number.

**Response**  
`OK THRESH=<value>`

---

#### `GET THRESH`

Returns the current impact detection threshold.

**Response**  
`THRESH=<value>`

---

#### `PING`

Checks whether the TCP server is alive (keep-alive).

**Response**  
`PONG`

---

### Error Responses

#### `ERR invalid threshold`

Triggered when the threshold value is not numeric or outside the allowed range.

#### `ERR unknown command`

Triggered when the command is unrecognized or malformed.
