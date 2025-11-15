# smart ball configuration

1. BUFFER_SIZE (SensorModule.h)
2. impactThreshold = 35.0f (SensorNOdule.h)

# IMUSample

```c
struct IMUSample {
    float ax, ay, az;
    float gx, gy, gz;
    float temp;
};
```

28 bytes for one record

# TCP Command Protocol

The SmartBall device communicates over TCP using plain ASCII text
commands. All commands must be terminated with a newline character `\n`.

## Commands

### 1. `SET THRESH=<value>`

Sets the impact detection threshold (acceleration magnitude in m/s²).

**Parameter**\
`<value>`: A floating-point number.

**Response**

    OK THRESH=<value>

------------------------------------------------------------------------

### 2. `GET THRESH`

Returns the current impact detection threshold.

**Response**

    THRESH=<value>

------------------------------------------------------------------------

### 3. `PING`

Checks whether the TCP server is alive (keep-alive).

**Response**

    PONG

------------------------------------------------------------------------

## Error Responses

### `ERR invalid threshold`

-   The threshold value is not numeric or outside the allowed range.

### `ERR unknown command`

-   The command is unrecognized or malformed.
