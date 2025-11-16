#include "SensorModule.h"

void SensorModule::begin() {
    Wire.begin(6, 7);  // SDA = 6, SCL = 7

    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip!");
        Serial.println("Retrying...");
        delay(200);

        if (!mpu.begin()) {
            Serial.println("[Sensor] MPU6050 not found even after retry!");
            return;
        }
    }

    Serial.println("MPU6050 initialized.");

    // Configure IMU settings
    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
    delay(100);

    head = 0;
    count = 0;
    impactDetected = false;
    consecutiveErrors = 0;

    Serial.print("Impact threshold (m/s^2): ");
    Serial.println(impactThreshold);
}

void SensorModule::pushSample(const IMUSample& s) {
    // Insert sample at (head + count) % BUFFER_SIZE
    int idx = (head + count) % BUFFER_SIZE;
    buffer[idx] = s;

    if (count < BUFFER_SIZE) {
        count++;
    } else {
        // Buffer full: overwrite oldest and move head forward
        head = (head + 1) % BUFFER_SIZE;
    }
}

float SensorModule::computeAccelNorm(const IMUSample& s) const {
    // Compute |a| = sqrt(ax^2 + ay^2 + az^2)
    return sqrtf(s.ax * s.ax + s.ay * s.ay + s.az * s.az);
}

void SensorModule::update() {
    sensors_event_t a, g, temp;
    
    bool ok = mpu.getEvent(&a, &g, &temp);
    if (!ok) {
        consecutiveErrors++;
        Serial.println("[Sensor] getEvent() failed.");

        if (consecutiveErrors >= 3) {
            recoverFromError();
        }
        return;
    }

    IMUSample sample;
    sample.ax   = a.acceleration.x;
    sample.ay   = a.acceleration.y;
    sample.az   = a.acceleration.z;
    sample.gx   = g.gyro.x;
    sample.gy   = g.gyro.y;
    sample.gz   = g.gyro.z;
    sample.temp = temp.temperature;

    // Zombie Mode Detection
    if (isZombie(sample)) {
        consecutiveErrors++;
        Serial.println("[Sensor] Zombie sample detected.");

        if (consecutiveErrors >= 5) {  //Threshold for recovery
            recoverFromError();
        }
        return;
    }

    // Reset error counter on successful read
    consecutiveErrors = 0;

    // Store raw IMU sample into ring buffer
    pushSample(sample);

    // Perform impact detection based on acceleration magnitude
    float aNorm = computeAccelNorm(sample);
    if (aNorm > impactThreshold) {
        impactDetected = true;
    }
}

String SensorModule::exportBufferedData() {
    // Convert all buffered samples into a single text block
    String out;
    // Pre-reserve some capacity to reduce reallocations
    out.reserve(count * 64);

    for (int i = 0; i < count; ++i) {
        int idx = (head + i) % BUFFER_SIZE;
        const IMUSample& s = buffer[idx];

        out += "AX:" + String(s.ax, 2)
            + ",AY:" + String(s.ay, 2)
            + ",AZ:" + String(s.az, 2)
            + ",GX:" + String(s.gx, 2)
            + ",GY:" + String(s.gy, 2)
            + ",GZ:" + String(s.gz, 2)
            + ",T:"  + String(s.temp, 2)
            + "\n";
    }

    // Clear buffer after exporting
    head = 0;
    count = 0;

    return out;
}

String SensorModule::getLatestData() const {
    if (count == 0) return "";

    int idx = (head + count - 1) % BUFFER_SIZE;
    const IMUSample& s = buffer[idx];

    String out = "AX:" + String(s.ax, 2)
               + ",AY:" + String(s.ay, 2)
               + ",AZ:" + String(s.az, 2)
               + ",GX:" + String(s.gx, 2)
               + ",GY:" + String(s.gy, 2)
               + ",GZ:" + String(s.gz, 2)
               + ",T:"  + String(s.temp, 2);

    return out;
}

bool SensorModule::isZombie(const IMUSample& s) const {
    bool allZero =
        s.ax == 0.0f && s.ay == 0.0f && s.az == 0.0f &&
        s.gx == 0.0f && s.gy == 0.0f && s.gz == 0.0f;

    bool tempZombie = fabs(s.temp - 36.53f) < 0.2f;

    return allZero && tempZombie;
}

void SensorModule::recoverFromError() {
    Serial.println("[Sensor] Zombie/error detected — reinitializing MPU6050 (begin())...");

    Wire.end();
    delay(5);
    Wire.begin(6, 7);

    head = 0;
    count = 0;
    consecutiveErrors = 0;

    begin();

    Serial.println("[Sensor] MPU6050 reinitialized via begin().");
}
