// IMU
// Raw I2C driver for MPU6050-compatible sensors
//
// Artificially-generated code in this file

#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "constants.h"

// Handles the bootleg-ish IMU6080 we have been given
class IMU {
    public:

    // degrees per second
    enum class GyroScale : uint8_t {
        DPS_250  = 0x00, // 131.0 LSB per °/s
        DPS_500  = 0x08, // 65.5  LSB per °/s
        DPS_1000 = 0x10, // 32.8  LSB per °/s
        DPS_2000 = 0x18  // 16.4  LSB per °/s
    };

    // 9.8m/s²
    enum class AccelScale : uint8_t {
        G_2  = 0x00, // 16384 LSB/g
        G_4  = 0x08, // 8192  LSB/g
        G_8  = 0x10, // 4096  LSB/g
        G_16 = 0x18  // 2048  LSB/g
    };

    enum class LowPassFrequency : uint8_t {
        HZ_260 = 0,
        HZ_184 = 1,
        HZ_94  = 2,
        HZ_44  = 3,
        HZ_21  = 4,
        HZ_10  = 5,
        HZ_5   = 6
    };

    // A run of Z-gyro samples taken off the FIFO, summed in raw LSB.
    //
    // The sum and not the mean, deliberately. Every sample in it covers
    // exactly samplePeriod() seconds, so the batch integrates to
    //
    //     dtheta = sum * gyroRadPerLsb() * samplePeriod()
    //
    // whatever the caller's loop was doing while they accumulated. That is the
    // whole reason for reading the FIFO rather than the data registers: the
    // sensor keeps sampling into its own buffer through a 23 ms OLED transfer,
    // so the samples taken during one are still here to be drained afterwards
    // instead of being lost between two reads.
    struct GyroBatch {
        int32_t sum;    // raw LSB, zero-rate offset NOT removed
        uint16_t count; // samples in sum
        bool lost;      // samples went missing; see drainGyroZ()
    };

    IMU(uint8_t address = IMU_ADDRESS) : address(address) {}

    // Sane defaults
    bool init(
        GyroScale gyro        = GyroScale::DPS_1000,
        AccelScale accel      = AccelScale::G_4,
        LowPassFrequency dlpf = LowPassFrequency::HZ_44
    ) {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() != 0) return false;

        // Wake, then move off the internal 8 MHz RC oscillator and onto the
        // PLL locked to the X gyro, which is what the datasheet recommends.
        // Not cosmetic here: drainGyroZ() below converts a sample count into a
        // time by multiplying by an assumed period, so however far the sensor's
        // clock wanders is a scale error on every angle this class reports.
        // The RC oscillator wanders with temperature; the PLL reference does
        // not, to anything like the same degree.
        writeReg(PWR_MGMT_1, PWR_CLKSEL_INTERNAL);
        delay(IMU_CLOCK_SETTLE_MS);
        writeReg(PWR_MGMT_1, PWR_CLKSEL_PLL_XGYRO);
        delay(IMU_CLOCK_SETTLE_MS);

        writeReg(GYRO_CONFIG, static_cast<uint8_t>(gyro));
        writeReg(ACCEL_CONFIG, static_cast<uint8_t>(accel));
        writeReg(CONFIG, static_cast<uint8_t>(dlpf));
        writeReg(SMPLRT_DIV, IMU_SAMPLE_RATE_DIVIDER);

        gyroscopicSensitivity    = gyroSensitivity(gyro);
        accelerometerSensitivity = accelSensitivity(accel);
        gyro_rad_per_lsb = static_cast<float>(DEG_TO_RAD) / gyroscopicSensitivity;

        // The rate SMPLRT_DIV divides is 8 kHz only when the DLPF is bypassed
        // altogether; any real filter setting drops it to 1 kHz. Worth deriving
        // rather than assuming, because getting it wrong is a silent factor of
        // eight on the heading.
        const float base = (dlpf == LowPassFrequency::HZ_260) ? 8000.0f : 1000.0f;
        sample_period    = (1.0f + static_cast<float>(IMU_SAMPLE_RATE_DIVIDER)) / base;

        // Z gyro only. The other five axes and the temperature would fill the
        // FIFO six times as fast, and nothing integrates them -- which is what
        // buys the depth IMU_SAMPLE_RATE_DIVIDER is asserted against.
        writeReg(FIFO_EN, FIFO_EN_ZG);
        writeReg(USER_CTRL, USER_CTRL_FIFO_ENABLE);
        fifo_ready = true;
        resetFifo();

        return true;
    }

    // Seconds covered by one FIFO sample. Fixed by SMPLRT_DIV and the DLPF, and
    // independent of anything the control loop does.
    float samplePeriod() const {
        return sample_period;
    }

    // Radians per second per raw LSB, from the configured full-scale range.
    // Precomputed because DEG_TO_RAD is a double literal and this part has no
    // double-precision unit -- deriving it here would put a soft-float divide
    // in the integration path.
    float gyroRadPerLsb() const {
        return gyro_rad_per_lsb;
    }

    // Empties the FIFO and returns what was in it.
    //
    // The `lost` flag means samples are missing and the batch is empty: either
    // the FIFO overflowed and dropped them off the front, or it came back with
    // an odd byte count. The second is the more dangerous of the two and the
    // reason for the check -- a read pointer that is not on a sample boundary
    // makes every subsequent pair straddle two samples, which decodes as
    // large values of arbitrary sign rather than as anything recognisably
    // wrong. Resetting and admitting the gap beats integrating that.
    GyroBatch drainGyroZ() {
        GyroBatch batch = {0, 0, false};
        if (!fifo_ready) return batch;

        uint8_t status = 0;
        if (!readByte(INT_STATUS, status)) return batch;

        uint16_t count = 0;
        if (!readUint16(FIFO_COUNTH, count)) return batch;

        // A full FIFO is one sample away from having dropped one, so it is
        // treated as though it already had.
        if ((status & INT_STATUS_FIFO_OFLOW) || count >= IMU_FIFO_CAPACITY_BYTES || (count & 1u)) {
            resetFifo();
            ++fifo_losses;
            batch.lost = true;
            return batch;
        }

        while (count >= 2) {
            const uint8_t want =
                (count > IMU_FIFO_CHUNK_BYTES) ? IMU_FIFO_CHUNK_BYTES : static_cast<uint8_t>(count);

            Wire.beginTransmission(address);
            Wire.write(FIFO_R_W);
            if (Wire.endTransmission(false) != 0) {
                ++read_failures;
                break;
            }
            Wire.requestFrom(address, want);

            uint8_t got = 0;
            while (Wire.available() >= 2) {
                const uint8_t hi = static_cast<uint8_t>(Wire.read());
                const uint8_t lo = static_cast<uint8_t>(Wire.read());
                batch.sum += static_cast<int16_t>(static_cast<uint16_t>(hi << 8) | lo);
                ++batch.count;
                got = static_cast<uint8_t>(got + 2);
            }

            // A short read is survivable: the FIFO's read pointer only advanced
            // by what actually came out, so the next drain resumes cleanly. A
            // single trailing byte is not, for the alignment reason above.
            if (Wire.available() != 0) {
                while (Wire.available()) Wire.read();
                ++read_failures;
                resetFifo();
                ++fifo_losses;
                batch.lost  = true;
                batch.sum   = 0;
                batch.count = 0;
                return batch;
            }

            if (got == 0) {
                ++read_failures;
                break;
            }
            count = static_cast<uint16_t>(count - got);
        }

        return batch;
    }

    // Discards whatever is buffered and restarts the stream on a sample
    // boundary. FIFO_RESET self-clears, and has to be written with the enable
    // bit still set or the FIFO comes back disabled.
    void resetFifo() {
        if (!fifo_ready) return;
        writeReg(USER_CTRL, USER_CTRL_FIFO_ENABLE | USER_CTRL_FIFO_RESET);
    }

    // Direct register reads, kept for bring-up and diagnostics. Heading no
    // longer comes through here -- see drainGyroZ().
    float gyroX() {
        return toRadPerSec(gyroXAvg.push(readWord(GYRO_XOUT_H)));
    }

    float gyroY() {
        return toRadPerSec(gyroYAvg.push(readWord(GYRO_YOUT_H)));
    }

    float gyroZ() {
        return toRadPerSec(gyroZAvg.push(readWord(GYRO_ZOUT_H)));
    }

    float accelX() {
        return toMPerSec2(accelXAvg.push(readWord(ACCEL_XOUT_H)));
    }

    float accelY() {
        return toMPerSec2(accelYAvg.push(readWord(ACCEL_YOUT_H)));
    }

    float accelZ() {
        return toMPerSec2(accelZAvg.push(readWord(ACCEL_ZOUT_H)));
    }

    // DIAGNOSTIC: failed reads since boot. A direct read failure returns 0,
    // which the rolling average cannot tell from "not rotating", so this
    // counter is the only way to see one happen.
    unsigned long failures() const {
        return read_failures;
    }

    // DIAGNOSTIC: FIFO resets forced by an overflow or a lost alignment, each
    // of which threw away a batch of heading. Should stay at zero.
    unsigned long fifoLosses() const {
        return fifo_losses;
    }

    private:

    static constexpr uint8_t SMPLRT_DIV   = 0x19;
    static constexpr uint8_t CONFIG       = 0x1A;
    static constexpr uint8_t GYRO_CONFIG  = 0x1B;
    static constexpr uint8_t ACCEL_CONFIG = 0x1C;
    static constexpr uint8_t FIFO_EN      = 0x23;
    static constexpr uint8_t INT_STATUS   = 0x3A;
    static constexpr uint8_t ACCEL_XOUT_H = 0x3B;
    static constexpr uint8_t ACCEL_YOUT_H = 0x3D;
    static constexpr uint8_t ACCEL_ZOUT_H = 0x3F;
    static constexpr uint8_t GYRO_XOUT_H  = 0x43;
    static constexpr uint8_t GYRO_YOUT_H  = 0x45;
    static constexpr uint8_t GYRO_ZOUT_H  = 0x47;
    static constexpr uint8_t USER_CTRL    = 0x6A;
    static constexpr uint8_t PWR_MGMT_1   = 0x6B;
    static constexpr uint8_t FIFO_COUNTH  = 0x72;
    static constexpr uint8_t FIFO_R_W     = 0x74;

    static constexpr uint8_t PWR_CLKSEL_INTERNAL   = 0x00;
    static constexpr uint8_t PWR_CLKSEL_PLL_XGYRO  = 0x01;
    static constexpr uint8_t FIFO_EN_ZG            = 0x10;
    static constexpr uint8_t INT_STATUS_FIFO_OFLOW = 0x10;
    static constexpr uint8_t USER_CTRL_FIFO_ENABLE = 0x40;
    static constexpr uint8_t USER_CTRL_FIFO_RESET  = 0x04;

    static constexpr float GRAVITY = 9.80665f;

    // Fixed-size rolling average over the last IMU_ROLLING_AVG_SAMPLES raw readings.
    class RollingAverage {
        public:

        int16_t push(int16_t sample) {
            sum -= samples[index];
            samples[index] = sample;
            sum += sample;
            index = (index + 1) % IMU_ROLLING_AVG_SAMPLES;
            if (count < IMU_ROLLING_AVG_SAMPLES) count++;
            return static_cast<int16_t>(sum / count);
        }

        private:

        int16_t samples[IMU_ROLLING_AVG_SAMPLES] = {0};
        int32_t sum                              = 0;
        uint8_t index                            = 0;
        uint8_t count                            = 0;
    };

    uint8_t address;
    // Defaulted rather than left indeterminate: a bus that does not answer
    // returns from init() before these are set, and nothing downstream should
    // be reading garbage even on a path that ought not to be reached.
    float gyroscopicSensitivity    = 32.8f;
    float accelerometerSensitivity = 8192.0f;
    float gyro_rad_per_lsb         = 0.0f;
    float sample_period            = 0.0f;
    bool fifo_ready                = false;

    unsigned long read_failures = 0;
    unsigned long fifo_losses   = 0;

    RollingAverage gyroXAvg, gyroYAvg, gyroZAvg;
    RollingAverage accelXAvg, accelYAvg, accelZAvg;

    float toRadPerSec(int16_t raw) const {
        return (raw / gyroscopicSensitivity) * DEG_TO_RAD;
    }

    float toMPerSec2(int16_t raw) const {
        return (raw / accelerometerSensitivity) * GRAVITY;
    }

    static float gyroSensitivity(GyroScale scale) {
        switch (scale) {
            case GyroScale::DPS_250:  return 131.0f;
            case GyroScale::DPS_500:  return 65.5f;
            case GyroScale::DPS_1000: return 32.8f;
            case GyroScale::DPS_2000: return 16.4f;
            default:                  return 32.8f;
        }
    }

    static float accelSensitivity(AccelScale scale) {
        switch (scale) {
            case AccelScale::G_2:  return 16384.0f;
            case AccelScale::G_4:  return 8192.0f;
            case AccelScale::G_8:  return 4096.0f;
            case AccelScale::G_16: return 2048.0f;
            default:               return 8192.0f;
        }
    }

    void writeReg(uint8_t reg, uint8_t val) {
        Wire.beginTransmission(address);
        Wire.write(reg);
        Wire.write(val);
        Wire.endTransmission();
    }

    bool readByte(uint8_t reg, uint8_t& out) {
        Wire.beginTransmission(address);
        Wire.write(reg);
        if (Wire.endTransmission(false) != 0) { ++read_failures; return false; }
        Wire.requestFrom(address, static_cast<uint8_t>(1));
        if (Wire.available() < 1) { ++read_failures; return false; }
        out = static_cast<uint8_t>(Wire.read());
        return true;
    }

    bool readUint16(uint8_t reg, uint16_t& out) {
        Wire.beginTransmission(address);
        Wire.write(reg);
        if (Wire.endTransmission(false) != 0) { ++read_failures; return false; }
        Wire.requestFrom(address, static_cast<uint8_t>(2));
        if (Wire.available() < 2) { ++read_failures; return false; }
        const uint8_t hi = static_cast<uint8_t>(Wire.read());
        const uint8_t lo = static_cast<uint8_t>(Wire.read());
        out              = static_cast<uint16_t>((static_cast<uint16_t>(hi) << 8) | lo);
        return true;
    }

    int16_t readWord(uint8_t reg) {
        uint16_t raw = 0;
        if (!readUint16(reg, raw)) return 0;
        return static_cast<int16_t>(raw);
    }
};
