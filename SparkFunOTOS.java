/*
    SPDX-License-Identifier: MIT
    
    Copyright (c) 2024 SparkFun Electronics
*/
package org.firstinspires.ftc.teamcode;

import java.util.Arrays;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

/*******************************************************************************
    sfeQwiicOtos.java - Java driver implementation for the SparkFun Qwiic Optical
    Tracking Odometry Sensor (OTOS).
*******************************************************************************/
@I2cDeviceType
@DeviceProperties(name = "SparkFunOTOS", xmlTag = "SparkFunOTOS")
public class SparkFunOTOS extends I2cDeviceSynchDevice {
    // Default I2C addresses of the Qwiic OTOS
    public static final byte DEFAULT_ADDRESS = 0x17;
    // Minimum scalar value for the linear and angular scalars
    public static final double MIN_SCALAR = 0.872;

    // Maximum scalar value for the linear and angular scalars
    public static final double MAX_SCALAR = 1.127;

    // OTOS register map
    protected static final byte REG_PRODUCT_ID = 0x00;
    protected static final byte REG_HW_VERSION = 0x01;
    protected static final byte REG_FW_VERSION = 0x02;
    protected static final byte REG_SCALAR_LINEAR = 0x04;
    protected static final byte REG_SCALAR_ANGULAR = 0x05;
    protected static final byte REG_IMU_CALIB = 0x06;
    protected static final byte REG_RESET = 0x07;
    protected static final byte REG_SIGNAL_PROCESS = 0x0E;
    protected static final byte REG_SELF_TEST = 0x0F;
    protected static final byte REG_OFF_XL = 0x10;
    protected static final byte REG_OFF_XH = 0x11;
    protected static final byte REG_OFF_YL = 0x12;
    protected static final byte REG_OFF_YH = 0x13;
    protected static final byte REG_OFF_HL = 0x14;
    protected static final byte REG_OFF_HH = 0x15;
    protected static final byte REG_STATUS = 0x1F;
    protected static final byte REG_POS_XL = 0x20;
    protected static final byte REG_POS_XH = 0x21;
    protected static final byte REG_POS_YL = 0x22;
    protected static final byte REG_POS_YH = 0x23;
    protected static final byte REG_POS_HL = 0x24;
    protected static final byte REG_POS_HH = 0x25;
    protected static final byte REG_VEL_XL = 0x26;
    protected static final byte REG_VEL_XH = 0x27;
    protected static final byte REG_VEL_YL = 0x28;
    protected static final byte REG_VEL_YH = 0x29;
    protected static final byte REG_VEL_HL = 0x2A;
    protected static final byte REG_VEL_HH = 0x2B;
    protected static final byte REG_ACC_XL = 0x2C;
    protected static final byte REG_ACC_XH = 0x2D;
    protected static final byte REG_ACC_YL = 0x2E;
    protected static final byte REG_ACC_YH = 0x2F;
    protected static final byte REG_ACC_HL = 0x30;
    protected static final byte REG_ACC_HH = 0x31;
    protected static final byte REG_POS_STD_XL = 0x32;
    protected static final byte REG_POS_STD_XH = 0x33;
    protected static final byte REG_POS_STD_YL = 0x34;
    protected static final byte REG_POS_STD_YH = 0x35;
    protected static final byte REG_POS_STD_HL = 0x36;
    protected static final byte REG_POS_STD_HH = 0x37;
    protected static final byte REG_VEL_STD_XL = 0x38;
    protected static final byte REG_VEL_STD_XH = 0x39;
    protected static final byte REG_VEL_STD_YL = 0x3A;
    protected static final byte REG_VEL_STD_YH = 0x3B;
    protected static final byte REG_VEL_STD_HL = 0x3C;
    protected static final byte REG_VEL_STD_HH = 0x3D;
    protected static final byte REG_ACC_STD_XL = 0x3E;
    protected static final byte REG_ACC_STD_XH = 0x3F;
    protected static final byte REG_ACC_STD_YL = 0x40;
    protected static final byte REG_ACC_STD_YH = 0x41;
    protected static final byte REG_ACC_STD_HL = 0x42;
    protected static final byte REG_ACC_STD_HH = 0x43;

    // Product ID register value
    protected static final byte PRODUCT_ID = 0x5F;

    // Conversion factors
    protected static final double METER_TO_INCH = 39.37;
    protected static final double INCH_TO_METER = 1.0 / METER_TO_INCH;
    protected static final double RADIAN_TO_DEGREE = 180.0 / Math.PI;
    protected static final double DEGREE_TO_RADIAN = Math.PI / 180.0;

    // Conversion factor for the linear position registers. 16-bit signed
    // registers with a max value of 10 meters (394 inches) gives a resolution
    // of about 0.0003 mps (0.012 ips)
    protected static final double METER_TO_INT16 = 32768.0 / 10.0;
    protected static final double INT16_TO_METER = 1.0 / METER_TO_INT16;

    // Conversion factor for the linear velocity registers. 16-bit signed
    // registers with a max value of 5 mps (197 ips) gives a resolution of about
    // 0.00015 mps (0.006 ips)
    protected static final double MPS_TO_INT16 = 32768.0 / 5.0;
    protected static final double INT16_TO_MPS = 1.0 / MPS_TO_INT16;

    // Conversion factor for the linear acceleration registers. 16-bit signed
    // registers with a max value of 157 mps^2 (16 g) gives a resolution of
    // about 0.0048 mps^2 (0.49 mg)
    protected static final double MPSS_TO_INT16 = 32768.0 / (16.0 * 9.80665);
    protected static final double INT16_TO_MPSS = 1.0 / MPSS_TO_INT16;

    // Conversion factor for the angular position registers. 16-bit signed
    // registers with a max value of pi radians (180 degrees) gives a resolution
    // of about 0.00096 radians (0.0055 degrees)
    protected static final double RAD_TO_INT16 = 32768.0 / Math.PI;
    protected static final double INT16_TO_RAD = 1.0 / RAD_TO_INT16;

    // Conversion factor for the angular velocity registers. 16-bit signed
    // registers with a max value of 34.9 rps (2000 dps) gives a resolution of
    // about 0.0011 rps (0.061 degrees per second)
    protected static final double RPS_TO_INT16 = 32768.0 / (2000.0 * DEGREE_TO_RADIAN);
    protected static final double INT16_TO_RPS = 1.0 / RPS_TO_INT16;

    // Conversion factor for the angular acceleration registers. 16-bit signed
    // registers with a max value of 3141 rps^2 (180000 dps^2) gives a
    // resolution of about 0.096 rps^2 (5.5 dps^2)
    protected static final double RPSS_TO_INT16 = 32768.0 / (Math.PI * 1000.0);
    protected static final double INT16_TO_RPSS = 1.0 / RPSS_TO_INT16;

    public static class Pose2D {
        public double x;
        public double y;
        public double h;

        public Pose2D() {
            x = 0.0;
            y = 0.0;
            h = 0.0;
        }

        public Pose2D(double x, double y, double h) {
            this.x = x;
            this.y = y;
            this.h = h;
        }
    }

    // Enumerations for linear units used by the OTOS driver
    public static enum LinearUnit {
        METERS,
        INCHES
    }

    // Enumerations for angular units used by the OTOS driver
    public static enum AngularUnit {
        RADIANS,
        DEGREES
    }

    public static class Version {
        public byte minor;
        public byte major;

        public Version() {
            set((byte) 0);
        }

        public Version(byte value) {
            set(value);
        }

        public void set(byte value) {
            minor = (byte) (value & 0x0F);
            major = (byte) ((value >> 4) & 0x0F);
        }

        public byte get() {
            return (byte) ((major << 4) | minor);
        }
    }

    public static class SignalProcessConfig {
        public boolean enLut;
        public boolean enAcc;
        public boolean enRot;
        public boolean enVar;

        public SignalProcessConfig() {
            set((byte) 0);
        }

        public SignalProcessConfig(byte value) {
            set(value);
        }

        public void set(byte value) {
            enLut = (value & 0x01) != 0;
            enAcc = (value & 0x02) != 0;
            enRot = (value & 0x04) != 0;
            enVar = (value & 0x08) != 0;
        }

        public byte get() {
            return (byte) ((enLut ? 0x01 : 0) | (enAcc ? 0x02 : 0) | (enRot ? 0x04 : 0) | (enVar ? 0x08 : 0));
        }
    }

    public static class SelfTestConfig {
        public boolean start;
        public boolean inProgress;
        public boolean pass;
        public boolean fail;

        public SelfTestConfig() {
            set((byte) 0);
        }

        public SelfTestConfig(byte value) {
            set(value);
        }

        public void set(byte value) {
            start = (value & 0x01) != 0;
            inProgress = (value & 0x02) != 0;
            pass = (value & 0x04) != 0;
            fail = (value & 0x08) != 0;
        }

        public byte get() {
            return (byte) ((start ? 0x01 : 0) | (inProgress ? 0x02 : 0) | (pass ? 0x04 : 0) | (fail ? 0x08 : 0));
        }
    }

    public static class Status {
        public boolean warnTiltAngle;
        public boolean warnOpticalTracking;
        public boolean errorPaa;
        public boolean errorLsm;

        public Status() {
            set((byte) 0);
        }

        public Status(byte value) {
            set(value);
        }

        public void set(byte value) {
            warnTiltAngle = (value & 0x01) != 0;
            warnOpticalTracking = (value & 0x02) != 0;
            errorPaa = (value & 0x40) != 0;
            errorLsm = (value & 0x80) != 0;
        }

        public byte get() {
            return (byte) ((warnTiltAngle ? 0x01 : 0) | (warnOpticalTracking ? 0x02 : 0) | (errorPaa ? 0x40 : 0) | (errorLsm ? 0x80 : 0));
        }
    }

    protected LinearUnit _linearUnit;
    protected AngularUnit _angularUnit;
    protected double _meterToUnit;
    protected double _radToUnit;

    public SparkFunOTOS(I2cDeviceSynchSimple i2cDeviceSynchSimple, boolean deviceClientIsOwned)
    {
        super(i2cDeviceSynchSimple, deviceClientIsOwned);

        deviceClient.setI2cAddress(I2cAddr.create7bit(DEFAULT_ADDRESS));
    }

    @Override
    protected boolean doInitialize()
    {
        // Set default units to inches and degrees
        _linearUnit = LinearUnit.INCHES;
        _angularUnit = AngularUnit.DEGREES;
        _meterToUnit = METER_TO_INCH;
        _radToUnit = RADIAN_TO_DEGREE;

        // Check if the device is connected
        return isConnected();
    }

    @Override
    public Manufacturer getManufacturer()
    {
        // TODO: Update with SparkFun once it's available
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName()
    {
        return "SparkFun Qwiic Optical Tracking Odometry Sensor";
    }

    public boolean begin() {
        // Just check if the device is connected, no other setup is needed
        return isConnected();
    }

    public boolean isConnected() {
        // Read the product ID
        byte prodId = deviceClient.read8(REG_PRODUCT_ID);

        // Check if the product ID is correct
        if (prodId != PRODUCT_ID)
            return false;

        // Everything checks out, we must be connected!
        return true;
    }

    public void getVersionInfo(Version hwVersion, Version fwVersion) {
        // Read hardware and firmware version registers
        byte[] rawData = new byte[2];
        int readBytes;
        rawData = deviceClient.read(REG_HW_VERSION, 2);
        
        // Store the version info
        hwVersion.set(rawData[0]);
        fwVersion.set(rawData[1]);
    }

    public boolean selfTest() {
        // Write the self-test register to start the test
        SelfTestConfig selfTest = new SelfTestConfig();
        selfTest.set((byte) 1);
        deviceClient.write8(REG_SELF_TEST, selfTest.get());
        
        // Loop until self-test is done, should only take ~20ms as of firmware v1.0
        for (int i = 0; i < 10; i++) {
            // Give a short delay between reads
            try {
                Thread.sleep(5);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return false;
            }

            // Read the self-test register
            selfTest.set(deviceClient.read8(REG_SELF_TEST));

            // Check if the self-test is done
            if (selfTest.inProgress == false) {
                break;
            }
        }

        // Check if the self-test passed
        return selfTest.pass;
    }

    public boolean calibrateImu() {
        return calibrateImu(255, true);
    }

    public boolean calibrateImu(int numSamples, boolean waitUntilDone) {
        // Write the number of samples to the device
        deviceClient.write8(REG_IMU_CALIB, numSamples);
        
        // Wait 1 sample period (2.4ms) to ensure the register updates
        try {
            Thread.sleep(3);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            return false;
        }

        // Do we need to wait until the calibration finishes?
        if (!waitUntilDone)
            return true;

        // Wait for the calibration to finish, which is indicated by the IMU
        // calibration register reading zero, or until we reach the maximum number
        // of read attempts
        for (int numAttempts = numSamples; numAttempts > 0; numAttempts--) {
            // Read the gryo calibration register value
            byte calibrationValue = deviceClient.read8(REG_IMU_CALIB);

            // Check if calibration is done
            if (calibrationValue == 0)
                return true;

            // Give a short delay between reads. As of firmware v1.0, samples take
            // 2.4ms each, so 3ms should guarantee the next sample is done. This
            // also ensures the max attempts is not exceeded in normal operation
            try {
                Thread.sleep(3);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return false;
            }
        }

        // Max number of attempts reached, calibration failed
        return false;
    }

    public int getImuCalibrationProgress() {
        // Read the IMU calibration register
        return deviceClient.read8(REG_IMU_CALIB);
    }

    public LinearUnit getLinearUnit() {
        return _linearUnit;
    }

    public void setLinearUnit(LinearUnit unit) {
        // Check if this unit is already set
        if (unit == _linearUnit)
            return;

        // Store new unit
        _linearUnit = unit;

        // Compute conversion factor to new units
        _meterToUnit = (unit == LinearUnit.METERS) ? 1.0 : METER_TO_INCH;
    }

    public AngularUnit getAngularUnit() {
        return _angularUnit;
    }

    public void setAngularUnit(AngularUnit unit) {
        // Check if this unit is already set
        if (unit == _angularUnit)
            return;

        // Store new unit
        _angularUnit = unit;

        // Compute conversion factor to new units
        _radToUnit = (unit == AngularUnit.RADIANS) ? 1.0 : RADIAN_TO_DEGREE;
    }

    public double getLinearScalar() {
        // Read the linear scalar from the device
        byte rawScalar = deviceClient.read8(REG_SCALAR_LINEAR);

        // Convert to double, multiples of 0.1%
        return (rawScalar * 0.001) + 1.0;
    }

    public boolean setLinearScalar(double scalar) {
        // Check if the scalar is out of bounds
        if (scalar < MIN_SCALAR || scalar > MAX_SCALAR)
            return false;

        // Convert to integer, multiples of 0.1% (+0.5 to round instead of truncate)
        byte rawScalar = (byte) ((scalar - 1.0) * 1000 + 0.5);

        // Write the scalar to the device
        deviceClient.write8(REG_SCALAR_LINEAR, rawScalar);

        // Done!
        return true;
    }

    public double getAngularScalar() {
        // Read the angular scalar from the device
        byte rawScalar = deviceClient.read8(REG_SCALAR_ANGULAR);

        // Convert to double, multiples of 0.1%
        return (rawScalar * 0.001) + 1.0;
    }

    public boolean setAngularScalar(double scalar) {
        // Check if the scalar is out of bounds
        if (scalar < MIN_SCALAR || scalar > MAX_SCALAR)
            return false;

        // Convert to integer, multiples of 0.1% (+0.5 to round instead of truncate)
        byte rawScalar = (byte) ((scalar - 1.0) * 1000 + 0.5);

        // Write the scalar to the device
        deviceClient.write8(REG_SCALAR_ANGULAR, rawScalar);

        // Done!
        return true;
    }

    public void resetTracking() {
        // Set tracking reset bit
        deviceClient.write8(REG_RESET, 0x01);
    }

    public SignalProcessConfig getSignalProcessConfig() {
        // Read the signal process register
        byte rawData = deviceClient.read8(REG_SIGNAL_PROCESS);
        return new SignalProcessConfig(rawData);
    }

    public void setSignalProcessConfig(SignalProcessConfig config) {
        // Write the signal process register
        deviceClient.write8(REG_SIGNAL_PROCESS, config.get());
    }

    public Status getStatus() {
        byte rawData = deviceClient.read8(REG_STATUS);
        return new Status(rawData);
    }

    public Pose2D getOffset() {
        return readPoseRegs(REG_OFF_XL, INT16_TO_METER, INT16_TO_RAD);
    }

    public void setOffset(Pose2D pose) {
        writePoseRegs(REG_OFF_XL, pose, METER_TO_INT16, RAD_TO_INT16);
    }

    public Pose2D getPosition() {
        return readPoseRegs(REG_POS_XL, INT16_TO_METER, INT16_TO_RAD);
    }

    public void setPosition(Pose2D pose) {
        writePoseRegs(REG_POS_XL, pose, METER_TO_INT16, RAD_TO_INT16);
    }

    public Pose2D getVelocity() {
        return readPoseRegs(REG_VEL_XL, INT16_TO_MPS, INT16_TO_RPS);
    }

    public Pose2D getAcceleration() {
        return readPoseRegs(REG_ACC_XL, INT16_TO_MPSS, INT16_TO_RPSS);
    }

    public Pose2D getPositionStdDev() {
        return readPoseRegs(REG_POS_STD_XL, INT16_TO_METER, INT16_TO_RAD);
    }

    public Pose2D getVelocityStdDev() {
        return readPoseRegs(REG_VEL_STD_XL, INT16_TO_MPS, INT16_TO_RPS);
    }

    public Pose2D getAccelerationStdDev() {
        return readPoseRegs(REG_ACC_STD_XL, INT16_TO_MPSS, INT16_TO_RPSS);
    }

    public void getPosVelAcc(Pose2D pos, Pose2D vel, Pose2D acc) {
        // Read all pose registers
        byte[] rawData = new byte[18];
        int bytesRead;
        rawData = deviceClient.read(REG_POS_XL, 18);
        
        // Convert raw data to pose units
        pos = regsToPose(Arrays.copyOfRange(rawData, 0, 6), INT16_TO_METER, INT16_TO_RAD);
        vel = regsToPose(Arrays.copyOfRange(rawData, 6, 12), INT16_TO_MPS, INT16_TO_RPS);
        acc = regsToPose(Arrays.copyOfRange(rawData, 12, 18), INT16_TO_MPSS, INT16_TO_RPSS);
    }

    public void getPosVelAccStdDev(Pose2D pos, Pose2D vel, Pose2D acc) {
        // Read all pose registers
        byte[] rawData = new byte[18];
        int bytesRead;
        rawData = deviceClient.read(REG_POS_STD_XL, 18);
        
        // Convert raw data to pose units
        pos = regsToPose(Arrays.copyOfRange(rawData, 0, 6), INT16_TO_METER, INT16_TO_RAD);
        vel = regsToPose(Arrays.copyOfRange(rawData, 6, 12), INT16_TO_MPS, INT16_TO_RPS);
        acc = regsToPose(Arrays.copyOfRange(rawData, 12, 18), INT16_TO_MPSS, INT16_TO_RPSS);
    }

    public void getPosVelAccAndStdDev(Pose2D pos, Pose2D vel, Pose2D acc,
                                              Pose2D posStdDev, Pose2D velStdDev, Pose2D accStdDev) {
        // Read all pose registers
        byte[] rawData = new byte[36];
        int bytesRead;
        rawData = deviceClient.read(REG_POS_XL, 36);
        
        // Convert raw data to pose units
        pos = regsToPose(Arrays.copyOfRange(rawData, 0, 6), INT16_TO_METER, INT16_TO_RAD);
        vel = regsToPose(Arrays.copyOfRange(rawData, 6, 12), INT16_TO_MPS, INT16_TO_RPS);
        acc = regsToPose(Arrays.copyOfRange(rawData, 12, 18), INT16_TO_MPSS, INT16_TO_RPSS);
        posStdDev = regsToPose(Arrays.copyOfRange(rawData, 18, 24), INT16_TO_METER, INT16_TO_RAD);
        velStdDev = regsToPose(Arrays.copyOfRange(rawData, 24, 30), INT16_TO_MPS, INT16_TO_RPS);
        accStdDev = regsToPose(Arrays.copyOfRange(rawData, 30, 36), INT16_TO_MPSS, INT16_TO_RPSS);
    }

    protected Pose2D readPoseRegs(byte reg, double rawToXY, double rawToH) {
        int bytesRead;
        byte[] rawData = new byte[6];

        // Attempt to read the raw pose data
        rawData = deviceClient.read(reg, 6);
        
        return regsToPose(rawData, rawToXY, rawToH);
    }

    protected void writePoseRegs(byte reg, Pose2D pose, double xyToRaw, double hToRaw) {
        // Store raw data in a temporary buffer
        byte[] rawData = new byte[6];
        poseToRegs(rawData, pose, xyToRaw, hToRaw);

        // Write the raw data to the device
        deviceClient.write(reg, rawData);
    }

    protected Pose2D regsToPose(byte[] rawData, double rawToXY, double rawToH) {
        // Store raw data
        short rawX = (short) ((rawData[1] << 8) | rawData[0]);
        short rawY = (short) ((rawData[3] << 8) | rawData[2]);
        short rawH = (short) ((rawData[5] << 8) | rawData[4]);

        // Store in pose and convert to units
        Pose2D pose = new Pose2D();
        pose.x = rawX * rawToXY * _meterToUnit;
        pose.y = rawY * rawToXY * _meterToUnit;
        pose.h = rawH * rawToH * _radToUnit;

        return pose;
    }

    protected void poseToRegs(byte[] rawData, Pose2D pose, double xyToRaw, double hToRaw) {
        // Convert pose units to raw data
        short rawX = (short) (pose.x * xyToRaw / _meterToUnit);
        short rawY = (short) (pose.y * xyToRaw / _meterToUnit);
        short rawH = (short) (pose.h * hToRaw / _radToUnit);

        // Store raw data in buffer
        rawData[0] = (byte) (rawX & 0xFF);
        rawData[1] = (byte) ((rawX >> 8) & 0xFF);
        rawData[2] = (byte) (rawY & 0xFF);
        rawData[3] = (byte) ((rawY >> 8) & 0xFF);
        rawData[4] = (byte) (rawH & 0xFF);
        rawData[5] = (byte) ((rawH >> 8) & 0xFF);
    }
}