// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utilities.DebugTable;

public class SwerveModule{
    public final CANSparkMax driveMotor;
    public final CANSparkMax steerMotor;
    public final CANCoder steerEncoder; 

    final double WHEEL_DIAMETER = 0.10033; // Metres
    final double DRIVE_REDUCTION = (15.0 / 32.0) * (10.0 / 60.0);
    final double STEER_REDUCTION = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    final double DRIVE_CONVERSION_FACTOR = Math.PI * WHEEL_DIAMETER * DRIVE_REDUCTION;

    double desiredAngle;

    public SwerveModule(ShuffleboardLayout tab, int driveID, int steerID, int steerCANID, double offset){
        driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        steerMotor = new CANSparkMax(steerID, MotorType.kBrushless);
        steerEncoder = new CANCoder(steerCANID);

        steerEncoder.configFactoryDefault();
        steerEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360); // NOTE: Might want to test putting it between [-180,180]
        steerEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        steerEncoder.configSensorDirection(true);
        steerEncoder.configMagnetOffset(offset);
        steerEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 100, 250);

        steerMotor.setSmartCurrentLimit(20);
        driveMotor.setSmartCurrentLimit(40);

        steerMotor.setIdleMode(IdleMode.kBrake);
        driveMotor.setIdleMode(IdleMode.kCoast);

        driveMotor.getEncoder().setVelocityConversionFactor(DRIVE_CONVERSION_FACTOR / 60.0);
        driveMotor.getEncoder().setPositionConversionFactor(DRIVE_CONVERSION_FACTOR);

        steerMotor.getEncoder().setPositionConversionFactor(Math.PI * STEER_REDUCTION);
        steerMotor.getEncoder().setVelocityConversionFactor(Math.PI * STEER_REDUCTION / 60);
        steerMotor.getEncoder().setPosition(steerEncoder.getAbsolutePosition());

        driveMotor.setInverted(true);
        steerMotor.setInverted(true);

        driveMotor.enableVoltageCompensation(12);

        steerMotor.getPIDController().setP(1);
        steerMotor.getPIDController().setD(1.0);

        steerMotor.getPIDController().setFeedbackDevice(steerMotor.getEncoder());

        steerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
        steerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 20);
        steerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 20);


        tab.addDouble("Absolute Angle", steerEncoder::getAbsolutePosition);
        tab.addDouble("Current Angle", () -> Math.toDegrees(steerMotor.getEncoder().getPosition()));
        tab.addDouble("Target Angle", () -> desiredAngle);
        tab.addDouble("Velocity", steerMotor.getEncoder()::getVelocity);
    }

    public void resetDrivePosition() {
        driveMotor.getEncoder().setPosition(0);
    }

    public void resetSteerPosition(){
        steerMotor.getEncoder().setPosition(Math.toRadians(steerEncoder.getAbsolutePosition()));
    }

    public double getDrivePosition(){
        return driveMotor.getEncoder().getPosition();
    }

    public double getSteerAngle(){
        return steerMotor.getEncoder().getPosition(); // May Switch to absolute Encoder
    }

    public void setX(double driveVolts, double targetAngle){ // DEBUG: Remove once confirmed PID gains are valid
        // if (Math.abs(steerEncoder.getAbsolutePosition() - targetAngle) < 1) return;
        desiredAngle = targetAngle;
        steerMotor.getPIDController().setReference(Math.toRadians(targetAngle), CANSparkMax.ControlType.kPosition);
        driveMotor.setVoltage(driveVolts);
    }

    public void set(double driveVolts, double targetAngle){
        // Put in range of [0, 360)
        // targetAngle %= 360;
        // targetAngle += (targetAngle < 0.0) ? 360 : 0;

        desiredAngle = targetAngle; // For DebugTable
        // targetAngle %= 360;
        // targetAngle += (targetAngle < 0.0) ? 360 : 0;

        // double stateAngle = steerMotor.getEncoder().getPosition() % 360;
        // stateAngle += (stateAngle < 0.0) ? 360 : 0;

        double diff = targetAngle - (steerMotor.getEncoder().getPosition() /2); // TEST: May need to replace with stateAngle
        DebugTable.set(String.format("Module: %d  Diff", steerMotor.getDeviceId()), diff);

        if (diff > 90 || diff < -90){ // move to a closer angle and drive backwards 
            targetAngle += 180; 
            driveVolts *= -1.0;
        }

        // double currentModAngle = steerEncoder.getAbsolutePosition() % 360;
        // currentModAngle += (currentModAngle < 0.0) ? 360 : 0;
        driveMotor.setVoltage(driveVolts);
        steerMotor.getPIDController().setReference(Math.toRadians(targetAngle), ControlType.kPosition);
    }
}