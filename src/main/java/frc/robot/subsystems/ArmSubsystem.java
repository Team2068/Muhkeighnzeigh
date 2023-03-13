package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax arm1Motor = new CANSparkMax(ArmConstants.ARM_1_MOTOR, MotorType.kBrushless);
    private final CANSparkMax arm2Motor = new CANSparkMax(ArmConstants.ARM_2_MOTOR, MotorType.kBrushless);

    private final DutyCycleEncoder armEncoder = new DutyCycleEncoder(0);
    // NOTE: found values using http://reca.lc/arm
    private final ArmFeedforward feedforward = new ArmFeedforward(0.01, 1.45, 0.63, 0.14); // TODO: use SysId to calculate feedforwards

    public ArmSubsystem() {
        armEncoder.setDutyCycleRange(0, 1);

        arm1Motor.setIdleMode(IdleMode.kBrake);
        arm2Motor.setIdleMode(IdleMode.kBrake);
        
        arm1Motor.setOpenLoopRampRate(0.2);
        arm2Motor.setOpenLoopRampRate(0.2);

        arm1Motor.setInverted(true);

        ArmConstants.setOffsets();
    }

    public void setVoltage(double voltage) {
        arm1Motor.setVoltage(voltage);
        arm2Motor.setVoltage(voltage);
    }

    public void set(double speed) {
        arm1Motor.set(speed);
        arm2Motor.set(speed);
    }
    
    public void stop() {
        arm1Motor.set(0);
        arm2Motor.set(0);
    }

    /**
     * @return The absolute arm angle in degrees from 0-360 up positive.
     */
    public double getArmPosition() {
        double abs = armEncoder.getAbsolutePosition();
        double deg = (abs - ArmConstants.ARM_OFFSET) * 360;
        return (abs > ArmConstants.ARM_LIMIT && abs < 1) ? (deg - 360) : deg;
    }

    public double calculateFeedforward(double positionRadians, double velocity) {
        return feedforward.calculate(positionRadians, velocity);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Position", getArmPosition());
        SmartDashboard.putNumber("Arm Abs Position", armEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Arm1 Power", arm1Motor.getBusVoltage());
        SmartDashboard.putNumber("Arm2 Power", arm2Motor.getBusVoltage());
        SmartDashboard.putNumber("Arm1 RPM", arm1Motor.get());
        SmartDashboard.putNumber("Arm2 RPM", arm2Motor.get());
    }
}
