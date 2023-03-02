package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class ClawSubsystem extends SubsystemBase {
    private final CANSparkMax wristMotor = new CANSparkMax(ClawConstants.WRIST_MOTOR, MotorType.kBrushless);
    private final CANSparkMax intakeMotor = new CANSparkMax(ClawConstants.INTAKE_MOTOR, MotorType.kBrushless);
    private final DutyCycleEncoder clawEncoder = new DutyCycleEncoder(1);
    private final SimpleMotorFeedforward clawFeedforward = new SimpleMotorFeedforward(0, 0, 0);

    public ClawSubsystem() {
        clawEncoder.setDutyCycleRange(0, 1);

        wristMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.setIdleMode(IdleMode.kCoast);
        wristMotor.setOpenLoopRampRate(.4);
        intakeMotor.setOpenLoopRampRate(.4);
    }

    public void setWristVoltage(double voltage) {
        wristMotor.setVoltage(voltage);
    }

    public void setWristSpeed(double speed) {
        wristMotor.set(speed);
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public void openClaw(double speed) {
        wristMotor.set(speed);
        intakeMotor.set(speed);
    }

    public void openClaw() {
        wristMotor.set(ClawConstants.WRIST_SPEED);
        intakeMotor.set(ClawConstants.INTAKE_SPEED);
    }

    public void closeClaw(double speed) {
        wristMotor.set(-speed);
        intakeMotor.set(-speed);
    }

    public void closeClaw() {
        wristMotor.set(-ClawConstants.WRIST_SPEED);
        intakeMotor.set(-ClawConstants.INTAKE_SPEED);
    }

    public void stopClaw() {
        wristMotor.set(0);
        intakeMotor.set(0);
    }

    public double getClawPosition() {
        double deg = (clawEncoder.getAbsolutePosition() + ClawConstants.WRIST_OFFSET) * 360;
        return (deg % 360) + (deg < 0 ? 360 : 0);
    }

    public double calculateClawFeedforward(double positionRadians, double velocity) {
        return clawFeedforward.calculate(positionRadians, velocity);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("getClawPosition()", getClawPosition());
        SmartDashboard.putNumber("Claw Power", wristMotor.getBusVoltage());
    }
}
