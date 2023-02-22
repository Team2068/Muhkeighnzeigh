package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class ClawSubsystem extends SubsystemBase {
    private CANSparkMax claw = new CANSparkMax(ClawConstants.CLAW_MOTOR, MotorType.kBrushless);
    private CANSparkMax intakeMotor = new CANSparkMax(ClawConstants.INTAKE_MOTOR, MotorType.kBrushless);

    public ClawSubsystem() {
        claw.setIdleMode(IdleMode.kCoast);
        intakeMotor.setIdleMode(IdleMode.kCoast);
    }

    public void openClaw(double speed) {
        claw.set(speed);
        intakeMotor.set(speed);
    }

    public void openClaw() {
        claw.set(ClawConstants.CLAW_SPEED);
        intakeMotor.set(ClawConstants.INTAKE_SPEED);
    }

    public void closeClaw(double speed) {
        claw.set(-speed);
        intakeMotor.set(-speed);
    }

    public void closeClaw() {
        claw.set(-ClawConstants.CLAW_SPEED);
        intakeMotor.set(-ClawConstants.INTAKE_SPEED);
    }

    public void stopClaw() {
        claw.set(0);
        intakeMotor.set(0);
    }
}
