package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class ClawSubsystem extends SubsystemBase {
    private final CANSparkMax claw = new CANSparkMax(ClawConstants.CLAW_MOTOR, MotorType.kBrushless);
    private final CANSparkMax intakeMotor = new CANSparkMax(ClawConstants.INTAKE_MOTOR, MotorType.kBrushless);
    private final DutyCycleEncoder clawEncoder = new DutyCycleEncoder(1);
    private final SimpleMotorFeedforward clawFeedforward = new SimpleMotorFeedforward(0, 0, 0);
    public ClawSubsystem() {
        clawEncoder.setDutyCycleRange(0,1);

        claw.setIdleMode(IdleMode.kBrake);
        intakeMotor.setIdleMode(IdleMode.kCoast);
        claw.setOpenLoopRampRate(.4);
        intakeMotor.setOpenLoopRampRate(.4);
    }
    public void setClawVoltage(double voltage){
        claw.setClawVoltage(voltage);
    }
    public void setClaw(double speed){
        claw.setClaw(speed);
        intakeMotor.setClaw(speed);
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
    public double getClawPosition(){
        double clawDeg = (clawEncoder.getAbsolutePosition() + ClawConstants.CLAW_OFFSET) *360;
        return (deg % 360) + (deg < 0 ? 360 : 0);
    }
    public double calculateClawFeedforward(double positionRadians, double velocity){
        return clawFeedforward.calculate(positionRadians, velocity);
    }
    @Override
    public void periodic(){
        SmartDashboard.putNumber("getClawPosition()", getClawPosition());
        SmartDashboard.putNumber("Claw Power", claw.getBusVoltage());
    }
}
