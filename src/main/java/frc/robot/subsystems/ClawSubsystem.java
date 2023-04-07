package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class ClawSubsystem extends SubsystemBase {
    private final CANSparkMax wristMotor = new CANSparkMax(ClawConstants.WRIST_MOTOR, MotorType.kBrushless);
    private final CANSparkMax intakeMotor = new CANSparkMax(ClawConstants.INTAKE_MOTOR, MotorType.kBrushed);
    private final DutyCycleEncoder clawEncoder = new DutyCycleEncoder(1);
    private final SimpleMotorFeedforward clawFeedforward = new SimpleMotorFeedforward(0.001, 0, 0);
    private final DoubleSolenoid clawSolenoid = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, 9, 10);
    private final Compressor compressor = new Compressor(1, PneumaticsModuleType.REVPH);

    public ClawSubsystem() {
        clawEncoder.setDutyCycleRange(0, 1);

        intakeMotor.setIdleMode(IdleMode.kCoast);
        wristMotor.setIdleMode(IdleMode.kBrake);

        // intakeMotor.setOpenLoopRampRate(.4);
        wristMotor.setOpenLoopRampRate(0);

        wristMotor.setSmartCurrentLimit(15  );

        compressor.enableDigital();
    }

    public void setWristVoltage(double voltage) {
        if((getClawPosition() >= 145  && voltage < 0) || (getClawPosition() <= -67 && voltage > 0)) {
            wristMotor.setVoltage(0);
            DriverStation.reportWarning("Claw driving into itself!", false);
            return;
        }else if (getClawPosition() == -169.5) {
            DriverStation.reportError("CLAW DISCONNECTED", false);
            wristMotor.setVoltage(0);
        }
        wristMotor.setVoltage(voltage);
    }

    public void setWristSpeed(double speed){
        wristMotor.set(speed);
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public void intake(){
        intakeMotor.set(-1);
    }

    public void output(){
        intakeMotor.set(0.1);
    }

    public void openClaw() {
        clawSolenoid.set(Value.kReverse);
    }

    public void closeClaw() {
        clawSolenoid.set(Value.kForward);
    }

    public void stopClaw() {
        intakeMotor.set(0);
    }

    public double getClawPosition() {
        double abs = clawEncoder.getAbsolutePosition();
        double deg = (abs - ClawConstants.WRIST_OFFSET) * 360;
        return -((abs > 0.08 && abs < 1) ? (deg - 360) : deg);
    }

    public double calculateClawFeedforward(double velocity) {
        return clawFeedforward.calculate(velocity);
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Claw State", (clawSolenoid.get() == Value.kReverse) ? "Open" : "Closed");
        SmartDashboard.putNumber("Claw Position", getClawPosition());
        SmartDashboard.putNumber("Claw Abs Position", clawEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Claw Power", wristMotor.getBusVoltage());
    }
}
