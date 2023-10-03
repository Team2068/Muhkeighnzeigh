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

        wristMotor.setOpenLoopRampRate(0);

        wristMotor.setSmartCurrentLimit(15);

        wristMotor.getEncoder().setPositionConversionFactor(360/31);
        // TODO: Set wristMotor's on-board encoder to be the absolute encoder [[[==]]]
        // wristMotor.getPIDController().setP(0.00001);
        syncWrist();



        compressor.enableDigital();
    }

    public void syncWrist(){
        wristMotor.getEncoder().setPosition(getPosition());
    }

    public void setWristVoltage(double voltage) {
        if((getPosition() >= 145  && voltage < 0) || (getPosition() <= -67 && voltage > 0)) {
            DriverStation.reportWarning("Claw driving into itself!", false);
            wristMotor.setVoltage(0);
        }else if (getPosition() == -169.5) {
            DriverStation.reportError("CLAW DISCONNECTED", false);
            wristMotor.setVoltage(0);
        }else{
            wristMotor.setVoltage(voltage);
        }
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

    // public void setPosition(double position){
    //     if (position < 145 || position> -18)
    //         wristMotor.getPIDController().setReference(position, ControlType.kPosition);
    // }

    public void openClaw() {
        clawSolenoid.set(Value.kReverse);
    }

    public void closeClaw() {
        clawSolenoid.set(Value.kForward);
    }

    public void stopClaw() {
        intakeMotor.set(0);
    }

    public double getPosition() {
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
        SmartDashboard.putNumber("Claw Position", getPosition());
        SmartDashboard.putNumber("Claw Abs Position", clawEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Claw Power", wristMotor.getBusVoltage());
    }
}
