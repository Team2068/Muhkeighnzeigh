package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    double lastSpeed = 0;
    double lastTime = Timer.getFPGATimestamp();
    SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0, 0, 0);
    private final CANSparkMax arm1Motor = new CANSparkMax(ArmConstants.ArmMotor1, MotorType.kBrushless);
    private final CANSparkMax arm2Motor = new CANSparkMax(ArmConstants.ArmMotor2, MotorType.kBrushless);
    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(0, 0);
    private final ProfiledPIDController controller = new ProfiledPIDController(0, 0, 0, constraints);
    private final DutyCycleEncoder armEncoder = new DutyCycleEncoder(0);
    private final double encoderOffset = 0.427;

    public ArmSubsystem() {
        armEncoder.setDutyCycleRange(0, 1);
    }

    public void goToLowerGoal(double lowerGoalPosition) {
        double lowerPidVal = controller.calculate(getArmPosition(), lowerGoalPosition);
        double lowerAcceleration = (controller.getSetpoint().velocity - lastSpeed)
                / (Timer.getFPGATimestamp() - lastTime);
        arm1Motor.setVoltage(lowerPidVal + feedForward.calculate(controller.getSetpoint().velocity, lowerAcceleration));
        arm2Motor.setVoltage(lowerPidVal + feedForward.calculate(controller.getSetpoint().velocity, lowerAcceleration));
    }

    public void goToUpperGoal(double upperGoalPosition) {
        double upperPidVal = controller.calculate(getArmPosition(), upperGoalPosition);
        double upperAcceleration = (controller.getSetpoint().velocity - lastSpeed)
                / (Timer.getFPGATimestamp() - lastTime);
        arm2Motor.setVoltage(upperPidVal + feedForward.calculate(controller.getSetpoint().velocity, upperAcceleration));
        arm1Motor.setVoltage(upperPidVal + feedForward.calculate(controller.getSetpoint().velocity, upperAcceleration));
    }

    /**
     * @return The absolute arm angle in degrees from 0-360 up positive.
     */
    public double getArmPosition() {
        double deg = (-armEncoder.getAbsolutePosition() + encoderOffset) * 360;
        return (deg % 360) + (deg < 0 ? 360 : 0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Absolute Rotation", armEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Arm Rotation", armEncoder.get());
        SmartDashboard.putNumber("getArmPosition()", getArmPosition());
    }
}
