package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase{
    private static double kDt = 0;
    SimpleMotorFeedforward feedforward = new 
    private final Encoder armEncoder = new Encoder(1, 2);
    private final CANSparkMax arm1Motor = new CANSparkMax(ArmConstants.ArmMotor1, MotorType.kBrushless);
    private final CANSparkMax arm2Motor = new CANSparkMax(ArmConstants.ArmMotor2, MotorType.kBrushless);
    private final TrapezoidProfile.Constraints constraints =
        new TrapezoidProfile.Constraints(0, 0);
    private final ProfiledPIDController controller =
        new ProfiledPIDController(0,0,0, constraints, kDt);


public void robotInit(){
    armEncoder.setDistancePerPulse(1/360 * 2 * Math.PI * 1.5);

}
public void goToLowerGoal(double lowerGoalPosition){
    double lowerPidVal = controller.calculate(armEncoder.getDistance(), lowerGoalPosition);
    double lowerAcceleration = (controller.getSetpoint().velocity - lastSpeed)/ (Timer.getFPGATimestamp()-lastTime);
    arm1Motor.setVoltage(lowerPidVal + feedForward.calculate(controller.getSetpoint().velocity,lowerAcceleration));
    arm2Motor.setVoltage(lowerPidVal + feedForward.calculate(controller.getSetpoint().velocity, lowerAcceleration));

}
public void goToUpperGoal(double upperGoalPosition){
    double upperPidVal = controller.calculate(armEncoder.getDistance(), upperGoalPosition);
    double upperAcceleration = (controller.getSetpoint().velocity - lastSpeed)/ (Timer.getFPGATimestamp() - lastSpeed);
    arm2Motor.setVoltage(upperPidVal + feedForward.calculate(controller.getSetpoint().velocity, upperAcceleration));
    arm1Motor.setVoltage(upperPidVal + feedForward.calculate );


}
    }

