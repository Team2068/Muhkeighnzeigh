package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class ClawSubsystem extends SubsystemBase{
    private CANSparkMax Claw = new CANSparkMax(ClawConstants.Claw,com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax IntakeMotor = new CANSparkMax(ClawConstants.IntakeMotor,com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);

    public ClawSubsystem(){
        Claw.setIdleMode(IdleMode.kCoast);
        IntakeMotor.setIdleMode(IdleMode.kCoast);
    }
    public void openClaw(double speed){
        Claw.set(speed);
        IntakeMotor.set(speed);
    }
    public void openClaw(){
        Claw.set(ClawConstants.ClawSpeed);
        IntakeMotor.set(ClawConstants.IntakeSpeed);
    }
    public void closeClaw(double speed){
        Claw.set(-speed);
        IntakeMotor.set(-speed);
    }
    public void closeClaw(){
        Claw.set(-ClawConstants.ClawSpeed);
        IntakeMotor.set(-ClawConstants.IntakeSpeed);
    }
    public void stopClaw(){
        Claw.set(0);
        IntakeMotor.set(0);
    }
}
