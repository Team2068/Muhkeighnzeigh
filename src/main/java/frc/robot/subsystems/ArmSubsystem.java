package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase{
    private CANSparkMax ArmMotor1 = new CANSparkMax(ArmConstants.ArmMotor1, MotorType.kBrushless);
    private CANSparkMax ArmMotor2 = new CANSparkMax(ArmConstants.ArmMotor2, MotorType.kBrushless);

    public ArmSubsystem(){
        ArmMotor1.setIdleMode(IdleMode.kCoast);
        ArmMotor2.setIdleMode(IdleMode.kCoast);
    }
    public void liftArm(double speed){
        ArmMotor1.set(speed);
        ArmMotor2.set(speed);
    }
    public void liftArm(){
        ArmMotor1.set(ArmConstants.ArmLiftSpeed);
        ArmMotor2.set(ArmConstants.ArmLiftSpeed);
    }
    public void lowerArm(double speed){
        ArmMotor1.set(-speed);
        ArmMotor2.set(-speed);
    }
    public void lowerArm(){
        ArmMotor1.set(ArmConstants.ArmLowerSpeed);
        ArmMotor2.set(ArmConstants.ArmLowerSpeed);
    }
    public void stopArm(){
        ArmMotor1.set(0);
        ArmMotor2.set(0);
    }
}
