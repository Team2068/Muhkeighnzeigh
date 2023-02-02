package frc.robot.subsystems;

import java.beans.Encoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends ProfiledPIDSubsystem{
    private CANSparkMax ArmMotor1 = new CANSparkMax(ArmConstants.ArmMotor1, MotorType.kBrushless);
    private CANSparkMax ArmMotor2 = new CANSparkMax(ArmConstants.ArmMotor2, MotorType.kBrushless);

    private final ArmFeedforward armFeedforward = 
    new ArmFeedforward(0, 0, 0, 0);
    public ArmSubsystem(){
   super(
    new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0))
   );

   
}
    @Override
    protected void useOutput(double output, State setpoint) {
        double feedforward =  feedforward.calculate(setpoint.position, setpoint.velocity);
        ArmMotor1.setVoltage(output + feedforward);
        ArmMotor2.set(output + feedforward);
    }
    @Override
    protected double getMeasurement() {
        // TODO Auto-generated method stub
        return 0;
    }
}