package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase{
    private static double kDt = 0;
    private final Encoder armEncoder = new Encoder(1, 2);
    private final CANSparkMax arm1Motor = new CANSparkMax(ArmConstants.ArmMotor1, MotorType.kBrushless);
    private final CANSparkMax arm2Motor = new CANSparkMax(ArmConstants.ArmMotor2, MotorType.kBrushless);
    private final TrapezoidProfile.Constraints constraints =
        new TrapezoidProfile.Constraints(0, 0);
    private final ProfiledPIDController controller =
        new ProfiledPIDController(0,0,0, constraints, kDt);
}
