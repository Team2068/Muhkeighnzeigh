package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TelescopeConstants;

public class TelescopeSubsystem extends SubsystemBase {
    private CANSparkMax telescopeMotor = new CANSparkMax(TelescopeConstants.TELESCOPE_MOTOR, MotorType.kBrushless);

    public TelescopeSubsystem() {
        telescopeMotor.setIdleMode(IdleMode.kCoast);
    }

    public void extendTelescope(double speed) {
        telescopeMotor.set(speed);
    }

    public void extendTelescope() {
        telescopeMotor.set(TelescopeConstants.TELESCOPE_SPEED);
    }

    public void retractTelescope(double speed) {
        telescopeMotor.set(-speed);
    }

    public void retractTelescope() {
        telescopeMotor.set(-TelescopeConstants.TELESCOPE_SPEED);
    }

    public void stopTelescope() {
        telescopeMotor.set(0);
    }
}
