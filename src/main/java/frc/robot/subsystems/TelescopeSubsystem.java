package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TelescopeConstants;

public class TelescopeSubsystem extends SubsystemBase {
    private CANSparkMax telescopeMotor = new CANSparkMax(TelescopeConstants.TELESCOPE_MOTOR, MotorType.kBrushless);
    public TelescopeSubsystem() {
        telescopeMotor.setIdleMode(IdleMode.kBrake);
        // telescopeMotor.getPIDController().SetP(0.0001);
    }

    public void extendTelescope(double speed) {
        telescopeMotor.set(speed);
    }

    public void extendTelescope() {
        telescopeMotor.set(TelescopeConstants.TELESCOPE_SPEED);
    }

    public void retractTelescope(double speed) {
        if(getPosition() <= 1)
            DriverStation.reportWarning("Retract stopped, position <= 0", false);
        else
            telescopeMotor.set(-speed);
    }

    public void retractTelescope() {
        retractTelescope(TelescopeConstants.TELESCOPE_SPEED);
    }

    public void stopTelescope() {
        telescopeMotor.set(0);
    }

    public void setVoltage(double voltage) {
        telescopeMotor.setVoltage(voltage);
    }

    public double getPosition() {
        return telescopeMotor.getEncoder().getPosition();
    }

    public void resetPosition() {
        telescopeMotor.getEncoder().setPosition(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Telescope Position", getPosition());
    }
}
