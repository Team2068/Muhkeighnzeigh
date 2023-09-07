package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TelescopeConstants;

public class TelescopeSubsystem extends SubsystemBase {
    CANSparkMax telescope = new CANSparkMax(TelescopeConstants.TELESCOPE_MOTOR, MotorType.kBrushless);
    
    public TelescopeSubsystem() {
        telescope.setIdleMode(IdleMode.kBrake);
        telescope.getPIDController().setP(0.03);
    }

    public void extend(double speed) {
        telescope.set(speed);
    }

    public void extend() {
        telescope.set(TelescopeConstants.TELESCOPE_SPEED);
    }

    public void retract(double speed) {
        if(getPosition() <= 1)
            DriverStation.reportWarning("Retract stopped, position <= 0", false);
        else
            telescope.set(-speed);
    }

    public void retract() {
        retract(TelescopeConstants.TELESCOPE_SPEED);
    }

    public void setPosition(double position){
        if (position > 0) telescope.getPIDController().setReference(position, ControlType.kPosition);
    }

    public void stop() {
        telescope.set(0);
    }

    public void setVoltage(double voltage) {
        telescope.setVoltage(voltage);
    }

    public double getPosition() {
        return telescope.getEncoder().getPosition();
    }

    public void resetPosition() {
        telescope.getEncoder().setPosition(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Telescope Position", getPosition());
    }
}
