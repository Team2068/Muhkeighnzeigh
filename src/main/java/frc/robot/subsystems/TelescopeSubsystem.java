package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TelescopeConstants;

public class TelescopeSubsystem extends SubsystemBase {
    public CANSparkMax telescopeMotor = new CANSparkMax(TelescopeConstants.TELESCOPE_MOTOR, MotorType.kBrushless);
    Photonvision pv;

    public TelescopeSubsystem() {
        telescopeMotor.setIdleMode(IdleMode.kBrake);
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

    public void autoExtend () {
        double m = 0.0;
        double b = 0.0;
        double distanceToTelescopePosition = (m * pv.getDistance(pv.camera.getLatestResult()) + b);

        while (telescopeMotor.getEncoder().getPosition() != distanceToTelescopePosition) {
            extendTelescope();
        }
      } 

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Telescope Position", telescopeMotor.getEncoder().getPosition());
    }
}
