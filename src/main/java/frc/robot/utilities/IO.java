package frc.robot.utilities;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.PhotonConstants;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.SetArmProfiled;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Photonvision;
import frc.robot.subsystems.TelescopeSubsystem;

public class IO {
    final CommandXboxController mechController = new CommandXboxController(0);
    final CommandXboxController driveController = new CommandXboxController(3);

    public final ArmSubsystem arm = new ArmSubsystem();
    public final LEDSubsystem leds = new LEDSubsystem();
    public final ClawSubsystem claw = new ClawSubsystem();
    public final DriveSubsystem driveSubsystem = new DriveSubsystem();
    public final TelescopeSubsystem telescope = new TelescopeSubsystem();
    public final Photonvision photon = new Photonvision(PhotonConstants.CAM_NAME);

    public SetArmProfiled armCommand = new SetArmProfiled(73, arm, telescope, photon::rotateMount, true);

    public void configGlobal() {
        photon.camera.setPipelineIndex(1);
        photon.mount.setAngle(PhotonConstants.FORWARD_ANGLE);
        
        driveSubsystem.setDefaultCommand(new DefaultDriveCommand(driveSubsystem,
                () -> -modifyAxis(driveController.getLeftY()) * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                () -> -modifyAxis(driveController.getLeftX()) * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                () -> -modifyAxis(driveController.getRightX())* DriveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));
        arm.setDefaultCommand(armCommand);
    }

    public void configTeleop() {
    }

    public void configTesting() {
        DriverStation.silenceJoystickConnectionWarning(true);

        mechController.a().onTrue(new InstantCommand(armCommand::stop));
        mechController.b().onTrue(new InstantCommand(() -> armCommand.setAngle(0)));
        mechController.x().onTrue(new InstantCommand(() -> armCommand.setAngle(-75)));
        mechController.y().onTrue(new InstantCommand(() -> armCommand.setAngle(60)));

        mechController.rightBumper().onTrue(new InstantCommand(claw::openClaw)
                .andThen(new InstantCommand(() -> leds.setAllLeds(new Color(0.2, 0.15, 0)))));
        mechController.leftBumper().onTrue(new InstantCommand(claw::closeClaw)
                .andThen(new InstantCommand(() -> leds.setAllLeds(new Color(0, 0, 0.25)))));

        mechController.leftTrigger().whileTrue(new InstantCommand(telescope::extendTelescope))
                .whileFalse(new InstantCommand(telescope::stopTelescope));
        mechController.povDown().whileTrue(new InstantCommand(telescope::retractTelescope))
                .whileFalse(new InstantCommand(telescope::stopTelescope));
        mechController.povUp().onTrue(new InstantCommand(telescope::resetPosition));

        claw.setDefaultCommand(new InstantCommand(
                () -> claw.setWristVoltage(
                        MathUtil.clamp(modifyAxis(mechController.getLeftY()) * ClawConstants.WRIST_VOLTAGE,
                                -ClawConstants.WRIST_VOLTAGE, ClawConstants.WRIST_VOLTAGE)),
                claw).alongWith(new InstantCommand(() -> claw.setIntakeSpeed(mechController.getRightY() * 2))));

        driveController.b().onTrue(new InstantCommand(driveSubsystem::syncEncoders));
        driveController.y().onTrue(new InstantCommand(driveSubsystem::resetOdometry));
        driveController.a().onTrue(new InstantCommand(driveSubsystem::syncEncoders));
        driveController.x().onTrue(new InstantCommand(driveSubsystem::zeroGyro));
        driveController.rightBumper().onTrue(new InstantCommand(()->armCommand.setAngle(60)));
        driveController.leftBumper().onTrue(new InstantCommand(armCommand::stop));
    }

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) <= deadband)
            return 0.0;
        deadband *= (value > 0.0) ? 1 : -1;
        return (value + deadband) / (1.0 + deadband);
    }

    private static double modifyAxis(double value) {
        value = deadband(value, 0.05); // Deadband
        value = Math.copySign(value * value, value); // Square the axis
        return value;
    }
}
