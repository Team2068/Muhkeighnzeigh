package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
  private final DriveSubsystem driveSubsystem;

  private final DoubleSupplier m_translationXSupplier;
  private final DoubleSupplier m_translationYSupplier;
  private final DoubleSupplier m_rotationSupplier;
  private final SlewRateLimiter xLimiter = new SlewRateLimiter(7);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(7);

  public DefaultDriveCommand(DriveSubsystem driveSubsystem, ChassisSpeeds chassisSpeeds) {
    this(
        driveSubsystem,
        () -> chassisSpeeds.vxMetersPerSecond,
        () -> chassisSpeeds.vyMetersPerSecond,
        () -> chassisSpeeds.omegaRadiansPerSecond);
  }

  public DefaultDriveCommand(
      DriveSubsystem driveSubsystem,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier rotationSupplier) {
    this.driveSubsystem = driveSubsystem;
    this.m_translationXSupplier = translationXSupplier;
    this.m_translationYSupplier = translationYSupplier;
    this.m_rotationSupplier = rotationSupplier;

    addRequirements(driveSubsystem);
  }

  @Override
  public void execute() {
    double xSpeed = xLimiter.calculate(m_translationXSupplier.getAsDouble());
    double ySpeed = yLimiter.calculate(m_translationYSupplier.getAsDouble());
    double rotationSpeed = m_rotationSupplier.getAsDouble() * 0.7;

    if (driveSubsystem.isFieldOriented()) {
      driveSubsystem.drive(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              xSpeed, ySpeed, rotationSpeed, driveSubsystem.getGyroscopeRotation()));
    } else {
      driveSubsystem.drive(new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed));
    }
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }
}
