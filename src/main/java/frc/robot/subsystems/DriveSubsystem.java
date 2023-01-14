package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper.GearRatio;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    public static double MAX_VOLTAGE = 9;
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 3;
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = (MAX_VELOCITY_METERS_PER_SECOND /
            Math.hypot(Constants.DRIVETRAIN_WHEELBASE_METERS / 2, Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2));

    private final SwerveDriveKinematics DriveKinematics = new SwerveDriveKinematics(
            new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2, Constants.DRIVETRAIN_WHEELBASE_METERS / 2),
            new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2, -Constants.DRIVETRAIN_WHEELBASE_METERS / 2),
            new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2, Constants.DRIVETRAIN_WHEELBASE_METERS / 2),
            new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2, -Constants.DRIVETRAIN_WHEELBASE_METERS / 2)

    );

    private final Pigeon2 pigeon2 = new Pigeon2(14);

    // FIXME: implement SwerveModulePositions
    SwerveDriveOdometry odometry = new SwerveDriveOdometry(
            DriveKinematics, getGyroscopeRotation(), getModulePositions(), new Pose2d(0, 0, new Rotation2d()));

    private final SwerveModule frontLeftModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backRightModule;

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);

    private boolean fieldOriented = false;

    public DriveSubsystem() {
        ShuffleboardTab tab = Shuffleboard.getTab("driveTrain");

        frontLeftModule = Mk4SwerveModuleHelper.createNeo(
                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0),
                GearRatio.L2,
                DriveConstants.FRONT_LEFT_DRIVE_MOTOR,
                DriveConstants.FRONT_LEFT_TURN_MOTOR,
                DriveConstants.FRONT_LEFT_ENCODER,
                DriveConstants.FRONT_LEFT_ENCODER_OFFSET);

        backLeftModule = Mk4SwerveModuleHelper.createNeo(
                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0),
                GearRatio.L2,
                DriveConstants.BACK_LEFT_DRIVE_MOTOR,
                DriveConstants.BACK_LEFT_TURN_MOTOR,
                DriveConstants.BACK_LEFT_ENCODER,
                DriveConstants.BACK_LEFT_ENCODER_OFFSET);
        frontRightModule = Mk4SwerveModuleHelper.createNeo(
                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0),
                GearRatio.L2,
                DriveConstants.FRONT_RIGHT_DRIVE_MOTOR,
                DriveConstants.FRONT_RIGHT_TURN_MOTOR,
                DriveConstants.FRONT_RIGHT_ENCODER,
                DriveConstants.FRONT_RIGHT_ENCODER_OFFSET);
        backRightModule = Mk4SwerveModuleHelper.createNeo(
                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0),
                GearRatio.L2,
                DriveConstants.BACK_RIGHT_DRIVE_MOTOR,
                DriveConstants.BACK_RIGHT_TURN_MOTOR,
                DriveConstants.BACK_RIGHT_ENCODER,
                DriveConstants.BACK_RIGHT_ENCODER_OFFSET);

    }

    public void ZeroGyro() {
        pigeon2.getYaw();
    }

    public SwerveDriveKinematics getKinematics() {
        return DriveKinematics;
    }

    public Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(Math.IEEEremainder(pigeon2.getYaw(), 360));
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;
    }

    private SwerveModulePosition getModulePosition(SwerveModule module) {
        return new SwerveModulePosition(module.getDrivePosition(), Rotation2d.fromRadians(module.getSteerAngle()));
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] pos = new SwerveModulePosition[4];
        pos[0] = getModulePosition(frontLeftModule);
        pos[1] = getModulePosition(backLeftModule);
        pos[2] = getModulePosition(frontRightModule);
        pos[3] = getModulePosition(backRightModule);
        return pos;
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry() {
        ZeroGyro();
        odometry.resetPosition(getGyroscopeRotation(), getModulePositions(), getPose());
    }

    public void setModuleStates(SwerveModuleState[] states) {
        frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[0].angle.getRadians());
        frontRightModule.set(states[1].speedMetersPerSecond / MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * MAX_VOLTAGE,
                states[1].angle.getRadians());
        backLeftModule.set(states[2].speedMetersPerSecond / MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * MAX_VOLTAGE,
                states[2].angle.getRadians());
        backRightModule.set(states[3].speedMetersPerSecond / MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * MAX_VOLTAGE,
                states[3].angle.getRadians());
    }

    public boolean isFieldOriented() {
        return isFieldOriented();
    }

    public void toggleFieldOriented() {
        fieldOriented = !isFieldOriented();
    }

    public void MaxSpeed() {
        MAX_VOLTAGE = 12;

    }

    public void NormalSpeed() {
        MAX_VOLTAGE = 9;

    }

    public void MinSpeed() {
        MAX_VOLTAGE = 5;
    }

    public void periodic() {
        SwerveModuleState[] states = DriveKinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, chassisSpeeds, MAX_VOLTAGE, MAX_VELOCITY_METERS_PER_SECOND,
                MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
        setModuleStates(states);
        Pose2d pose = getPose();
        SmartDashboard.putNumber("X position", pose.getX());
        SmartDashboard.putNumber("Y position", pose.getX());
        SmartDashboard.putNumber("Odometry rotation", pose.getRotation().getDegrees());
        SmartDashboard.putString("Drive Mode", fieldOriented ? "Field" : "Robot");
    }
}
