package frc.robot.turret;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.LimelightProfiler;
import frc.lib.MultithreadedSubsystem;
import frc.lib.PosePublisher;
import frc.robot.LimelightHelpers;
import frc.robot.RobotConstants;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.azimuth.Azimuth;
import frc.robot.drive.Drive;
import frc.robot.hood.Hood;
import frc.robot.shooter.Shooter;

/** Turret subsystem */
public class Turret extends MultithreadedSubsystem {

  /** Turret subsystem singleton */
  private static Turret instance = null;

  /** Azimuth subsystem reference */
  private final Azimuth azimuth;

  /** Hood subsystem reference */
  private final Hood hood;

  /** Shooter subsystem reference */
  private final Shooter shooter;

  /** Drive subsystem reference */
  private final Drive drive;

  /** Current turret state */
  private TurretState state;

  /** Estimated turret pose */
  private volatile Pose2d turretPose;

  /** Turret limelight profiler */
  private final LimelightProfiler limelightProfiler;

  /** Enables/disables limelight profiling (when enabled can cause performance issues) */
  private boolean limelightProfilingEnabled = false;

  // State specific control variables

  /** Robot pose used by targetHubFromPose */
  private Pose2d manualRobotPose;

  /**
   * Gets turret subsystem instance
   * 
   * @return turret subsystem instance
   */
  public static Turret getInstance() {
    if (instance == null) {
      instance = new Turret();
    }

    return instance;
  }

  /** Turret subsystem constructor */
  private Turret() {
    azimuth = Azimuth.getInstance();
    hood = Hood.getInstance();
    shooter = Shooter.getInstance();
    drive = Drive.getInstance();

    state = TurretState.STOW;

    LimelightHelpers.setCameraPose_RobotSpace("limelight-turret", -0.115913, 0.080866, 0.734112, 0, 15, 0);
    limelightProfiler = new LimelightProfiler("limelight-turret");
  }

  @Override 
  public void initializeTab() {
    ShuffleboardTab tab = Shuffleboard.getTab("Turret");

    tab.addString("State", () -> state.name());
    tab.addBoolean("At target state", () -> atTargetState());
  }

  @Override
  public void periodic() {
    Pose2d robot = Drive.getInstance().getPose();
    PosePublisher.publish("Turret (Local)", RobotConstants.localTurretPose(azimuth.localPosition()));
    PosePublisher.publish("Turret (Global)", RobotConstants.globalTurretPose(robot, azimuth.localPosition()));

    PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-turret");
    if (limelightProfilingEnabled) {
      limelightProfiler.update(poseEstimate);
    }

    if (poseEstimate != null && poseEstimate.tagCount > 1) {
      Pose2d estimated = poseEstimate.pose;
      PosePublisher.publish("Camera estimated turret pose", estimated);
      Angle azimuthAngle = azimuth.getValues().position;
      Pose2d robotPose = new Pose2d(
        estimated.getTranslation().minus(RobotConstants.ROBOT_TO_TURRET.toTranslation2d()),
        estimated.getRotation().minus(new Rotation2d(azimuthAngle)));

      PosePublisher.publish("Camera estimated bot pose", robotPose);
      drive.addVisionMeasurement(robotPose, poseEstimate.timestampSeconds, VecBuilder.fill(0.5, 0.5, 100*drive.getGyroValues().yawVelociy.in(RotationsPerSecond)));
    }
    
    //TODO: Maybe try moving this to fastPeriodic() if it isn't too much of a performance hit
    turretPose = RobotConstants.globalTurretPose(Drive.getInstance().getPose(), azimuth.getValues().position).toPose2d();
    Translation2d translationToHub = TurretTargetsSupplier.projectedAllianceHub().minus(turretPose.getTranslation());
    SmartDashboard.putNumber("Turret distance (meters)", translationToHub.getNorm());

    PosePublisher.publish("Estimated turret pose", turretPose);
  }

  @Override
  public void fastPeriodic() {
    switch (state) {
      case STOW:
        azimuth.setSetpoint(Rotations.of(0.25));
        hood.setSetpoint(hood.getMinPosition());
        shooter.setSetpoint(RotationsPerSecond.of(0.0));
        break;
      case ALLOW_EXTERNAL_CONTROL:
        break;
      case ALLOW_EXTERNAL_CONTROL_FACING_HUB:
        faceHub(turretPose);
        break;
      case FACE_HUB:
        faceHub(turretPose);
        hood.setSetpoint(hood.getMinPosition());
        shooter.setSetpoint(RotationsPerSecond.of(0.0));
        break;
      case TARGET_HUB:
        faceHub(turretPose);
        targetHub(turretPose);
        break;
      case FACE_ALLIANCE_WALL:
        faceAllianceWall(turretPose);
        shooter.setSetpoint(RotationsPerSecond.of(40));
        hood.setSetpoint(Rotations.of(0.075));
        break;
      case FACE_ALLIANCE_WALL_SOTM:
        SwerveDriveState driveState = Drive.getInstance().getState();
        ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(driveState.Speeds, driveState.Pose.getRotation());
        faceAllianceWallSOTM(turretPose, new Translation2d(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond));
        shooter.setSetpoint(RotationsPerSecond.of(40));
        hood.setSetpoint(Rotations.of(0.075));
        break;
      case TARGET_HUB_FROM_POSE:
        Pose2d manualTurretPose = RobotConstants.globalTurretPose(manualRobotPose, azimuth.getValues().position).toPose2d();
        faceHub(manualTurretPose);
        targetHub(manualTurretPose);
        break;
    }
  }

  public boolean atTargetState() {
    switch (state) {
      case STOW:
        return azimuth.nearSetpoint(Rotations.of(0.01));
      case FACE_HUB:
        return azimuth.nearSetpoint(Rotations.of(0.1));
      case TARGET_HUB:
        return azimuth.nearSetpoint(Rotations.of(0.075))
            && shooter.nearSetpoint(RotationsPerSecond.of(4))
            && hood.nearSetpoint(Rotations.of(0.1));
      case FACE_ALLIANCE_WALL:
        return azimuth.nearSetpoint(Rotations.of(0.1));
      case FACE_ALLIANCE_WALL_SOTM:
        return azimuth.nearSetpoint(Rotations.of(0.2));
      default:
        return true;
    }
  }

  /** 
   * Turns the azimuth to face your alliance hub given an estimated turret pose
   * 
   * @param turretPose estimated turret pose
   */
  private void faceHub(Pose2d turretPose) {
    Translation2d translationToHub = TurretTargetsSupplier.projectedAllianceHub().minus(turretPose.getTranslation());
    Rotation2d rotationError = translationToHub.getAngle().minus(turretPose.getRotation());

    SmartDashboard.putNumber("Turret distance (meters)", translationToHub.getNorm()); // getNorm does return distance in meters it isn't documented but translations are in meters

    azimuth.setSetpoint(azimuth.getValues().position.plus(rotationError.getMeasure()));
  }

  private void faceAllianceWall(Pose2d turretPose) {
    Rotation2d rotationError = TurretTargetsSupplier.faceAllianceWall().minus(turretPose.getRotation());

    azimuth.setSetpoint(azimuth.getValues().position.plus(rotationError.getMeasure()));
  }

  private void faceAllianceWallSOTM(Pose2d turretPose, Translation2d turretVelocity) {
    Translation2d shotVelocity = new Translation2d(4, turretPose.getRotation()).plus(turretVelocity);
    Rotation2d rotationError = TurretTargetsSupplier.faceAllianceWall().minus(shotVelocity.getAngle());

    azimuth.setSetpoint(azimuth.getValues().position.plus(rotationError.getMeasure()));
  }

  private void targetHub(Pose2d turretPose) {
    Translation2d translationToHub = TurretTargetsSupplier.projectedAllianceHub().minus(turretPose.getTranslation());
    Distance turretDistance = Meters.of(translationToHub.getNorm());

    Angle hoodSetpoint = TurretTargeter.targetHubHood(turretDistance);
    AngularVelocity shooterSetpoint = TurretTargeter.targetHubShooter(turretDistance);

    hood.setSetpoint(hoodSetpoint);
    shooter.setSetpoint(shooterSetpoint);
  }

  /**
   * Stows the turret and waits until fully stowed
   * 
   * @return a command that stows the turret and waits until fully stowed
   */
  public Command stow() {
    return Commands.runOnce(() -> {
      state = TurretState.STOW;
    }, this, azimuth, hood, shooter)
    .andThen(Commands.waitUntil(this::atTargetState));
  }

  /**
   * Allows external control of azimuth, shooter, and hood using their setSetpoint methods
   * 
   * @return a command that allows external control of azimuth, shooter, and hood using their setSetpoint methods
   */
  public Command allowExternalControl() {
    return Commands.runOnce(() -> {
      state = TurretState.ALLOW_EXTERNAL_CONTROL;
    }, this)
    .andThen(Commands.waitUntil(this::atTargetState));
  }

  /**
   * Allows external control of shooter and hood using their setSetpoint methods while looking at the hub with azimuth
   * 
   * @return a command that allows external control of shooter and hood using their setSetpoint methods while looking at the hub with azimuth
   */
  public Command allowExternalControlFacingHub() {
    return Commands.runOnce(() -> {
      state = TurretState.ALLOW_EXTERNAL_CONTROL_FACING_HUB;
    }, this, azimuth)
    .andThen(Commands.waitUntil(this::atTargetState));
  }

  /**
   * Faces the azimuth at the alliance hub and waits until facing the hub
   * 
   * @return a command that faces the azimuth at the aliance hub and waits until facing the hub
   */
  public Command faceHub() {
    return Commands.runOnce(() -> {
      state = TurretState.FACE_HUB;
    }, this, azimuth)
    .andThen(Commands.waitUntil(this::atTargetState));
  }

  /**
   * Targets the alliance hub with the azimuth, shooter, and hood, and waits until targetting accurately
   * 
   * @return a command that targets the alliance hub with the azimuth, shooter, and hood, and waits until targetting accurately
   */
  public Command targetHub() {
    return Commands.runOnce(() -> {
      state = TurretState.TARGET_HUB;
    }, this, azimuth, hood, shooter)
    .andThen(Commands.waitUntil(this::atTargetState));
  }

  public Command faceAllianceWall() {
    return Commands.runOnce(() -> {
      state = TurretState.FACE_ALLIANCE_WALL;
    }, this, azimuth, hood, shooter)
    .andThen(Commands.waitUntil(this::atTargetState));
  }

  public Command faceAllianceWallSOTM() {
    return Commands.runOnce(() -> {
      state = TurretState.FACE_ALLIANCE_WALL_SOTM;
    }, this, azimuth, hood, shooter)
    .andThen(Commands.waitUntil(this::atTargetState)); 
  }

  public Command targetHubFromPose(Pose2d robotPose) {
    return Commands.runOnce(() -> {
      state = TurretState.TARGET_HUB_FROM_POSE;
      manualRobotPose = robotPose;
    }, this, azimuth, hood, shooter)
    .andThen(Commands.waitUntil(this::atTargetState));
  }

  public void setLimelightProfilingEnabled(boolean enabled) {
    limelightProfilingEnabled = enabled;
  }
}
