package frc.robot.turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.MultithreadedSubsystem;
import frc.lib.PosePublisher;
import frc.robot.LimelightHelpers;
import frc.robot.RobotConstants;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.azimuth.Azimuth;
import frc.robot.drive.Drive;
import frc.robot.hood.Hood;
import frc.robot.shooter.Shooter;

import java.util.function.Supplier;

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

  /** Current turret state */
  private TurretState state;

  /** Cache the drive pose to avoid costly pose estimator recalculation. */
  private volatile Pose2d drivePose;

  /** Estimated turret pose */
  private volatile Pose2d turretPose;

  /** Fixed drive pose, for TARGET_HUB_FIXED_POSE. */
  private volatile Pose2d fixedDrivePose = new Pose2d();

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

    state = TurretState.STOW;

    LimelightHelpers.setCameraPose_RobotSpace("limelight-turret", -0.115913, 0.080866, 0.734112, 0, 15, 0);
  }

  @Override 
  public void initializeTab() {
    ShuffleboardTab tab = Shuffleboard.getTab("Turret");

    tab.addString("State", () -> state.name());
  }

  @Override
  public void periodic() {
    Pose2d robot = Drive.getInstance().getPose();
    PosePublisher.publish("Turret (Local)", RobotConstants.localTurretPose(azimuth.localPosition()));
    PosePublisher.publish("Turret (Global)", RobotConstants.globalTurretPose(robot, azimuth.localPosition()));

    PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-turret");

    if (poseEstimate != null && poseEstimate.tagCount > 1) {
      Pose2d estimated = poseEstimate.pose;
      PosePublisher.publish("Camera estimated turret pose", estimated);
      Angle azimuthAngle = azimuth.getValues().position;
      Pose2d robotPose = new Pose2d(
        estimated.getTranslation().minus(RobotConstants.ROBOT_TO_TURRET.toTranslation2d()),
        estimated.getRotation().minus(new Rotation2d(azimuthAngle)));

      PosePublisher.publish("Camera estimated bot pose", robotPose);
      Drive.getInstance().addVisionMeasurement(robotPose, poseEstimate.timestampSeconds);
    }
    
    //TODO: Maybe try moving this to fastPeriodic() if it isn't too much of a performance hit
    drivePose = Drive.getInstance().getPose();
    turretPose = RobotConstants.globalTurretPose(drivePose, azimuth.getValues().position).toPose2d();
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
      case TARGET_HUB_FIXED_POSE:
        Pose2d fixedTurretPose = RobotConstants.globalTurretPose(fixedDrivePose, azimuth.getValues().position).toPose2d();
        faceHub(fixedTurretPose);
        targetHub(fixedTurretPose);
        break;
    }
  }

  public boolean atTargetState() {
    switch (state) {
      case STOW:
        return azimuth.nearSetpoint(Rotations.of(0.01));
      case FACE_HUB:
        return azimuth.nearSetpoint(Rotations.of(0.1))
            && azimuth.getValues().velocity.abs(RotationsPerSecond) < 0.2;
      case TARGET_HUB:
      case TARGET_HUB_FIXED_POSE:
        return azimuth.nearSetpoint(Rotations.of(0.075))
            && shooter.nearSetpoint(RotationsPerSecond.of(4))
            && hood.nearSetpoint(Rotations.of(0.1));
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

    Distance turretDistance = Meters.of(translationToHub.getNorm());
    Angle hoodSetpoint = TurretTargeter.targetHubHood(turretDistance);

    hood.setSetpoint(hoodSetpoint);
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

  public Command targetHubFrom(Supplier<Pose2d> drivePose) {
    return Commands.runOnce(() -> {
              state = TurretState.TARGET_HUB_FIXED_POSE;
              fixedDrivePose = drivePose.get();
            }, this, azimuth, hood, shooter)
            .andThen(Commands.waitUntil(this::atTargetState));
  }
}
