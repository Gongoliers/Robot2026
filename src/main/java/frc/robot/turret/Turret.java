package frc.robot.turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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

  // Variables used by control states

  /** Hub target */
  private Translation2d hubTarget;

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

    hubTarget = new Translation2d(0, 0);

    LimelightHelpers.setCameraPose_RobotSpace("turret", -0.115913, 0.080866, 0.734112, 0, 15, 0);
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
  }

  @Override
  public void fastPeriodic() {
    // For now, pose estimation will be done in 2d, since it's what odometry supports and should work ok
    PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("turret");

    Pose2d turretPose = new Pose2d();

    if (poseEstimate != null && poseEstimate.tagCount > 1) {
      turretPose = poseEstimate.pose;
      Angle azimuthAngle = azimuth.getValues().position;
      Pose2d robotPose = new Pose2d(
        turretPose.getTranslation().minus(RobotConstants.ROBOT_TO_TURRET.toTranslation2d()),
        turretPose.getRotation().minus(new Rotation2d(azimuthAngle)));

      Drive.getInstance().addVisionMeasurement(robotPose, poseEstimate.timestampSeconds);
    } else {
      turretPose = RobotConstants.globalTurretPose(Drive.getInstance().getPose(), azimuth.getValues().position).toPose2d();
    }

    PosePublisher.publish("Estimated turret pose", turretPose);

    switch (state) {
      case STOW:
        if (azimuth.getCurrentCommand() == null) {azimuth.setSetpoint(Rotations.of(0.0));};
        if (hood.getCurrentCommand() == null) {hood.setSetpoint(hood.getMinPosition());};
        if (shooter.getCurrentCommand() == null) {shooter.setSetpoint(RotationsPerSecond.of(0.0));};
        break;
      case SCORING:
        targetHub();
        break;
    }
  }

  /** Autoaim the turret to shoot in the hub, automagically firing when locked on */
  private void targetHub() {
    // gurt
  }

  /**
   * Stows the turret
   * 
   * @return a command that stows the turret while running
   */
  public Command stow() {
    return Commands.run(() -> {
      state = TurretState.STOW;
    }, this);
  }

  /**
   * Targets a position on the field to try and fire hub shots at
   * 
   * @param targetPosition 2d projection of the positon to aim ait (will aim to fire about 7ft above the given point)
   * @return a command that targets a position on the field to fire hub shots at
   */
  public Command scoreAtTarget(Translation2d targetPosition) {
    return Commands.run(() -> {
      hubTarget = targetPosition;
      state = TurretState.SCORING;
    }, this, azimuth, hood, shooter);
  }
}
