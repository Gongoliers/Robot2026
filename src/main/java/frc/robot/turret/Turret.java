package frc.robot.turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
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
    if (getCurrentCommand() != null) {
      SmartDashboard.putString("Turret.RunningCommand", getCurrentCommand().getName());
    }
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
        azimuth.setSetpoint(Rotations.of(-0.25));
        hood.setSetpoint(hood.getMinPosition());
        shooter.setSetpoint(RotationsPerSecond.of(0.0));
        break;
      case ALLOW_EXTERNAL_CONTROL:
        if (azimuth.getCurrentCommand() == null) {azimuth.setSetpoint(Rotations.of(-0.25));};
        if (hood.getCurrentCommand() == null) {hood.setSetpoint(hood.getMinPosition());};
        if (shooter.getCurrentCommand() == null) {shooter.setSetpoint(RotationsPerSecond.of(0.0));};
      case FACE_HUB:
        faceHub(turretPose);
        break;
      case TARGET_HUB:
        faceHub(turretPose);
        break;
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

    azimuth.setSetpoint(azimuth.getSetpoint().plus(rotationError.getMeasure()));
  }

  /**
   * Stows the turret
   * 
   * @return a command that stows the turret while it's running
   */
  public Command stow() {
    return Commands.run(() -> {
      state = TurretState.STOW;
    }, this, azimuth, hood, shooter).withName("stow");
  }

  /**
   * Stows the turret by default, but allows external control of the shooter, hood, and azimuth through other commands
   * 
   * @return a command that stows the turret by default, but allows external control of the turret by other commands while it's running
   */
  public Command allowExternalControl() {
    return Commands.run(() -> {
      state = TurretState.ALLOW_EXTERNAL_CONTROL;
    }, this);
  }

  /**
   * Faces the azimuth at the alliance hub
   * 
   * @return a command that faces the azimuth at the aliance hub while it's running
   */
  public Command faceHub() {
    return Commands.run(() -> {
      state = TurretState.FACE_HUB;
    }, this, azimuth, hood, shooter).withName("faceHub");
  }
}
