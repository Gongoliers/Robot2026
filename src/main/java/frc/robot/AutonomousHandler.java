package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.drive.Drive;
import frc.robot.superstructure.Superstructure;

public class AutonomousHandler {
  
  private static AutonomousHandler instance = null;

  private final Drive drive = Drive.getInstance();

  private final Superstructure superstructure = Superstructure.getInstance();

  public static AutonomousHandler getInstance() {
    if (instance == null) {
      instance = new AutonomousHandler();
    }

    return instance;
  }

  private AutonomousHandler() {
    // Set up auto builder, sendable chooser for picking autos, and named commands in here
  }

  private Pose2d getPose() {
    return drive.getPose();
  }

  private void resetPose(Pose2d newPose) {
    drive.resetPose(newPose);
  }

  private ChassisSpeeds getRobotRelativeSpeeds() {
    SwerveDriveState driveState = drive.getState();
    return ChassisSpeeds.fromFieldRelativeSpeeds(driveState.Speeds, driveState.Pose.getRotation());
  }
}
