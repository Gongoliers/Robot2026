package frc.robot;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.drive.Drive;
import frc.robot.superstructure.Superstructure;

public class AutonomousHandler {
  
  /** AutonomousHandler instance */
  private static AutonomousHandler instance = null;

  /** Drive reference */
  private final Drive drive = Drive.getInstance();

  /** Superstructure reference */
  private final Superstructure superstructure = Superstructure.getInstance();

  /** Used in dashboard to pick what autonomous routine to run */
  private final SendableChooser<Command> autoChooser;

  /** Used during autos to drive the robot with a drive.drive(Supplier<ChassisSpeeds>) commad to override default control */
  private ChassisSpeeds swerveSpeeds = new ChassisSpeeds();

  public static AutonomousHandler getInstance() {
    if (instance == null) {
      instance = new AutonomousHandler();
    }

    return instance;
  }

  private AutonomousHandler() {
    // Configure AutoBuilder
    RobotConfig robotConfig = new RobotConfig(1, 1, null, null);
    try {
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      DriverStation.reportError("No autos allowed", true);
    }

    AutoBuilder.configure(
      this::getPose, 
      this::resetPose, 
      this::getRobotRelativeSpeeds, 
      (speeds, feedforwards) -> driveRobotRelative(speeds), 
      new PPHolonomicDriveController(
        new PIDConstants(5, 0, 0), 
        new PIDConstants(5, 0, 0)), 
      robotConfig, 
      () -> {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == Alliance.Red;
        }
        return false;
      });

    // Set up named commands
    NamedCommands.registerCommand("FaceHub", superstructure.faceHub());
    NamedCommands.registerCommand("Intake", superstructure.intake());
    NamedCommands.registerCommand("Score", Commands.sequence(
      superstructure.score(),
      Commands.waitSeconds(3)
    ));
    
    // Publish auto chooser
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(autoChooser);
  }

  public Command getAutonomousCommand() {
    return Commands.race(
      autoChooser.getSelected(),
      drive.drive(() -> swerveSpeeds)
    );
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

  private void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    swerveSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, drive.getState().Pose.getRotation());
  }
}
