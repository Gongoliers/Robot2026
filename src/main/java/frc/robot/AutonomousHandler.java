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

  public static AutonomousHandler getInstance() {
    if (instance == null) {
      instance = new AutonomousHandler();
    }

    return instance;
  }

  private AutonomousHandler() {
    // SUPER SKETCHY!!! but `new RobotConfig` crashed because of null args, and robotConfig needs to be defined
    RobotConfig robotConfig = null;
    try {
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      DriverStation.reportError("No autos allowed", true);
    }

    AutoBuilder.configure(
      drive::getPose,
      drive::resetPose,
      drive::getRobotRelativeSpeeds,
      (speeds, feedforwards) -> drive.driveRobotRelative(speeds),
      new PPHolonomicDriveController(
        new PIDConstants(5, 0, 0),
        new PIDConstants(5, 0, 0)), 
      robotConfig, 
      () -> DriverStation.getAlliance().filter(value -> value == Alliance.Red).isPresent(),
      drive);

    // Set up named commands
    NamedCommands.registerCommand("FaceHub", superstructure.faceHub());
    NamedCommands.registerCommand("Intake", superstructure.intake());
    NamedCommands.registerCommand("Score", superstructure.score().andThen(Commands.waitSeconds(3)));
    
    // Publish auto chooser
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("PathPlanner Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
