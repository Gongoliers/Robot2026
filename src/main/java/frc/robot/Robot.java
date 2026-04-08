// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.PosePublisher;
import frc.robot.configuration.FieldRegion;
import frc.robot.configuration.Objective;
import frc.robot.drive.Drive;

import java.util.Arrays;

public class Robot extends TimedRobot {

  /** Robot container */
  private final RobotContainer robotContainer;

  /** Command to run during autonomous */
  private Command autonomousCommand;

  public Robot() {
    robotContainer = RobotContainer.getInstance();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // Get the robot's current pose to determine what action it should take
    Pose2d pose = Drive.getInstance().getPose();

    // Publish some helper information for debugging
    PosePublisher.publish("Corners", FieldRegion.allCorners());
    PosePublisher.publish("Drive Pose", pose);

    // Determine the region the robot is in
    FieldRegion[] candidates = FieldRegion.containing(pose.getTranslation());
    // FieldRegion.FIELD is reserved for when the robot is not in any other region
    FieldRegion region = Arrays.stream(candidates).filter(candidate -> candidate != FieldRegion.FIELD).findFirst().orElse(FieldRegion.FIELD);
    SmartDashboard.putString("Region", region.name());

    // Find the objective should be based on our alliance
    DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    Objective objective = region.objective(alliance);

    // Publish the action and the scoring target
    SmartDashboard.putString("Action", objective.action().name());
    PosePublisher.publish("Scoring Target", objective.target().position());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
