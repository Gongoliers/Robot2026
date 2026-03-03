// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.PosePublisher;
import frc.robot.drive.Drive;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import java.io.IOException;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

public class Robot extends TimedRobot {

  /** Robot container */
  private final RobotContainer robotContainer;

  /** Command to run during autonomous */
  private Command autonomousCommand;

  private final VisionSystemSim visionSim;

  private final PhotonCamera camera;

  private final AprilTagFieldLayout tagLayout;

  private final Transform3d robotToCamera;

  public Robot() {
    robotContainer = RobotContainer.getInstance();

    visionSim = new VisionSystemSim("main");
    tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    visionSim.addAprilTags(tagLayout);
    SimCameraProperties cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
    cameraProp.setCalibError(0.25, 0.08);
    cameraProp.setFPS(20);
    cameraProp.setAvgLatencyMs(35);
    cameraProp.setLatencyStdDevMs(5);
    camera = new PhotonCamera("camera");
    PhotonCameraSim cameraSim = new PhotonCameraSim(camera, cameraProp);
    var robotToCameraTrl = new Translation3d(0.1, 0, 0.5);
    Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-15), 0);
    robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);
    visionSim.addCamera(cameraSim, robotToCamera);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    var pose = Drive.getInstance().getPose();
    visionSim.update(pose);
    var results = camera.getAllUnreadResults();
    for (var result : results) {
      var multiTagResult = result.getMultiTagResult();
      if (multiTagResult.isPresent()) {
        var fieldToCamera = multiTagResult.get().estimatedPose.best;
        var estimate = new Pose3d().plus(fieldToCamera).plus(robotToCamera.inverse());
        PosePublisher.publish("Pose Estimate", estimate);
        Drive.getInstance().addPoseEstimate(estimate.toPose2d(), result.metadata.captureTimestampMicros * 1000);
      }
    }
  }

  private static final Distance Z_TOL = Inches.of(2);

  private boolean estimateIsGood(Pose3d current, Pose3d estimate) {
    // TODO Implement more filters
    boolean onFloor = estimate.getMeasureZ().isNear(Meters.zero(), Z_TOL);
    SmartDashboard.putBoolean("On Floor?", onFloor);
    return onFloor;
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
