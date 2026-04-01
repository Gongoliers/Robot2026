package frc.lib;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

/** Class used to get useful data about a limelight's vision measurements */
public class LimelightProfiler {

  /** Limelight name */
  private final String name;

  /** Size of measurement window */
  private int windowSize;

  /** Array list of pose estimates that is used to calculate averages and standard deviations */
  private final ArrayList<PoseEstimate> posesWindow = new ArrayList<>();
  
  /**
   * Constructs a limelight profiler
   * 
   * @param limelightName Name of limelight to be profiled
   * @param measurementWindow Width of window used to determine average measurements and standard devation (100 by default)
   */
  public LimelightProfiler(String limelightName, int measurementWindow) {
    name = limelightName;
    windowSize = measurementWindow;
  }

  /**
   * Constructs a limelight profiler
   * 
   * @param limelightName Name of limelight to be profiled
   */
  public LimelightProfiler(String limelightName) {
    this(limelightName, 100);
  }

  /** 
   * Given a pose estimate logs analysis (If you already got a pose estimate from the limelight recently, why do it again just for analysis)
   * 
   * @param poseEstimate to perform analysis on
   */
  public void update(PoseEstimate poseEstimate) {
    if (poseEstimate != null) {
      posesWindow.add(0, poseEstimate);

      while (posesWindow.size() > windowSize) {
        posesWindow.remove(posesWindow.size()-1);
      }

      // Calculate means
      int currentWindowSize = posesWindow.size();
      double xSum = 0;
      double ySum = 0;
      double cosSum = 0;
      double sinSum = 0;
      for (PoseEstimate estimate : posesWindow) {
        xSum += estimate.pose.getX();
        ySum += estimate.pose.getY();
        cosSum += estimate.pose.getRotation().getCos();
        sinSum += estimate.pose.getRotation().getSin();
      }

      double xMean = xSum / currentWindowSize;
      double yMean = ySum / currentWindowSize;
      double cosMean = cosSum / currentWindowSize;
      double sinMean = sinSum / currentWindowSize;
      double circularMean = Math.atan2(sinMean, cosMean);
      
      // Calculate standard deviations
      double meanResultantLength = Math.hypot(cosMean, sinMean);
      double circularStdDev = Math.sqrt(Math.log(1/(meanResultantLength*meanResultantLength)));

      double xDevSum = 0;
      double yDevSum = 0;
      for (PoseEstimate estimate : posesWindow) {
        xDevSum += Math.abs(estimate.pose.getX() - xMean);
        yDevSum += Math.abs(estimate.pose.getY() - yMean);
      }

      double xStdDev = xDevSum / currentWindowSize;
      double yStdDev = yDevSum / currentWindowSize;

      // Log
      PosePublisher.publish(name+"/Pose Estimate", poseEstimate.pose);
      SmartDashboard.putNumber(name+"/Current Window Size", currentWindowSize);
      PosePublisher.publish(name+"/Mean Pose Estimate", new Pose2d(xMean, yMean, Rotation2d.fromRadians(circularMean)));
      SmartDashboard.putNumber(name+"/x Std Deviation (m)", xStdDev);
      SmartDashboard.putNumber(name+"/y Std Deviation (m)", yStdDev);
      SmartDashboard.putNumber(name+"/Translational Std Deviation (m)", Math.hypot(xStdDev, yStdDev));
      SmartDashboard.putNumber(name+"/Angular Std Deviation (rad)", circularStdDev);
    }
  }

  /** Gets a new pose estimate and logs new analysis */
  public void update() {
    PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
    update(poseEstimate);
  }

  /**
   * Change the measurement window size
   * 
   * @param newWindowSize New measurement window size
   */
  public void setWindowSize(int newWindowSize) {
    windowSize = newWindowSize;
  }

  /** Clears the measurement window of all measurements */
  public void clearWindow() {
    posesWindow.clear();
  }
}
