package frc.robot.drive;

import frc.lib.swerves.IdealSwerveSim;
import frc.lib.swerves.PhoenixSwerve;
import frc.lib.swerves.SwerveOutput;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class DriveFactory {

  public static SwerveOutput createSwerve() {
    if (Robot.isReal() && RobotConstants.ENABLED_SUBSYSTEMS.contains(RobotConstants.Subsystem.SWERVE)) {
      return new PhoenixSwerve(
        TunerConstants.DrivetrainConstants,
        TunerConstants.FrontLeft,
        TunerConstants.FrontRight,
        TunerConstants.BackLeft,
        TunerConstants.BackRight);
    }

    return new IdealSwerveSim();
  }
}
