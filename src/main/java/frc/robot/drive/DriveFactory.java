package frc.robot.drive;

import frc.lib.swerves.IdealSwerveSim;
import frc.lib.swerves.PhoenixSwerve;
import frc.lib.swerves.SwerveOutput;
import frc.robot.HardwareManager;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class DriveFactory {

  public static SwerveOutput createSwerve() {
    if (HardwareManager.isEnabled(HardwareManager.Hardware.SWERVE)) {
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
