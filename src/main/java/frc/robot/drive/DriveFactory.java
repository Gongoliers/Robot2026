package frc.robot.drive;

import frc.lib.swerves.PhoenixSwerve;
import frc.lib.swerves.SwerveOutput;

public class DriveFactory {

  public static SwerveOutput createSwerve() {
      return new PhoenixSwerve(
        TunerConstants.DrivetrainConstants,
        TunerConstants.FrontLeft,
        TunerConstants.FrontRight,
        TunerConstants.BackLeft,
        TunerConstants.BackRight);
  }
}
