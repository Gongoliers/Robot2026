package frc.robot.drive;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.Subsystem;
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
