package frc.robot.turret;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.function.Supplier;

/** Class with static methods to get turret targets on the field */
public class TurretTargetsSupplier {

  private static boolean isRed() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
  }
  
  /** Gets the 2d translation of your alliance hub */
  public static Translation2d projectedAllianceHub() {
    if (isRed()) {
      return new Translation2d(
        Inches.of(468.56),
        Inches.of(158.32));
    }

    return new Translation2d(
      Inches.of(181.56),
      Inches.of(158.32));
  }

  public static Rotation2d faceAllianceWall() {
    if (isRed()) {
      return Rotation2d.fromDegrees(0);
    }

    return Rotation2d.fromDegrees(180);
  }

  public static final Distance FIELD_LENGTH = Inches.of(651.22);

  public static final Distance PASS_DISTANCE = Feet.of(2);

  public static final Distance BLUE_PASS_X = PASS_DISTANCE;

  public static final Distance RED_PASS_X = FIELD_LENGTH.minus(BLUE_PASS_X);

  public static Translation2d passFrom(Pose2d pose) {
    Distance x = isRed() ? RED_PASS_X : BLUE_PASS_X;
    Distance y = pose.getMeasureY();
    return new Translation2d(x, y);
  }

}
