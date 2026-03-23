package frc.robot.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** Class with static methods to help with getting chassis speeds to drive the drive subsystem */
public class DriveSpeedsUtils {

  /**
   * Gets field relative chassis speeds from controller input with some max linear and angular velocity
   * 
   * @param controller Controller to grab inputs from
   * @param maxLinearVelocity Maximum desired linear velocity
   * @param maxAngularVelocity Maximum desired angular velocity
   * @param deadzone Controller deadzone, by default uses deadzone of 0.1
   * @return Field relative chassis speeds constructed from controller input with some max linear and angular velocity
   */
  public static ChassisSpeeds fromController(CommandXboxController controller, LinearVelocity maxLinearVelocity, AngularVelocity maxAngularVelocity, double deadzone) {
    Translation2d translation = new Translation2d(-controller.getLeftY(), -controller.getLeftX());
    double deadzoneSlope = 1/(1-deadzone);
    double deadzonedTranslationMagnitude = Math.max(0, deadzoneSlope * translation.getNorm() - deadzoneSlope * deadzone);
    Translation2d deadzonedTranslation = translation.div(translation.getNorm() * deadzonedTranslationMagnitude);

    LinearVelocity vx = maxLinearVelocity.times(deadzonedTranslation.getX());
    LinearVelocity vy = maxLinearVelocity.times(deadzonedTranslation.getY());

    double rotationMagnitude = -controller.getRightX();
    double deadzonedRotationMagnitude = Math.copySign(Math.max(0, deadzoneSlope * Math.abs(rotationMagnitude) - deadzoneSlope * deadzone), rotationMagnitude);

    AngularVelocity vomega = maxAngularVelocity.times(deadzonedRotationMagnitude);

    return new ChassisSpeeds(vx, vy, vomega);
  }

  /**
   * Gets field relative chassis speeds from controller input with some max linear and angular velocity
   * 
   * @param controller Controller to grab inputs from
   * @param maxLinearVelocity Maximum desired linear velocity
   * @param maxAngularVelocity Maximum desired angular velocity
   * @return Field relative chassis speeds constructed from controller input with some max linear and angular velocity
   */
  public static ChassisSpeeds fromController(CommandXboxController controller, LinearVelocity maxLinearVelocity, AngularVelocity maxAngularVelocity) {
    return fromController(controller, maxLinearVelocity, maxAngularVelocity, 0.1);
  }
}
