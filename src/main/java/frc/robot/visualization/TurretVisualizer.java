package frc.robot.visualization;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.PosePublisher;

import static edu.wpi.first.units.Units.*;

public class TurretVisualizer {
    /** Distances from the center of the turret to the robot's origin. */
    // TODO These were tuned by hand to get the turret visualization to line up; probably not accurate to CAD
    public static final Translation3d ROBOT_TO_TURRET = new Translation3d(Inches.of(2.075), Inches.of(-3.9), Inches.of(14.939).plus(Meters.of(0.075)));

    /** Length of the hood. Used for visualization. */
    private static final Distance HOOD_LENGTH = Inches.of(24);

    /** Mechanism for displaying the hood. */
    private static final Mechanism2d HOOD_MECHANISM = new Mechanism2d(1, 1);

    /** Root for the hood mechanism. */
    private static final MechanismRoot2d HOOD_ROOT = HOOD_MECHANISM.getRoot("HoodRoot", 0.5, 0);

    /** Ligament for the hood mechanism. */
    private static final MechanismLigament2d HOOD = HOOD_ROOT.append(new MechanismLigament2d("Hood", HOOD_LENGTH.in(Meters), 0.0));

    /** Constructs a rotation with the only specified yaw rotation. */
    public static Rotation3d yawRotation(Angle rotation) {
        return new Rotation3d(Degrees.zero(), Degrees.zero(), rotation);
    }

    /** Gets turret pose relative to the robot's pose. */
    public static Pose3d localTurretPose(Angle rotation) {
        return new Pose3d(ROBOT_TO_TURRET, yawRotation(rotation));
    }

    /** Gets the global rotated turret pose. */
    public static Pose3d globalTurretPose(Pose3d robotPose, Angle rotation) {
        return new Pose3d(robotPose.getTranslation().plus(ROBOT_TO_TURRET), robotPose.getRotation().plus(yawRotation(rotation)));
    }

    /** Gets the rotated turret pose relative to the robot's pose. */
    public static Pose3d globalTurretPose(Pose2d robotPose, Angle rotation) {
        return globalTurretPose(new Pose3d(robotPose), rotation);
    }

    /** Updates the turret visualization. */
    public static void update(Pose3d robotPose, Angle rotation, Angle inclination) {
        // Hood inclination is reflected over the Z axis because it is opposite the turret angle
        HOOD.setAngle(180 - inclination.in(Degrees));

        // Publish the updates
        PosePublisher.publish("Turret", globalTurretPose(robotPose, rotation));
        SmartDashboard.putData("Hood Mechanism", HOOD_MECHANISM);
    }

    /** Updates the turret visualization. */
    public static void update(Pose2d robotPose, Angle rotation, Angle inclination) {
        update(new Pose3d(robotPose), rotation, inclination);
    }
}
