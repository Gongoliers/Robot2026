package frc.robot.visualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.PosePublisher;

import static edu.wpi.first.units.Units.*;

public class TurretVisualizer {
    /** Transform from the robot center to the turret center */
    // TODO Update these measurements with measurements from CAD
    private static final Transform3d ROBOT_TO_TURRET = new Transform3d(Inches.of(10), Inches.of(10), Inches.of(0.0), Rotation3d.kZero);

    /** Length of the hood. Used for visualization. */
    private static final Distance HOOD_LENGTH = Inches.of(24);

    /** Mechanism for displaying the hood. */
    private static final Mechanism2d HOOD_MECHANISM = new Mechanism2d(1, 1);

    /** Root for the hood mechanism. */
    private static final MechanismRoot2d HOOD_ROOT = HOOD_MECHANISM.getRoot("HoodRoot", 0.5, 0);

    /** Ligament for the hood mechanism. */
    private static final MechanismLigament2d HOOD = HOOD_ROOT.append(new MechanismLigament2d("Hood", HOOD_LENGTH.in(Meters), 0.0));

    /** Updates the turret visualization. */
    public static void update(Pose3d robotPose, Angle rotation, Angle inclination) {
        Pose3d turretPose = robotPose.transformBy(ROBOT_TO_TURRET);
        Rotation3d instrinsticYawRotation = new Rotation3d(Degrees.zero(), Degrees.zero(), rotation);
        Pose3d rotatedTurretPose = turretPose.rotateAround(turretPose.getTranslation(), instrinsticYawRotation);

        // Hood inclination is reflected over the Z axis because it is opposite the turret angle
        HOOD.setAngle(180 - inclination.in(Degrees));

        // Publish the updates
        PosePublisher.publish("Turret", rotatedTurretPose);
        SmartDashboard.putData("Hood Mechanism", HOOD_MECHANISM);
    }

    /** Updates the turret visualization. */
    public static void update(Pose2d robotPose, Angle rotation, Angle inclination) {
        update(new Pose3d(robotPose), rotation, inclination);
    }
}
