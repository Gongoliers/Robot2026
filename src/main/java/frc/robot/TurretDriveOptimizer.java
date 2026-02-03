package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.configuration.TurretBounds;

import java.util.Objects;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

public class TurretDriveOptimizer {

    public enum Costs {
        EQUAL(1, 1),
        PREFER_DRIVE(1, 10),
        PREFER_TURRET(10, 1);

        private final double driveCost_;

        private final double turretCost_;

        Costs(double driveCost, double turretCost) {
            this.driveCost_ = driveCost;
            this.turretCost_ = turretCost;
        }

        public double driveCost() {
            return driveCost_;
        }

        public double turretCost() {
            return turretCost_;
        }

        public double totalCosts() {
            return driveCost_ + turretCost_;
        }
    }

    public record TurretDriveAngles(Angle turret, Angle drive) {
        public TurretDriveAngles {
            Objects.requireNonNull(turret);
            Objects.requireNonNull(drive);
        }
    }

    private static Angle clamp(Angle angle, Angle min, Angle max) {
        double clamped = MathUtil.clamp(angle.in(Radians), min.in(Radians), max.in(Radians));
        SmartDashboard.putNumber("clamped", clamped);
        return Radians.of(clamped);
    }

    private static Angle wrap(Angle angle) {
        double offsetRotations = angle.in(Rotations) + 0.5;
        double wrappedRotations = offsetRotations % 1;
        double unoffsetRotations = wrappedRotations - 0.5;
        return Rotations.of(unoffsetRotations);
    }

    public static TurretDriveAngles optimize(Angle drive, Angle turret, Angle turretToTarget, TurretBounds bounds, Costs costs) {
        // Initialize variables for tracking the least-costly turret and drive rotation
        double bestCost = Double.POSITIVE_INFINITY;
        Angle bestTurret = turret;
        Angle bestDrive = drive;

        // Perform multiple iterations, each dealing with a different multiple of the turret to target angle
        for (int k = -2; k <= 2; k++) {
            Angle multiple = Rotations.of(k);
            Angle unwrappedTurretToTarget = turretToTarget.plus(multiple);

            // Calculate the globally optimal turret angle (i.e. the minimum of the parabola formed by the intersection
            // of the constraint equation and the cost paraboloid)
            Angle penalizedTurretAngle = turret.times(costs.turretCost());
            Angle penalizedDriveAngle = unwrappedTurretToTarget.minus(drive).times(costs.driveCost());
            Angle unconstrainedOptimalTurretAngle = penalizedTurretAngle.plus(penalizedDriveAngle).div(costs.totalCosts());

            // The globally optimal turret angle might not be achievable with the desired bounds; snap to those bounds
            Angle optimalTurretAngle = clamp(unconstrainedOptimalTurretAngle, bounds.min(), bounds.max());
            // The drive angle will take up any remaining rotation
            Angle optimalDriveAngle = turretToTarget.minus(optimalTurretAngle);

            // Calculate the cost to achieve this combined turret and drive rotation
            Angle turretDistance = wrap(optimalTurretAngle.minus(turret));
            Angle driveDistance = wrap(optimalDriveAngle.minus(drive));
            double turretCost = costs.turretCost() * Math.pow(turretDistance.in(Radians), 2);
            double swerveCost = costs.driveCost() * Math.pow(driveDistance.in(Radians), 2);
            double cost = turretCost + swerveCost;

            // Save this combined turret and drive rotation if it is the least costly
            if (cost < bestCost) {
                bestCost = cost;
                bestTurret = optimalTurretAngle;
                bestDrive = optimalDriveAngle;
            }
        }

        // Return the least costly turret and drive rotation
        return new TurretDriveAngles(bestTurret, bestDrive);
    }

}
