package frc.lib;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayEntry;
import edu.wpi.first.networktables.StructEntry;

import java.util.Arrays;
import java.util.HashMap;

public class PosePublisher {

    private final NetworkTableInstance instance;
    private final HashMap<String, StructEntry<Pose3d>> pose3ds;
    private final HashMap<String, StructArrayEntry<Pose3d>> pose3dArrays;
    private final HashMap<String, StructEntry<Pose2d>> pose2ds;
    private final HashMap<String, StructArrayEntry<Pose2d>> pose2dArrays;

    public PosePublisher() {
        this.instance = NetworkTableInstance.getDefault();
        this.pose3ds = new HashMap<>();
        this.pose3dArrays = new HashMap<>();
        this.pose2ds = new HashMap<>();
        this.pose2dArrays = new HashMap<>();
    }

    private StructEntry<Pose3d> get3dEntry(String key) {
        if (pose3ds.containsKey(key)) {
            return pose3ds.get(key);
        }

        StructEntry<Pose3d> entry = instance.getStructTopic(key, Pose3d.struct).getEntry(new Pose3d());
        pose3ds.put(key, entry);
        return entry;
    }

    private StructArrayEntry<Pose3d> get3dArrayEntry(String key) {
        if (pose3dArrays.containsKey(key)) {
            return pose3dArrays.get(key);
        }

        StructArrayEntry<Pose3d> entry = instance.getStructArrayTopic(key, Pose3d.struct).getEntry(new Pose3d[]{});
        pose3dArrays.put(key, entry);
        return entry;
    }

    private StructEntry<Pose2d> get2dEntry(String key) {
        if (pose2ds.containsKey(key)) {
            return pose2ds.get(key);
        }

        StructEntry<Pose2d> entry = instance.getStructTopic(key, Pose2d.struct).getEntry(new Pose2d());
        pose2ds.put(key, entry);
        return entry;
    }

    private StructArrayEntry<Pose2d> get2dArrayEntry(String key) {
        if (pose2dArrays.containsKey(key)) {
            return pose2dArrays.get(key);
        }

        StructArrayEntry<Pose2d> entry = instance.getStructArrayTopic(key, Pose2d.struct).getEntry(new Pose2d[]{});
        pose2dArrays.put(key, entry);
        return entry;
    }

    public void publish(String key, Pose3d pose) {
        get3dEntry(key).set(pose);
    }

    private Pose3d translation3dToPose3d(Translation3d translation) {
        return new Pose3d(translation, Rotation3d.kZero);
    }

    private Pose2d translation2dToPose2d(Translation2d translation) {
        return new Pose2d(translation, Rotation2d.kZero);
    }

    public void publish(String key, Translation3d translation) {
        publish(key, translation3dToPose3d(translation));
    }

    public void publish(String key, Pose3d[] poses) {
        get3dArrayEntry(key).set(poses);
    }

    public void publish(String key, Translation3d[] translations) {
        publish(key, Arrays.stream(translations).map(this::translation3dToPose3d).toArray(Pose3d[]::new));
    }

    public void publish(String key, Pose2d pose) {
        get2dEntry(key).set(pose);
    }

    public void publish(String key, Translation2d translation) {
        publish(key, translation2dToPose2d(translation));
    }

    public void publish(String key, Pose2d[] poses) {
        get2dArrayEntry(key).set(poses);
    }

    public void publish(String key, Translation2d[] translations) {
        publish(key, Arrays.stream(translations).map(this::translation2dToPose2d).toArray(Pose2d[]::new));
    }

}
