package frc.lib;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayEntry;

import java.util.Arrays;
import java.util.HashMap;

public class PosePublisher {

    private final NetworkTableInstance instance;
    private final HashMap<String, StructArrayEntry<Pose3d>> entries;

    public PosePublisher() {
        this.instance = NetworkTableInstance.getDefault();
        this.entries = new HashMap<>();
    }

    private StructArrayEntry<Pose3d> getEntry(String key) {
        if (entries.containsKey(key)) {
            return entries.get(key);
        }

        StructArrayEntry<Pose3d> entry = instance.getStructArrayTopic(key, Pose3d.struct).getEntry(new Pose3d[]{});
        entries.put(key, entry);
        return entry;
    }

    public void publish(String key, Pose3d[] poses) {
        getEntry(key).set(poses);
    }

    public void publish(String key, Pose3d pose) {
        publish(key, new Pose3d[]{pose});
    }

    public void publish(String key, Translation3d translation) {
        publish(key, PoseUtils.translation3DToPose3D(translation));
    }

    public void publish(String key, Translation3d[] translations) {
        publish(key, Arrays.stream(translations).map(PoseUtils::translation3DToPose3D).toArray(Pose3d[]::new));
    }

    public void publish(String key, Pose2d pose) {
        publish(key, new Pose3d(pose));
    }

    public void publish(String key, Translation2d translation) {
        publish(key, PoseUtils.translation2DToPose2D(translation));
    }

    public void publish(String key, Pose2d[] poses) {
        publish(key, Arrays.stream(poses).map(Pose3d::new).toArray(Pose3d[]::new));
    }

    public void publish(String key, Translation2d[] translations) {
        publish(key, Arrays.stream(translations).map(PoseUtils::translation2DToPose2D).toArray(Pose2d[]::new));
    }

}
