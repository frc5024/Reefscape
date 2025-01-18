package frc.lib.camera;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * 
 */
public class Camera {
    public enum Type {
        APRILTAG, COLOURED_SHAPE
    }

    public enum Processor {
        PHOTONVISION, LIMELIGHT
    }

    private final String name;
    private final Type type;
    private final Processor processor;
    private final int pipeline_index;
    private final double x;
    private final double y;
    private final double z;
    private final double roll;
    private final double pitch;
    private final double yaw;

    private final Transform3d cameraToRobot;

    /**
     * Units in Meters and Radians
     */
    public Camera(String name, Type type, Processor processor, int pipeline_index, double x, double y, double z,
            double roll, double pitch, double yaw) {
        this.name = name;
        this.processor = processor;
        this.type = type;
        this.pipeline_index = pipeline_index;
        this.x = x;
        this.y = y;
        this.z = z;
        this.roll = roll;
        this.pitch = pitch;
        this.yaw = yaw;

        this.cameraToRobot = new Transform3d(
                new Translation3d(this.x, this.y, this.z),
                new Rotation3d(this.roll, this.pitch, this.yaw));
    }

    /**
     * Getters and Setters
     */
    public String getName() {
        return this.name;
    }

    public Type getType() {
        return this.type;
    }

    public Processor getProcessor() {
        return this.processor;
    }

    public int getPipelineIndex() {
        return this.pipeline_index;
    }

    public double getHeight() {
        return this.z;
    }

    public double getPitch() {
        return this.pitch;
    }

    public double getYaw() {
        return this.yaw;
    }

    public Transform3d getCameraToRobot() {
        return this.cameraToRobot;
    }

    public Transform3d getRobotToCamera() {
        return this.cameraToRobot.inverse();
    }
}
