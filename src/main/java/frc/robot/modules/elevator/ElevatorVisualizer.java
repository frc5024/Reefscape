package frc.robot.modules.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorLevel;
import frc.robot.Constants.MechanismConstants;
import frc.robot.utils.MapleSimUtil;

/**
 * 
 */
public class ElevatorVisualizer {
    private static ElevatorVisualizer mInstance = null;
    private final ElevatorMechanism elevatorMechanism = new ElevatorMechanism();
    public final Translation2d elevatorOrigin2d = new Translation2d(0.10686, 0.06357);
    public final Translation3d elevatorOrigin3d = new Translation3d(elevatorOrigin2d.getX(), 0.0,
            elevatorOrigin2d.getY());

    private final String name;
    ElevatorLevel elevatorLevel;

    private Pose3d algaeIntakePose;
    private Pose3d coralIntakePose;

    /**
     * 
     */
    private ElevatorVisualizer(String name) {
        this.name = name;

        this.algaeIntakePose = new Pose3d();
        this.coralIntakePose = new Pose3d();
    }

    /**
     * 
     */
    public static ElevatorVisualizer getInstance(String name) {
        if (mInstance == null) {
            mInstance = new ElevatorVisualizer(name);
        }

        return mInstance;
    }

    /**
     * 
     */
    public void update(double positionInMeters, ElevatorLevel elevatorLevel, boolean hasAlgae, boolean hasCoral) {
        this.elevatorMechanism.setLength(positionInMeters);
        this.elevatorLevel = elevatorLevel;

        double[] heights = this.elevatorMechanism.getShaftLengths();
        double elevatorAngleInRadians = ElevatorConstants.ANGLE.getRadians();

        Logger.recordOutput("Mechanism2d", MechanismConstants.CANVAS);
        Logger.recordOutput("Mechanism3d/" + name,
                new Pose3d[] {
                        new Pose3d(), // outer frame
                        new Pose3d(new Translation3d(heights[0], new Rotation3d(0.0, -elevatorAngleInRadians, 0.0)),
                                new Rotation3d()), // middle frame
                        new Pose3d(new Translation3d(heights[1], new Rotation3d(0.0, -elevatorAngleInRadians, 0.0)),
                                new Rotation3d()), // inner frame
                        new Pose3d(new Translation3d(heights[2], new Rotation3d(0.0, -elevatorAngleInRadians, 0.0)),
                                new Rotation3d()), // box
                        new Pose3d(new Translation3d(heights[2], new Rotation3d(0.0, -elevatorAngleInRadians, 0.0)),
                                new Rotation3d()), // algae intake
                        new Pose3d(new Translation3d(heights[2], new Rotation3d(0.0, -elevatorAngleInRadians, 0.0)),
                                new Rotation3d()) // coral intake
                });

        /* Algae Intake Subsystem */
        if (hasAlgae) {
            Pose2d robotPoseA = MapleSimUtil.getSwerveDriveSimulation().getSimulatedDriveTrainPose();
            Transform3d algaeTransform = new Transform3d(Units.inchesToMeters(-13.5), 0.0,
                    heights[2] + Units.inchesToMeters(14.5),
                    new Rotation3d());

            this.algaeIntakePose = new Pose3d(robotPoseA).transformBy(algaeTransform);
        } else {
            this.algaeIntakePose = new Pose3d();
        }

        Logger.recordOutput("FieldSimulation/Algae Intake Pose", this.algaeIntakePose);

        /* Coral Intake Subsystem */
        if (hasCoral) {
            Pose2d robotPose = MapleSimUtil.getSwerveDriveSimulation().getSimulatedDriveTrainPose();
            Transform3d transform3d = new Transform3d(Units.inchesToMeters(11.5), 0.025,
                    heights[2] + Units.inchesToMeters(18),
                    new Rotation3d(0.0, Units.degreesToRadians(35), 0.0));

            this.coralIntakePose = new Pose3d(robotPose).transformBy(transform3d);
        } else {
            this.coralIntakePose = new Pose3d();
        }

        Logger.recordOutput("FieldSimulation/Coral Intake Pose", this.coralIntakePose);
    }

    /**
     * 
     */
    public static Pose3d getAlgaePose(String name) {
        ElevatorVisualizer elevatorVisualizer = ElevatorVisualizer.getInstance(name);
        return elevatorVisualizer.algaeIntakePose;
    }

    /**
     * 
     */
    public static Pose3d getCoralPose(String name) {
        ElevatorVisualizer elevatorVisualizer = ElevatorVisualizer.getInstance(name);
        return elevatorVisualizer.coralIntakePose;
    }

    /**
     * 
     */
    public static double getOuttakeAngle(String name) {
        ElevatorVisualizer elevatorVisualizer = ElevatorVisualizer.getInstance(name);
        return elevatorVisualizer.elevatorLevel.angleInDegrees;
    }
}
