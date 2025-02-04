package frc.robot.modules.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * 
 */
public class ElevatorVisualizer {
    private final ElevatorMechanism elevatorMechanism = new ElevatorMechanism();
    public static final Translation2d elevatorOrigin2d = new Translation2d(0.10686, 0.06357);
    public static final Translation3d elevatorOrigin3d = new Translation3d(elevatorOrigin2d.getX(), 0.0,
            elevatorOrigin2d.getY());

    private final String name;

    /**
     * 
     */
    public ElevatorVisualizer(String name) {
        this.name = name;
    }

    /**
     * 
     */
    public void update(double positionInMeters) {
        this.elevatorMechanism.setLength(positionInMeters);

        Logger.recordOutput("Mechanism3d/" + name,
                new Pose3d[] {
                        new Pose3d(), // outer frame
                        new Pose3d(), // middle frame
                        new Pose3d(), // inner frame
                        new Pose3d(), // box
                        new Pose3d(), // algae intake
                        new Pose3d() // coral intake
                });
    }
}
