package frc.robot.modules.elevator;

import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.MechanismConstants;

/**
 * 
 */
public class ElevatorMechanism {
    private static final double kMetersPerPulse = 0.01;
    private final double THICKNESS = 6;
    private final double OUTER_BOTTOM_RAIL = Units.inchesToMeters(16.75);
    private final double MIDDLE_BOTTOM_RAIL = Units.inchesToMeters(16.5);
    private final double BOTTOM_RAIL_EXTENSION = Units.inchesToMeters(3.125);
    private final double OUTER_RAIL = Units.inchesToMeters(35.67);
    private final double MIDDLE_RAIL = Units.inchesToMeters(37.67);
    private static final double kElevatorMinimumLength = 0.5;

    private final Color8Bit OUTER_COLOR = new Color8Bit(Color.kPurple);
    private final Color8Bit MIDDLE_COLOR = new Color8Bit(Color.kAliceBlue);

    private final LoggedMechanismRoot2d canvasRoot;
    private final LoggedMechanismLigament2d olbLigament;
    private final LoggedMechanismLigament2d orbLigament;
    private final LoggedMechanismLigament2d osLigament;
    private final LoggedMechanismLigament2d mlbLigament;
    private final LoggedMechanismLigament2d mrbLigament;

    /**
     * 
     */
    public ElevatorMechanism() {
        canvasRoot = MechanismConstants.CANVAS.getRoot("InitialRoot", MechanismConstants.CANVAS_SIZE_METERS / 2, 0);

        olbLigament = new LoggedMechanismLigament2d("OuterLeftBottomRail", OUTER_BOTTOM_RAIL, 0, THICKNESS,
                OUTER_COLOR);
        orbLigament = new LoggedMechanismLigament2d("OuterRightBottomRail", OUTER_BOTTOM_RAIL, -90, THICKNESS,
                OUTER_COLOR);

        // canvasRoot.append(olbLigament);

        olbLigament.append(
                new LoggedMechanismLigament2d("LeftExtension", BOTTOM_RAIL_EXTENSION, 90, THICKNESS, OUTER_COLOR));
        orbLigament.append(
                new LoggedMechanismLigament2d("RightExtenstion", BOTTOM_RAIL_EXTENSION, -90, THICKNESS, OUTER_COLOR));

        olbLigament.append(new LoggedMechanismLigament2d("OuterLeftRail", OUTER_RAIL, 0, THICKNESS, OUTER_COLOR));
        orbLigament.append(new LoggedMechanismLigament2d("OuterRightRail", OUTER_RAIL, 0, THICKNESS, OUTER_COLOR));

        osLigament = new LoggedMechanismLigament2d("OuterShaft", 0.1, 90, THICKNESS, MIDDLE_COLOR);

        // canvasRoot.append(orbLigament);
        // canvasRoot.append(osLigament);

        mlbLigament = new LoggedMechanismLigament2d("MiddleLeftBottomRail", MIDDLE_BOTTOM_RAIL, 90, THICKNESS,
                MIDDLE_COLOR);
        mrbLigament = new LoggedMechanismLigament2d("MiddleRightBottomRail", MIDDLE_BOTTOM_RAIL, -90, THICKNESS,
                MIDDLE_COLOR);

        mlbLigament.append(new LoggedMechanismLigament2d("MiddleLeftRail", MIDDLE_RAIL, 0, THICKNESS, MIDDLE_COLOR));
        mrbLigament.append(new LoggedMechanismLigament2d("MiddleRightRail", MIDDLE_RAIL, 0, THICKNESS, MIDDLE_COLOR));
    }

    /**
     * 
     */
    public void update() {
        // this.elevator.setLength(kElevatorMinimumLength +
        // this.elevatorEncoder.getDistance());
        // this.wrist.setAngle(this.wristPot.get());
    }
}
