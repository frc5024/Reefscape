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
    private final double THICKNESS = 5;
    private final double OUTER_BOTTOM_RAIL = Units.inchesToMeters(19.0);
    private final double MIDDLE_BOTTOM_RAIL = Units.inchesToMeters(17.0);
    private final double INNER_BOTTOM_RAIL = Units.inchesToMeters(14.5);
    private final double BOX_BOTTOM_RAIL = Units.inchesToMeters(10.0);
    private final double BOTTOM_RAIL_EXTENSION = Units.inchesToMeters(3.125);
    private final double OUTER_RAIL = Units.inchesToMeters(34.0);
    private final double MIDDLE_RAIL = Units.inchesToMeters(35.0);
    private final double INNER_RAIL = Units.inchesToMeters(35.0);
    private final double BOX_RAIL = Units.inchesToMeters(12.0);
    private static final double kElevatorMinimumLength = 0.0;

    private final Color8Bit OUTER_COLOR = new Color8Bit(Color.kPurple);
    private final Color8Bit MIDDLE_COLOR = new Color8Bit(Color.kAliceBlue);
    private final Color8Bit INNER_COLOR = new Color8Bit(Color.kDarkSeaGreen);
    private final Color8Bit BOX_COLOR = new Color8Bit(Color.kBlue);
    private final Color8Bit SHAFT_COLOR = new Color8Bit(Color.kWheat);

    private final LoggedMechanismRoot2d canvasRoot;
    private final LoggedMechanismLigament2d oblLigament;
    private final LoggedMechanismLigament2d obrLigament;
    private final LoggedMechanismLigament2d osLigament;
    private final LoggedMechanismLigament2d mblLigament;
    private final LoggedMechanismLigament2d mbrLigament;
    private final LoggedMechanismLigament2d msLigament;
    private final LoggedMechanismLigament2d iblLigament;
    private final LoggedMechanismLigament2d ibrLigament;
    private final LoggedMechanismLigament2d isLigament;
    private final LoggedMechanismLigament2d bblLigament;
    private final LoggedMechanismLigament2d bbrLigament;

    /**
     * 
     */
    public ElevatorMechanism() {
        canvasRoot = MechanismConstants.CANVAS.getRoot("InitialRoot", MechanismConstants.CANVAS_SIZE_METERS / 2, 0);

        // Outer Elevator
        oblLigament = new LoggedMechanismLigament2d(
                "OuterBottomLeftRail", OUTER_BOTTOM_RAIL / 2, 180, THICKNESS, OUTER_COLOR);
        obrLigament = new LoggedMechanismLigament2d(
                "OuterBottomRightRail", OUTER_BOTTOM_RAIL / 2, 0, THICKNESS, OUTER_COLOR);
        osLigament = new LoggedMechanismLigament2d("OuterShaft", kElevatorMinimumLength, 90, THICKNESS, SHAFT_COLOR);

        oblLigament.append(
                new LoggedMechanismLigament2d("LeftExtension", BOTTOM_RAIL_EXTENSION, 0, THICKNESS, OUTER_COLOR));
        obrLigament.append(
                new LoggedMechanismLigament2d("RightExtenstion", BOTTOM_RAIL_EXTENSION, 0, THICKNESS, OUTER_COLOR));

        oblLigament.append(new LoggedMechanismLigament2d("OuterLeftRail", OUTER_RAIL, -90, THICKNESS, OUTER_COLOR));
        obrLigament.append(new LoggedMechanismLigament2d("OuterRightRail", OUTER_RAIL, 90, THICKNESS, OUTER_COLOR));

        // Middle Elevator
        mblLigament = new LoggedMechanismLigament2d("MiddleBottomLeftRail", MIDDLE_BOTTOM_RAIL / 2, 90, THICKNESS,
                MIDDLE_COLOR);
        mbrLigament = new LoggedMechanismLigament2d("MiddleBottomRightRail", MIDDLE_BOTTOM_RAIL / 2, -90, THICKNESS,
                MIDDLE_COLOR);

        msLigament = new LoggedMechanismLigament2d("MiddleShaft", kElevatorMinimumLength, 0, THICKNESS, SHAFT_COLOR);

        mblLigament.append(new LoggedMechanismLigament2d("MiddleLeftRail", MIDDLE_RAIL, -90, THICKNESS, MIDDLE_COLOR));
        mbrLigament.append(new LoggedMechanismLigament2d("MiddleRightRail", MIDDLE_RAIL, 90, THICKNESS, MIDDLE_COLOR));

        osLigament.append(mblLigament);
        osLigament.append(mbrLigament);

        // Inner Elevator
        iblLigament = new LoggedMechanismLigament2d("InnerBottomLeftRail", INNER_BOTTOM_RAIL / 2, 90, THICKNESS,
                INNER_COLOR);
        ibrLigament = new LoggedMechanismLigament2d("InnerBottomRightRail", INNER_BOTTOM_RAIL / 2, -90, THICKNESS,
                INNER_COLOR);

        isLigament = new LoggedMechanismLigament2d("InnerShaft", kElevatorMinimumLength, 0, THICKNESS, SHAFT_COLOR);

        iblLigament.append(new LoggedMechanismLigament2d("InnerLeftRail", INNER_RAIL, -90, THICKNESS, INNER_COLOR));
        ibrLigament.append(new LoggedMechanismLigament2d("InnerRightRail", INNER_RAIL, 90, THICKNESS, INNER_COLOR));

        msLigament.append(iblLigament);
        msLigament.append(ibrLigament);
        osLigament.append(msLigament);

        // Box
        bblLigament = new LoggedMechanismLigament2d("BoxBottomLeftRail", BOX_BOTTOM_RAIL / 2, 90, THICKNESS,
                BOX_COLOR);
        bbrLigament = new LoggedMechanismLigament2d("BoxBottomRightRail", BOX_BOTTOM_RAIL / 2, -90, THICKNESS,
                BOX_COLOR);

        bblLigament.append(new LoggedMechanismLigament2d("BoxLeftRail", BOX_RAIL, -90, THICKNESS, BOX_COLOR));
        bbrLigament.append(new LoggedMechanismLigament2d("BoxRightRail", BOX_RAIL, 90, THICKNESS, BOX_COLOR));

        isLigament.append(bblLigament);
        isLigament.append(bbrLigament);
        msLigament.append(isLigament);

        // Add ligaments to the canvas
        canvasRoot.append(oblLigament);
        canvasRoot.append(obrLigament);
        canvasRoot.append(osLigament);
    }

    /**
     * 
     */
    public void update(double length) {
        if (length < 0)
            length = 0.01;
        this.osLigament.setLength(kElevatorMinimumLength + length);
        this.msLigament.setLength(kElevatorMinimumLength + (length * 1.5));
        this.isLigament.setLength(kElevatorMinimumLength + (length * 2.0));
    }
}
