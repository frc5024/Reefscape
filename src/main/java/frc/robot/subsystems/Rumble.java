package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rumble extends SubsystemBase {

    private final XboxController driver;
    private final XboxController operator;

    private static Rumble mInstance;

    public static Rumble getInstance() {
        if (mInstance == null) {
            mInstance = new Rumble();
        }
        return mInstance;
    }

    // Rumble states
    public enum RumbleState {
        OFF,
        STATIC,
        DOUBLE
    }

    private Rumble() {
        driver = new XboxController(0);
        operator = new XboxController(1);
    }

    // Start static rumble
    public void staticRumble(boolean isDriver) {
        XboxController controller = isDriver ? driver : operator;
        new Thread(() -> {
            rumbleOn(controller);
            try {
                Thread.sleep(500); // Rumble for 0.5 seconds
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            rumbleOff(controller);
        }).start();
    }

    // Start double rumble
    public void doubleRumble(boolean isDriver) {
        XboxController controller = isDriver ? driver : operator;
        new Thread(() -> {
            try {
                rumbleOn(controller);
                Thread.sleep(200); // First rumble
                rumbleOff(controller);
                Thread.sleep(100); // Pause
                rumbleOn(controller);
                Thread.sleep(200); // Second rumble
                rumbleOff(controller);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }).start();
    }

    // Turn rumble off
    private void rumbleOff(XboxController controller) {
        controller.setRumble(GenericHID.RumbleType.kBothRumble, 0);
    }

    // Turn rumble on
    private void rumbleOn(XboxController controller) {
        controller.setRumble(GenericHID.RumbleType.kBothRumble, 1.0);
    }
}