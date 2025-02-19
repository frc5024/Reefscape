package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rumble extends SubsystemBase {

    private final XboxController driver;
    private final XboxController operator;

    // Rumble states
    public enum RumbleState {
        OFF,
        STATIC,
        DOUBLE
    }

    public Rumble() {
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

    // private final XboxController driver;

    // private Timer rumbleTime;

    // boolean timeComplete = false;
    // double time = 0;
    // RumbleState state;
    // long cycle = 0;

    // // sets possible rumble states
    // public enum RumbleState {
    // OFF,
    // STATIC,
    // DOUBLE
    // }

    // //sets rumbles predetermineds
    // public Rumble() {
    // driver = new XboxController(0);

    // rumbleTime = new Timer();
    // rumbleTime.reset();
    // }

    // //when called turns on rumble and starts timer
    // public void staticRumble() {
    // state = RumbleState.STATIC;
    // rumbleTime.reset();
    // rumbleOn();
    // rumbleTime.start();

    // }

    // // currently not in use (WIP)
    // public void doubleRumble() {
    // state = RumbleState.DOUBLE;
    // rumbleTime.reset();
    // rumbleOn();
    // rumbleTime.start();

    // }

    // // takes the state and waits until timer is above a certain point before
    // turning off rumbling
    // @Override
    // public void periodic() {
    // time = rumbleTime.get();

    // // static rumble section
    // if (state == RumbleState.STATIC && time >= 0.5){
    // rumbleOff();
    // state = RumbleState.OFF;
    // rumbleTime.stop();
    // }
    // }

    // //turns motor to %0
    // public void rumbleOff(){
    // driver.setRumble(GenericHID.RumbleType.kBothRumble, 0);
    // }

    // //turns motor to 100%
    // public void rumbleOn() {
    // driver.setRumble(GenericHID.RumbleType.kBothRumble, 1.0);
    // }
}