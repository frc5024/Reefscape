package frc.robot.subsystems;

import frc.robot.subsystems.LEDController;
import frc.robot.subsystems.LEDPreset;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDs extends SubsystemBase {
    private static LEDs mInstance = null;
    private LEDController ledController;


    public static LEDs getInstance() {
        if (mInstance == null) {
            mInstance = new LEDs();
        }
        return mInstance;
    }
    private LEDs(){
        ledController = new LEDController(Constants.LEDs.ledPort);
    }

    public void setLEDS(ILEDPreset colour){
        System.out.println("YESSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS");
        ledController.set(colour);
    }
}
 