package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Webcam extends SubsystemBase {

    private Climb m_climbSubsystem;

    UsbCamera webcam;
    private static Webcam mInstance = null;

    public static Webcam getInstance() {

        if (mInstance == null) {
            mInstance = new Webcam();
        }
        return mInstance;
    }

    public Webcam() {
        webcam = CameraServer.startAutomaticCapture();
        webcam.setResolution(160, 120);
        webcam.setPixelFormat(PixelFormat.kMJPEG);
    }
}
