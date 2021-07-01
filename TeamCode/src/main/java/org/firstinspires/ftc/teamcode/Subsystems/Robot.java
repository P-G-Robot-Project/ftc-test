package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Robot {

    public Wheelbase wheels;
    public Detach detach;

    public Robot(OpMode opMode) {
        wheels = new Wheelbase(opMode);
        detach = new Detach(opMode);
    }
}
