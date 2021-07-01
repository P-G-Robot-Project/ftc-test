package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Utils.JServo;
import org.firstinspires.ftc.teamcode.Utils.Subsystem;

public class Detach implements Subsystem {

    public OpMode opMode;
    public JServo detach;

    // TODO: tune values
    public final Double DETACH_POINT = 0.2;
    public final Double REATTACH_POINT = 0.6;

    public Detach(OpMode mode) {
        opMode = mode;
        detach = new JServo(opMode.hardwareMap, "Detach");
    }

    @Override
    public void update() {

    }

    public void detach() {
        detach.setPosition(DETACH_POINT);
    }

    public void reattach() {
        detach.setPosition(REATTACH_POINT);
    }
}
