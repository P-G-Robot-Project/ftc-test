package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Utils.JMotor;
import org.firstinspires.ftc.teamcode.Utils.Subsystem;

public class Wheelbase implements Subsystem {

    public OpMode opMode;
    public JMotor left;
    public JMotor right;

    public Wheelbase(OpMode mode) {
        opMode = mode;
        left = new JMotor(mode.hardwareMap, "L");
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right = new JMotor(mode.hardwareMap, "R");
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void update() {

    }

    public void drive(double speed, double turn) {
        double scale = 1 / Math.sqrt(speed * speed + turn * turn);
        left.setPower((speed + turn) * scale);
        right.setPower((speed - turn) * scale);
    }

    public void stop() {
        left.setPower(0);
        right.setPower(0);
    }

}
