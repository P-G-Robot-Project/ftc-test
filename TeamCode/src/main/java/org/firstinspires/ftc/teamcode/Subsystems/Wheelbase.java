package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Utils.JMotor;
import org.firstinspires.ftc.teamcode.Utils.Pose2d;
import org.firstinspires.ftc.teamcode.Utils.Subsystem;

public class Wheelbase implements Subsystem {

    public OpMode opMode;
    public JMotor left;
    public JMotor right;
    public BNO055IMU imu;

    private Pose2d pos;
    private double lastLeft;
    private double lastRight;
    private double lastHeading;

    // TODO: get real values here
    private static final double TICKS_PER_REV = 8192;
    private static final double WHEEL_RADIUS = 1;

    public Wheelbase(OpMode mode) {
        opMode = mode;
        left = new JMotor(mode.hardwareMap, "L");
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right = new JMotor(mode.hardwareMap, "R");
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        imu = mode.hardwareMap.get(BNO055IMU.class, "imu");
        pos = new Pose2d();
        lastLeft = 0;
        lastRight = 0;
        lastHeading = 0;
    }

    @Override
    public void update() {
        updatePos();
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

    public double getHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public double encoderTicksToInches(double ticks) {
        return ticks / TICKS_PER_REV * 2 * WHEEL_RADIUS * Math.PI;
    }

    public void updatePos() {
        double newLeft = left.getCurrentPosition();
        double newRight = right.getCurrentPosition();
        double newHeading = getHeading();
        double deltaL = newLeft - lastLeft;
        double deltaR = newRight - lastRight;
        double deltaH = newHeading - lastHeading;
        double turn = deltaL - deltaR;
        double forward = turn - Math.max(deltaL, deltaR);
        // TODO: definitely wrong math, needs testing
        forward = encoderTicksToInches(forward);
        pos.heading += deltaH;
        pos.x += Math.sin(pos.heading) * forward;
        pos.y += Math.cos(pos.heading) * forward;
        lastLeft = newLeft;
        lastRight = newRight;
        lastHeading = newHeading;
    }

    public void resetPos() {
        pos.x = 0;
        pos.y = 0;
        pos.heading = 0;
    }

    public void resetHeading() {
        pos.heading = 0;
    }

    public Pose2d getPos() {
        return pos;
    }

    public boolean turnToAngle(double angle, double error) {
        if (angle > pos.heading + error || angle < pos.heading - error) {
            drive(0, (angle - pos.heading)/180);
        } else {
            return true;
        }
        return false;
    }

    public boolean goToPos(double x, double y, double errorXY, double errorH) {
        double angle = Math.atan2(y - pos.y, x - pos.x);
        if (turnToAngle(angle, errorH)) {
            double distance = Math.sqrt(Math.pow(pos.x - x, 2) + Math.pow(pos.y - y, 2));
            if (distance > errorXY) {
                drive(0.4, 0);
            } else {
                return true;
            }
        }
        return false;
    }

}
