package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Utils.Pose2d;

@Autonomous(name = "Square")
public class SquareAuto extends OpMode {

    private Robot robot;
    private static final double XYError = 1;
    private static final double HError = 3;

    private AutoStates state = AutoStates.INIT;

    private ElapsedTime time;
    private static final double DETACH_TIME = 0.2;
    private int turns = 0;

    @Override
    public void init() {
        robot = new Robot(this);
        time = new ElapsedTime();
    }

    @Override
    public void loop() {

        robot.wheels.update();

        switch (state) {

            case INIT:
                time.reset();
                state = AutoStates.DETACH;
                break;

            case DETACH:
                robot.detach.detach();
                if (time.seconds() > DETACH_TIME) {
                    time.reset();
                    state = AutoStates.FORWARD;
                }
                break;

            case FORWARD:
                robot.wheels.resetPos();
                if (robot.wheels.goToPos(10, 0, XYError, HError)) {
                    time.reset();
                    state = AutoStates.TURN;
                }
                break;

            case TURN:
                robot.wheels.resetPos();
                if (robot.wheels.turnToAngle(-90, HError)) {
                    time.reset();
                    if (turns == 3) {
                        state = AutoStates.STOP;
                    } else {
                        turns++;
                        state = AutoStates.FORWARD;
                    }
                }
                break;

            case STOP:
                robot.wheels.stop();
                break;
        }
    }

    enum AutoStates {
        INIT,
        DETACH,
        FORWARD,
        TURN,
        STOP
    }
}
