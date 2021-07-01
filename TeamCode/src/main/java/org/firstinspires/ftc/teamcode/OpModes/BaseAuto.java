package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;

@Autonomous(name = "BaseAuto")
public class BaseAuto extends OpMode {

    private static final double DETACH_TIME = 0.2;
    private static final double FORWARD_TIME = 2;
    private static final double TURN_TIME = 0.3;
    private static final double DRIVE_SPEED = 1;
    private static final double TURN_SPEED = 1;
    private Robot robot;
    private AutoStates state;
    private ElapsedTime time;
    private boolean returned = false;

    @Override
    public void init() {
        robot = new Robot(this);
        state = AutoStates.INIT;
        time = new ElapsedTime();
    }

    @Override
    public void loop() {
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
                if (time.seconds() < FORWARD_TIME) {
                    robot.wheels.drive(DRIVE_SPEED, 0);
                } else {
                    time.reset();
                    state = AutoStates.TURN;
                }
                break;

            case TURN:
                if (time.seconds() < TURN_TIME) {
                    robot.wheels.drive(0, TURN_SPEED);
                } else {
                    time.reset();
                    if (returned) {
                        state = AutoStates.STOP;
                    } else {
                        returned = true;
                        state = AutoStates.FORWARD;
                    }
                }
                break;

            case STOP:
                robot.wheels.stop();
                break;
        }
    }

    private enum AutoStates {
        INIT,
        DETACH,
        FORWARD,
        TURN,
        STOP
    }

}
