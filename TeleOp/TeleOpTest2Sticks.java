package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotSystems.DriveTrain;

@TeleOp(name="TeleOpTest2Sticks", group="11846")
public class TeleOpTest2Sticks extends OpMode{
    DriveTrain driveTrain = new DriveTrain();

    double left = 0.00;
    double right = 0.00;

    double turnStick = 0.00;
    double angleStick = 0.00;
    double extendStick = 0.00;

    final double SLIDE_POWER = 1.00;

    @Override
    public void init() {
        driveTrain.initHW(hardwareMap);
    }

    @Override
    public void loop() {
        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;

        turnStick = -gamepad2.right_stick_x;
        angleStick = -gamepad2.left_stick_y;
        extendStick = -gamepad2.right_stick_y;

        driveTrain.LeftDrive((right*right*right)/1.5);
        driveTrain.RightDrive((left*left*left)/1.5);

        if(gamepad2.left_bumper){
            driveTrain.OpenServo();
        }
        if(gamepad2.right_bumper){
            driveTrain.CloseServo();
        }

        telemetry.addData("TurnArmTicks", driveTrain.TurnArmWithSticks(turnStick));
        telemetry.addData("ServoPos", driveTrain.ServoPos());
        telemetry.addData("AngleArmTicks", driveTrain.AngleArmWithSticks(angleStick));
        telemetry.addData("angleArm", driveTrain.ReturnArmAngleTicks());
        telemetry.addData("turnArm", driveTrain.ReturnArmTurnTicks());
        telemetry.update();

    }
}