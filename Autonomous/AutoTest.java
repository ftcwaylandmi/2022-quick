package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotSystems.DriveTrain;

@Autonomous(name = "AutoTest", group = "11846")
public class AutoTest extends LinearOpMode{
    private ElapsedTime myruntime = new ElapsedTime();
    private DriveTrain driveTrain = new DriveTrain();

    @Override
    public void runOpMode(){
        telemetry.addData("Status", "I am booting up. Almost there");
        telemetry.update();

        driveTrain.initHW(hardwareMap);
        waitForStart();
        myruntime.reset();

        driveTrain.wait(3000);
        driveTrain.Drive(1);
        driveTrain.wait(1000);
        driveTrain.StopDrive();


    }

}