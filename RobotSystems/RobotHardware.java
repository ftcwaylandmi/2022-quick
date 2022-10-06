package org.firstinspires.ftc.teamcode.RobotSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {
    public DcMotor module1 = null;
    public DcMotor module2 = null;

    public DcMotor angleArm = null;
    public DcMotor turnArm = null;

    public Servo grabServo = null;

    public DcMotor extendArm = null;

    HardwareMap hwMap = null;

    public void init (HardwareMap ahwMap){
        hwMap = ahwMap;


        // DriveTrain
        module1 = hwMap.get(DcMotor.class, "md1");
        module2 = hwMap.get(DcMotor.class, "md2");

        module1.setDirection(DcMotor.Direction.REVERSE);
        module2.setDirection(DcMotor.Direction.FORWARD);

        module1.setPower(0);
        module2.setPower(0);

        module1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        module2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // DriveTrain End

        // Arm
        angleArm = hwMap.get(DcMotor.class, "angleArm");
        turnArm = hwMap.get(DcMotor.class, "turnArm");
        extendArm = hwMap.get(DcMotor.class, "extendArm");
        grabServo = hwMap.get(Servo.class, "grabServo");

        angleArm.setDirection(DcMotor.Direction.REVERSE);
        turnArm.setDirection(DcMotor.Direction.REVERSE);
        extendArm.setDirection(DcMotor.Direction.FORWARD);
        grabServo.setDirection(Servo.Direction.FORWARD);

        angleArm.setPower(0);
        turnArm.setPower(0);
        extendArm.setPower(0);

        angleArm.setTargetPosition(angleArm.getCurrentPosition());
        turnArm.setTargetPosition(turnArm.getCurrentPosition());

        angleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turnArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Arm End
    }
}




