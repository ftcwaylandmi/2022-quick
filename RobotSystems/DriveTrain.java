package org.firstinspires.ftc.teamcode.RobotSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DriveTrain {

    RobotHardware robotHardware = new RobotHardware();

    // Constants
    int MAX_TURN_TICKS = 200;
    int MIN_TURN_TICKS = -200;

    double GRAB_SERVO_MAX = 0.65;
    double GRAB_SERVO_MIN = 0.5;

    // Variables
    int angleArmCurrentTicks = 0;
    int turnArmCurrentTicks = 0;


    // Private
    private int _cubicInt(double in){
        int calc = (int) (in*in*in);
        return calc;
    }

    public void initHW(HardwareMap ahw) { // init All Hardware/Devices
        robotHardware.init(ahw);
    }

    public void initRobot() {
    }

    public double ServoPos(){
        return robotHardware.grabServo.getPosition();
    }

    public void OpenServo(){
        robotHardware.grabServo.setPosition(GRAB_SERVO_MIN);
    }

    public void CloseServo(){
        robotHardware.grabServo.setPosition(GRAB_SERVO_MAX);
    }

    public void StopDrive() {
        robotHardware.module1.setPower(0);
        robotHardware.module2.setPower(0);
    }

    public void Drive(double p) {
        robotHardware.module1.setPower(-p);
        robotHardware.module2.setPower(-p);
    }

    public void ArmExtend(double p) {
        robotHardware.extendArm.setPower(p);
    }

    public int TurnArmWithSticks(double in){
        if(in == 0) {
            robotHardware.turnArm.setTargetPosition(turnArmCurrentTicks);
            robotHardware.turnArm.setPower(.1);
            return 1;
        } else {
            int calculateToTicks = _cubicInt(in*3);
            robotHardware.turnArm.setTargetPosition(robotHardware.turnArm.getCurrentPosition() + calculateToTicks);
            robotHardware.turnArm.setPower(1);
            turnArmCurrentTicks = robotHardware.turnArm.getCurrentPosition();
            return 2;
        }
    }

    public int AngleArmWithSticks(double in){
        if(in == 0) {
            robotHardware.angleArm.setTargetPosition(angleArmCurrentTicks);
            robotHardware.angleArm.setPower(1);
            return 1;
        } else {
            int calculateToTicks = _cubicInt(in * 3);
            robotHardware.angleArm.setTargetPosition(robotHardware.angleArm.getCurrentPosition() + calculateToTicks);
            robotHardware.angleArm.setPower(1);
            angleArmCurrentTicks = robotHardware.angleArm.getCurrentPosition();
            return 2;
        }
    }

    public int ReturnArmAngleTicks(){
        return robotHardware.angleArm.getCurrentPosition();
    }

    public int ReturnArmTurnTicks(){
        return robotHardware.turnArm.getCurrentPosition();
    }

    public void RightDrive(double p) {
        robotHardware.module1.setPower(p);
    }

    public void LeftDrive(double p) {
        robotHardware.module2.setPower(p);
    }

    public void TurnRight(double p){
        robotHardware.module1.setPower(-p);
        robotHardware.module2.setPower(p);
    }

    public void TurnLeft(double p){
        robotHardware.module1.setPower(p);
    }

    public void wait(int sleeptime) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.milliseconds() < sleeptime) {

        }
    }
}