package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveTrain {
    public DcMotor FrontLeftMotor;
    public DcMotor BackLeftMotor;
    public DcMotor FrontRightMotor;
    public DcMotor BackRightMotor;

    private IMU imu;

    public void init(HardwareMap hwMap) {

        FrontLeftMotor = hardwareMap.get(DcMotor.class, MotorName.FRONT_LEFT.getName());
        FrontRightMotor = hardwareMap.get(DcMotor.class, MotorName.FRONT_RIGHT.getName());
        BackLeftMotor = hardwareMap.get(DcMotor.class, MotorName.BACK_LEFT.getName());
        BackRightMotor = hardwareMap.get(DcMotor.class, MotorName.BACK_RIGHT.getName());
        FrontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hwMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

    }

    public enum MotorName {
        FRONT_LEFT("leftFront"),
        FRONT_RIGHT("rightFront"),
        BACK_LEFT("leftBack"),
        BACK_RIGHT("rightBack");

        private final String name;

        MotorName(String name) {
            this.name = name;
        }

        public String getName() {
            return name;
        }
    }

    public double setMotorPower(MotorName motor,
                                MotorName motor1,
                                MotorName motor2,
                                MotorName motor3,
                                double power) {
        switch (motor) {
            case FRONT_LEFT:
                FrontLeftMotor.setPower(power);
                break;
            case FRONT_RIGHT:
                FrontRightMotor.setPower(power);
                break;
            case BACK_LEFT:
                BackLeftMotor.setPower(power);
                break;
            case BACK_RIGHT:
                BackRightMotor.setPower(power);
                break;
        }
        switch (motor1) {
            case FRONT_LEFT:
                FrontLeftMotor.setPower(power);
                break;
            case FRONT_RIGHT:
                FrontRightMotor.setPower(power);
                break;
            case BACK_LEFT:
                BackLeftMotor.setPower(power);
                break;
            case BACK_RIGHT:
                BackRightMotor.setPower(power);
                break;
        }
        switch (motor2) {
            case FRONT_LEFT:
                FrontLeftMotor.setPower(power);
                break;
            case FRONT_RIGHT:
                FrontRightMotor.setPower(power);
                break;
            case BACK_LEFT:
                BackLeftMotor.setPower(power);
                break;
            case BACK_RIGHT:
                BackRightMotor.setPower(power);
                break;
        }
        switch (motor3) {
            case FRONT_LEFT:
                FrontLeftMotor.setPower(power);
                break;
            case FRONT_RIGHT:
                FrontRightMotor.setPower(power);
                break;
            case BACK_LEFT:
                BackLeftMotor.setPower(power);
                break;
            case BACK_RIGHT:
                BackRightMotor.setPower(power);
                break;
        }
        return power;
    }

    public void MovementEnum(double x, double y, double rx) {
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        //get plane coordinates of robot from yaw
        double a = Math.cos(-botHeading);
        double b = Math.sin(-botHeading);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        FrontLeftMotor.setPower(setMotorPower(
                MotorName.FRONT_LEFT,
                MotorName.BACK_LEFT,
                MotorName.BACK_RIGHT,
                MotorName.FRONT_RIGHT,1));
        BackLeftMotor.setPower(setMotorPower(
                MotorName.FRONT_LEFT,
                MotorName.BACK_LEFT,
                MotorName.BACK_RIGHT,
                MotorName.FRONT_RIGHT,1));
        FrontRightMotor.setPower(setMotorPower(
                MotorName.FRONT_LEFT,
                MotorName.BACK_LEFT,
                MotorName.BACK_RIGHT,
                MotorName.FRONT_RIGHT,1));
        BackRightMotor.setPower(setMotorPower(
                MotorName.FRONT_LEFT,
                MotorName.BACK_LEFT,
                MotorName.BACK_RIGHT,
                MotorName.FRONT_RIGHT,1));
    }

}
