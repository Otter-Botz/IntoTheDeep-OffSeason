package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.teamcode.Constants.Hardware.BackLeftMotor;
import static org.firstinspires.ftc.teamcode.Constants.Hardware.BackRightMotor;
import static org.firstinspires.ftc.teamcode.Constants.Hardware.FrontLeftMotor;
import static org.firstinspires.ftc.teamcode.Constants.Hardware.FrontRightMotor;
import static org.firstinspires.ftc.teamcode.Constants.Hardware.imu;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveBase {

    public void resetYaw(){
        imu.resetYaw();

    }

    public void Movement(double y, double x, double rx, double power ){

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
        double frontLeftPower = (rotY + rotX + rx) / denominator * power;
        double backLeftPower = (rotY - rotX + rx) / denominator * power;
        double frontRightPower = (rotY - rotX - rx) / denominator * power;
        double backRightPower = (rotY + rotX - rx) / denominator * power;

        FrontLeftMotor.setPower(frontLeftPower);
        BackLeftMotor.setPower(backLeftPower);
        FrontRightMotor.setPower(frontRightPower);
        BackRightMotor.setPower(backRightPower);

    }

        /*
        telemetry.addData("rotx", rotX);
        telemetry.addData("roty", rotY);
        telemetry.addData("frontleftpower", frontLeftPower);
        telemetry.addData("frontrightpower", frontRightPower);
        telemetry.addData("backleftpower", backLeftPower);
        telemetry.addData("backrightpower", backRightPower);
        telemetry.addData("Denominator", denominator);
        telemetry.addData("Parameterforimu", parameters);
        telemetry.addData("leftStickY", gamepad1.left_stick_y);
        telemetry.addData("leftStickX", gamepad1.left_stick_x);
        telemetry.addData("rightStickY", gamepad1.right_stick_y);
        telemetry.addData("rightstickx", gamepad1.right_stick_x);
        telemetry.addData("imuyawpitchroll", imu.getRobotYawPitchRollAngles());
        telemetry.addData("cos of radian", Math.cos(-botHeading));
        telemetry.addData("sin of radian", Math.sin(-botHeading));
        telemetry.addData("radiens yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        telemetry.addData("a", a);
        telemetry.addData("b", b);
        telemetry.update();
        */
}