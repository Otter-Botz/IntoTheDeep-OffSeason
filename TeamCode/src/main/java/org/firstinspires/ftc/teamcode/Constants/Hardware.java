package org.firstinspires.ftc.teamcode.Constants;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Hardware {

    public static void initRobot() {
        sliderInit(hardwareMap);
        servoInit(hardwareMap);
        sensorInit(hardwareMap);
        driveInit(hardwareMap);
    }

    public static DcMotor sliderMotor;
    public static DcMotor sliderMotorMotor;

    public static void sliderInit(HardwareMap hwMap) {
        sliderMotor = hwMap.get(DcMotor.class, "sliderMotor");
        sliderMotorMotor = hwMap.get(DcMotor.class, "sliderMotorMotor");

        sliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderMotorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderMotorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sliderMotorMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public static Servo clawServo;
    public static Servo wristServo;
    public static Servo armServo;

    public static void servoInit(HardwareMap hwMap) {
        clawServo = hwMap.get(Servo.class, "clawServo");
        wristServo = hwMap.get(Servo.class, "wristServo");
        armServo = hwMap.get(Servo.class, "armServo");
    }

    public static DistanceSensor sensorDistance;
    public static TouchSensor touchSensor;
    public static ColorSensor colorSensor;

    public static void sensorInit(HardwareMap hwMap) {

        sensorDistance = hwMap.get(DistanceSensor.class, "sensor_distance");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;

        touchSensor = hwMap.get(TouchSensor.class, "sensor_touch");

        colorSensor = hwMap.get(ColorSensor.class, "sensor_color");
    }

    public static DcMotor FrontLeftMotor;
    public static DcMotor BackLeftMotor;
    public static DcMotor FrontRightMotor;
    public static DcMotor BackRightMotor;

    public static IMU imu;

    public static void driveInit(HardwareMap hwMap) {
        FrontLeftMotor = hwMap.get(DcMotor.class, "frontLeft");
        BackLeftMotor = hwMap.get(DcMotor.class, "backLeft");
        FrontRightMotor = hwMap.get(DcMotor.class, "frontRight");
        BackRightMotor = hwMap.get(DcMotor.class, "backRight");
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
}
