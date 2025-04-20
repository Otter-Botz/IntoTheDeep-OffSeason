package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants.Hardware;
import org.firstinspires.ftc.teamcode.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.SubSystems.DriveBase;
import org.firstinspires.ftc.teamcode.SubSystems.LimeLight;
import org.firstinspires.ftc.teamcode.SubSystems.Servos;
import org.firstinspires.ftc.teamcode.SubSystems.Motors;


@TeleOp(name = "TeleOp")
public class TeleOpFinal extends LinearOpMode {

    DriveBase Drive = new DriveBase();
    //LimeLight LimeLight = new LimeLight();
    Servos Servo = new Servos();
    Motors Motor = new Motors();

    @Override
    public void runOpMode() throws InterruptedException {

        Hardware.initRobot();

        waitForStart();

        while (opModeIsActive()) {
            //Drive
            Drive.Movement(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    gamepad1.right_trigger //Power
            );
            if (gamepad1.a) {
                Servo.openClaw();
            }
            else if (gamepad1.x) {
                Servo.closeClaw();
            }
            else if (gamepad1.y) {
                Servo.wristUp();
            }
            else if (gamepad1.b) {
                Servo.wristDown();
            }

            if (gamepad1.left_bumper) {
                Motor.controlSlider(RobotConstants.sliderPower[9],
                        RobotConstants.sliderPositions[9]);
            }









        }
    }
}
