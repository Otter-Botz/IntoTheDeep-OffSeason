package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.DriveBase;
import org.firstinspires.ftc.teamcode.SubSystems.LimeLight;
import org.firstinspires.ftc.teamcode.SubSystems.Servos;


@TeleOp(name = "TeleOp")
public class TeleOpFinal extends LinearOpMode {

    DriveBase Drive = new DriveBase();
    //LimeLight LimeLight = new LimeLight();
    Servos Servo = new Servos();

    @Override
    public void runOpMode() throws InterruptedException {

        Drive.init(hardwareMap);
        //LimeLight.init(hardwareMap);
        Servo.init(hardwareMap);

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
            else if (gamepad1.y && !gamepad1.y) {
                Servo.switchGrabState();
            }
            else if (gamepad2.a) {
                Servo.wristUp();
            }
            else if (gamepad1.b) {
                Servo.wristDown();
            }
            else if (gamepad2.y && !gamepad2.y) {
                Servo.switchPivotState();
            }








        }
    }
}
