package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.DriveBase;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.LimeLight;
import org.firstinspires.ftc.teamcode.SubSystems.Servos;


@TeleOp(name = "TeleOp")
public class TeleOpFinal extends LinearOpMode {

    DriveBase Drive = new DriveBase();
    LimeLight LimeLight = new LimeLight();
    Servos Servo = new Servos();
    DriveTrain DriveTrain = new DriveTrain();

    @Override
    public void runOpMode() throws InterruptedException {

        Drive.init(hardwareMap);
        LimeLight.init(hardwareMap);
        Servo.init(hardwareMap);


        while (opModeIsActive()) {
            //Drive
            Drive.Movement(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    gamepad1.right_trigger //Power
            );

            DriveTrain.MovementEnum(
                    -gamepad2.left_stick_y,
                    gamepad2.left_stick_x,
                    gamepad2.right_stick_x
            );

            Servo.ServoButtons();
            // Gamepad1.a = Fully Open = 1
            // Gamepad1.b = Closed = 0
            // Gamepad1.x = Half Open = 0.5





        }
    }
}
