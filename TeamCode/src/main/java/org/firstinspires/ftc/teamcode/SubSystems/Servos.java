package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Servos {

    public Servo Servo;

    private ServoPosition currentServoPosition = ServoPosition.CLOSED;

    public void init(HardwareMap hwMap) {
        Servo = hwMap.get(Servo.class, "Servo");
    }

    public enum ServoPosition {
        CLOSED(0.0),
        HALF_OPEN(0.5),
        FULLY_OPEN(1);

        public final double position;

        ServoPosition(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }

    public void ServoButtons() {
        if (gamepad1.a) {
            currentServoPosition = ServoPosition.FULLY_OPEN;
        } else if (gamepad1.b) {
            currentServoPosition = ServoPosition.CLOSED;
        } else if (gamepad1.x) {
            currentServoPosition = ServoPosition.HALF_OPEN;
        }

        Servo.setPosition(currentServoPosition.getPosition());

        telemetry.addData("Claw Position", currentServoPosition);
        telemetry.update();
    }
}
