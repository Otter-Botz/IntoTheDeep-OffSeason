package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Sensors {

    public DistanceSensor sensorDistance;
    public TouchSensor touchSensor;
    public ColorSensor colorSensor;

    public void init(HardwareMap hwMap) {

        sensorDistance = hwMap.get(DistanceSensor.class, "sensor_distance");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;

        touchSensor = hwMap.get(TouchSensor.class, "sensor_touch");

        colorSensor = hwMap.get(ColorSensor.class, "sensor_color");
    }

    public double getDistanceInch() {
        return sensorDistance.getDistance(DistanceUnit.INCH);
    }

    public void telemetryDistance() {
        telemetry.addData("range", String.format("%.01f mm", sensorDistance.getDistance(DistanceUnit.MM)));
        telemetry.addData("range", String.format("%.01f cm", sensorDistance.getDistance(DistanceUnit.CM)));
        telemetry.addData("range", String.format("%.01f m", sensorDistance.getDistance(DistanceUnit.METER)));
        telemetry.addData("range", String.format("%.01f in", sensorDistance.getDistance(DistanceUnit.INCH)));
        telemetry.update();
    }

    public void sensorTouched() {

        if (touchSensor.isPressed()) {
            telemetry.addData("Touch Sensor", "Is Pressed");
        } else {
            telemetry.addData("Touch Sensor", "Is Not Pressed");
        }

        telemetry.update();
    }


}
