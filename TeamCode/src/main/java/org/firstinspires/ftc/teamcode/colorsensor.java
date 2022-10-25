package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;






@Autonomous
public class colorsensor extends LinearOpMode {
    private HackHers_Lib everything;
    private

    ColorSensor CS;

    @Override
    public void runOpMode(){

        super.waitForStart();

        everything.Color_Sensor.red();





        CS= hardwareMap.get(ColorSensor.class, "CS");





    }

    }



