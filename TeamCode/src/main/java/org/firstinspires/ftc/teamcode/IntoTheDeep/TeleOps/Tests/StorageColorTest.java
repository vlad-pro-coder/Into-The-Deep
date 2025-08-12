package org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOps.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Intake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.RobotInitializers;

@TeleOp
@Config
public class StorageColorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotInitializers.InitializeFull(hardwareMap);


        waitForStart();

        while (opModeIsActive()){
            Intake.colorsensor.getColorSeenBySensor();
            RobotInitializers.Dashtelemetry.addData("color", Intake.getStorageStatus());
            RobotInitializers.Dashtelemetry.addData("disatnce",Intake.rangesensor.getDist());
            RobotInitializers.Dashtelemetry.addData("isagaisttrapdoor",Intake.SampleReachedTrapDoor());
            RobotInitializers.Dashtelemetry.addData("r",Intake.colorsensor.RGB.R);
            RobotInitializers.Dashtelemetry.addData("g",Intake.colorsensor.RGB.G);
            RobotInitializers.Dashtelemetry.addData("b",Intake.colorsensor.RGB.B);
            RobotInitializers.Dashtelemetry.addData("d",Intake.colorsensor.RGB.D);
            RobotInitializers.Dashtelemetry.update();
        }
    }
}
