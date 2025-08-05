package org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOps.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Extendo;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Intake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Lift;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Outtake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.RobotInitializers;

@TeleOp
@Config
public class TestComponentsStates extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {

        RobotInitializers.InitializeFull(hardwareMap);
        Intake.DropUp();

        waitForStart();

        while (opModeIsActive()) {
            RobotInitializers.clearCache();
            Lift.update();
            Extendo.update();

            RobotInitializers.Dashtelemetry.addData("lift motor1 lift amps",Lift.motor1.getCurrent(CurrentUnit.AMPS));
            RobotInitializers.Dashtelemetry.addData("lift encoder Velocity",Lift.encoder.getVelocity());
            RobotInitializers.Dashtelemetry.addData("lift pos",Lift.getPosition());

            RobotInitializers.Dashtelemetry.addData("extendo motor1 lift amps",Extendo.motor.getCurrent(CurrentUnit.AMPS));
            RobotInitializers.Dashtelemetry.addData("extendo encoder Velocity",Extendo.encoder.getVelocity());
            RobotInitializers.Dashtelemetry.addData("extendo pos",Extendo.getPosition());

            RobotInitializers.Dashtelemetry.update();

        }
    }
}
