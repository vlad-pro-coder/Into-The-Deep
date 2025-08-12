package org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.ApproachingSubmersibleActions.AfterThirdSampleGoToSubmersible.AfterThirdSampleGoToSubmersibleActions;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.ApproachingSubmersibleActions.FailedToDetect.FailedToDetectActions;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.ApproachingSubmersibleActions.TakeCachedSample.TakeCachedSampleActions;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.ApproachingSubmersibleActions.TakeSubmersibleSampleToHighBasket.GoingBackAndForthToHighBasketAndSubmersibleActions;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.CHASSIS_sample1pos;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.CHASSIS_sample2pos;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.CHASSIS_sample3pos;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.EXTENDO_sample1;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.EXTENDO_sample2;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.EXTENDO_sample3;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.EXTENSION_overbasket;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.LIFT_firstsample;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.LIFT_preload;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.LIFT_secondsample;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.LIFT_thirdsample;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.OVERHEAD_firstsample;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.OVERHEAD_idle;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.OVERHEAD_preload;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.OVERHEAD_secondsample;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.OVERHEAD_thirdsample;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.besttxty;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.FOURpreloadsLogic.PutLastSamplePredefined.PutLastSamplePredefinedActions;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.FOURpreloadsLogic.SimultaniosSampleGrabingScoring.SimultaniosSampleGrabingScoringActions;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Scheduler;
import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Task;
import org.firstinspires.ftc.teamcode.IntoTheDeep.CameraPipelines.YellowSampleDetectionPipeline;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Chassis;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Extendo;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Intake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Lift;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Localizer;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Outtake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.RobotInitializers;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Wrapers.Logitech920;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Wrapers.PinPoint;

@Autonomous
@Config
public class Samples extends LinearOpMode {
    public enum RobotStates{
        CRUISETOSUBMERSIBLE,
        OCCUPIEDPRELOADS,
        DETECT,
        FAILEDDETECTION,
        GRABFROMSUBMERSIBLE,
        SUBMERSIBLESAMPLECYCLE,
    }
    public boolean WereInstructionGiven = false;
    public RobotStates robotstate = RobotStates.OCCUPIEDPRELOADS;
    public ElapsedTime time = new ElapsedTime();
    Logitech920 camera;
    @Override
    public void runOpMode() throws InterruptedException {
        RobotInitializers.InitializeFull(hardwareMap);
        Localizer.Reset();
        while(Localizer.pinPoint.getDeviceStatus() == PinPoint.DeviceStatus.CALIBRATING) {
            RobotInitializers.Dashtelemetry.addLine("pinpoint reseting");
            RobotLog.ii("pinpoint","" + Localizer.pinPoint.getDeviceStatus());
        }
        RobotInitializers.InitializeForOperation();

        camera  = new Logitech920("logitik",hardwareMap);
        Scheduler tasks = new Scheduler();
        tasks.AddAnotherScheduler(SimultaniosSampleGrabingScoringActions(CHASSIS_sample1pos,EXTENDO_sample1,0.7,LIFT_preload,OVERHEAD_preload,EXTENSION_overbasket));
        tasks.AddAnotherScheduler(SimultaniosSampleGrabingScoringActions(CHASSIS_sample2pos,EXTENDO_sample2,0.7,LIFT_firstsample,OVERHEAD_firstsample,EXTENSION_overbasket));
        tasks.AddAnotherScheduler(SimultaniosSampleGrabingScoringActions(CHASSIS_sample3pos,EXTENDO_sample3,0.7,LIFT_secondsample,OVERHEAD_secondsample,EXTENSION_overbasket));
        tasks.AddAnotherScheduler(PutLastSamplePredefinedActions(LIFT_thirdsample,OVERHEAD_thirdsample,EXTENSION_overbasket));

        Chassis.usedTrajectory = Chassis.trajectoryStates.FREEWILL;
        Lift.DoingAuto = true;
        Extendo.DoingAuto = true;

        Outtake.setOverHeadPos(OVERHEAD_idle);


        while(opModeInInit()){
            RobotInitializers.clearCache();

            Intake.Block();
            Outtake.openClaw();

            Lift.update();
            Extendo.update();
            Outtake.update();
            Localizer.Update();

            RobotInitializers.Dashtelemetry.addData("x encoder",Localizer.pinPoint.getEncoderX());
            RobotInitializers.Dashtelemetry.addData("y encoder",Localizer.pinPoint.getEncoderY());
        }

        waitForStart();
        time.reset();
        while(opModeIsActive())
        {
            RobotInitializers.clearCache();

            RobotInitializers.Dashtelemetry.addData("is Scheduler done", tasks.tasks.size());

            switch (robotstate){
                case OCCUPIEDPRELOADS:
                    if(tasks.IsSchedulerDone()) {
                        tasks.AddAnotherScheduler(AfterThirdSampleGoToSubmersibleActions());
                        robotstate = RobotStates.CRUISETOSUBMERSIBLE;
                        camera.startCamera();
                    }
                    break;
                case CRUISETOSUBMERSIBLE:
                    if(tasks.IsSchedulerDone())
                    {
                        robotstate = RobotStates.DETECT;
                        WereInstructionGiven = false;
                    }
                    break;
                case DETECT:
                    if(!WereInstructionGiven){
                        tasks.
                                addTask(new Task() {
                                    @Override
                                    protected void Actions() {
                                        Intake.StopSpinner();
                                        Intake.Unblock();
                                    }

                                    @Override
                                    protected boolean Conditions() {
                                        return true;
                                    }
                                });
                        WereInstructionGiven = true;
                    }
                    if(tasks.IsSchedulerDone())//and no blur
                    {
                        besttxty = camera.yellow.getBestTxTy();
                        robotstate = RobotStates.GRABFROMSUBMERSIBLE;
                        WereInstructionGiven = false;
                    }
                    break;

                case GRABFROMSUBMERSIBLE:
                    if(!WereInstructionGiven){
                        tasks.AddAnotherScheduler(TakeCachedSampleActions(besttxty,0.7));
                        WereInstructionGiven = true;
                    }
                    if(Intake.HasWrongTeamPiece() && !Intake.isStorageEmpty())
                    {
                        tasks.clear();
                        robotstate = RobotStates.FAILEDDETECTION;
                        WereInstructionGiven = false;
                    }
                    if(tasks.IsSchedulerDone()){
                        RobotLog.ii("color detected", "" + Intake.getStorageStatus());
                        RobotLog.ii("reached trapdoor", "" + Intake.SampleReachedTrapDoor());

                        if(Intake.HasMixedTeamPiece()) {
                            robotstate = RobotStates.SUBMERSIBLESAMPLECYCLE;
                            RobotLog.ii("accepted","");
                        }
                        else {
                            robotstate = RobotStates.FAILEDDETECTION;
                            RobotLog.ii("rejected","");
                        }
                        WereInstructionGiven = false;
                    }
                    break;
                case FAILEDDETECTION:
                    if(!WereInstructionGiven){
                        tasks.AddAnotherScheduler(FailedToDetectActions());
                        WereInstructionGiven = true;
                    }
                    if(tasks.IsSchedulerDone()){
                        robotstate = RobotStates.DETECT;
                        WereInstructionGiven = false;
                    }
                    break;
                case SUBMERSIBLESAMPLECYCLE:
                    if(!WereInstructionGiven){
                        tasks.AddAnotherScheduler(GoingBackAndForthToHighBasketAndSubmersibleActions());
                        WereInstructionGiven = true;
                    }
                    if(tasks.IsSchedulerDone()){
                        robotstate = RobotStates.CRUISETOSUBMERSIBLE;
                        WereInstructionGiven = false;
                    }
                    break;
            }

            RobotInitializers.Dashtelemetry.addData("currentState",robotstate);

            Chassis.update();
            Lift.update();
            Extendo.update();
            Outtake.update();
            tasks.update();
            Localizer.Update();

            if(time.seconds() > 30)
                break;
        }

    }
}
