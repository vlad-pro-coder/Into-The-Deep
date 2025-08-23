package org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOpLogic;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOpLogic.InitialReset.InitialResetNotParked;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOpLogic.InitialReset.InitialResetParked;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOpLogic.SampleLogic.DropSampleAndRetract.DropSampleAndRetractActions;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOpLogic.SampleLogic.HighBasketScore.HighBasketScoreActions;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOpLogic.SampleLogic.LowBasketScore.LowBasketScoreActions;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOpLogic.PanicActions.PanicElevatorPos.PanicElevatorUpActions;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOpLogic.PanicActions.PanicReset.PanicResetActions;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOpLogic.SampleLogic.TransferSample.TransferSampleSquareActions;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOpLogic.SwitchToSamples.SwitchToSampsActions;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOpLogic.SwitchToSpecimens.SwitchToSpecsActions;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOpLogic.SampleLogic.TransferSample.TransferSampleActions;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.TeleopsStarter.gm1;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.TeleopsStarter.gm2;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Scheduler;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.Samples;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Extendo;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Intake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Lift;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Outtake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.RobotInitializers;

@Config
public class MainHandler {

    Scheduler currentTasks;
    private boolean DidLiftSampleUp = false;
    private boolean TightGripOfClaw = true;
    private boolean TransferColorSpecific = true;
    modes mode = modes.SAMPLES;
    ActionStates CurrentState = ActionStates.SAMPLETRANSFER;
    Gamepad prev1 = new Gamepad();
    Gamepad prev2 = new Gamepad();

    public MainHandler(){
        currentTasks = new Scheduler();
        if(Samples.Wasparked)
            currentTasks.AddAnotherScheduler(InitialResetParked());
        else
            currentTasks.AddAnotherScheduler(InitialResetNotParked());
    }
    public enum ActionStates{
        SAMPLETRANSFER,
        SAMPLEBASKET,
        RETRACTINGSAMPLE,

        SPECIMENGRABANDSTAYIDLE,
        SPECIMENRETRACTFORNEXT,
        SAMPLEOVERHEADOBSERVATIONTHROW,
        SAMPLEOBSERVATIONOPENCLAW,
    }

    public enum modes{
        SAMPLES,
        SPECIMENS,
    }

    public void IntakeActions(){
        if(CurrentState == ActionStates.SAMPLEBASKET && !currentTasks.IsSchedulerDone()) {
            RobotInitializers.Dashtelemetry.addLine("error cannot move extendo in free to move intake");
            return;
        }
        if(gm1.left_bumper && gm1.right_bumper){
            Intake.RotateToEject();
            Intake.Unblock();
        }
        if(gm1.left_bumper){
            //eject
            Intake.RotateToEject(0.5);
            Intake.Unblock();
        }
        else if(gm1.right_bumper) {
            Intake.RotateToStore();
            if(Extendo.getPosition() > 50)
                Intake.DropDown();
            Intake.Unblock();
        }
        else {
            Intake.DropUp();
            Intake.StopSpinner();
            if(Intake.HasMixedTeamPiece() && CurrentState != ActionStates.SAMPLEBASKET)
                Intake.Block();
            else
                Intake.Unblock();
        }
    }


    public void update(){

        IntakeActions();

        /*if(gm2.triangle!=prev2.triangle && gm2.triangle && currentTasks.IsSchedulerDone()){
            if(mode == modes.SAMPLES && CurrentState == ActionStates.SPECIMENGRABANDSTAYIDLE){
                currentTasks.AddAnotherScheduler(SwitchToSpecsActions);
                mode = modes.SPECIMENS;
                CurrentState = ActionStates.SAMPLETRANSFER;
            }
            else if(CurrentState == ActionStates.SAMPLETRANSFER) {
                currentTasks.AddAnotherScheduler(SwitchToSampsActions);
                mode = modes.SAMPLES;
                CurrentState = ActionStates.SPECIMENGRABANDSTAYIDLE;
            }
        }*/

        if(gm1.circle != prev1.circle && gm1.circle)
        {
            currentTasks.clear();
            currentTasks.AddAnotherScheduler(PanicResetActions());
            if(mode == modes.SAMPLES)
                CurrentState = ActionStates.SAMPLETRANSFER;
            else
                CurrentState = ActionStates.SPECIMENGRABANDSTAYIDLE;
        }
        if(gm1.triangle != prev1.triangle && gm1.triangle) {
            currentTasks.clear();
            currentTasks.AddAnotherScheduler(PanicElevatorUpActions());
            if(mode == modes.SAMPLES)
                CurrentState = ActionStates.SAMPLETRANSFER;
            else
                CurrentState = ActionStates.SPECIMENGRABANDSTAYIDLE;
        }

        if(gm2.square != prev2.square && gm2.square)
            TransferColorSpecific = !TransferColorSpecific;


        switch (mode){
            case SAMPLES:
                switch (CurrentState){
                    case SAMPLETRANSFER:
                        if(gm1.square != prev1.square && gm1.square) {
                            currentTasks.clear();
                            currentTasks.AddAnotherScheduler(TransferSampleSquareActions());
                            CurrentState = ActionStates.SAMPLEBASKET;
                        }
                        if(currentTasks.IsSchedulerDone() && ((TransferColorSpecific && Intake.HasMixedTeamPiece()) || (!TransferColorSpecific && Intake.HasOnlyNeutre()))) {//change
                            currentTasks.AddAnotherScheduler(TransferSampleActions());
                            CurrentState = ActionStates.SAMPLEBASKET;
                        }
                        break;
                    case SAMPLEBASKET:
                        if(true) {
                            if(gm1.dpad_up != prev1.dpad_up && gm1.dpad_up) {
                                currentTasks.AddAnotherScheduler(HighBasketScoreActions());
                                Lift.currentPos = 0;
                                DidLiftSampleUp = true;
                            }
                            if(gm1.dpad_down != prev1.dpad_down && gm1.dpad_down) {
                                currentTasks.AddAnotherScheduler(LowBasketScoreActions());
                                Lift.currentPos = 0;
                                DidLiftSampleUp = true;
                            }
                            if(currentTasks.IsSchedulerDone() && DidLiftSampleUp){
                                Outtake.ExtensionMoveWhenOverBasket(Math.abs(gm2.right_stick_y) < 0.05 ? 0:-gm2.right_stick_y);
                                int left_bumper = gm2.left_bumper ? 3 : 0;
                                int right_bumper = gm2.right_bumper ? 3 : 0;
                                Outtake.OverheadMoveWhenOverBasket(right_bumper - left_bumper);
                            }
                            if(gm1.cross != prev1.cross && gm1.cross && DidLiftSampleUp && currentTasks.IsSchedulerDone() && TightGripOfClaw) {
                                Outtake.closeClawTightTeleop();
                                TightGripOfClaw = !TightGripOfClaw;
                            }
                            else if(gm1.cross != prev1.cross && gm1.cross && DidLiftSampleUp && currentTasks.IsSchedulerDone() && !TightGripOfClaw) {
                                Outtake.closeClawTight();
                                TightGripOfClaw = !TightGripOfClaw;
                            }
                            if(gm1.square != prev1.square && gm1.square && DidLiftSampleUp)
                                CurrentState = ActionStates.RETRACTINGSAMPLE;
                        }
                        break;
                    case RETRACTINGSAMPLE:
                        currentTasks.AddAnotherScheduler(DropSampleAndRetractActions());
                        CurrentState = ActionStates.SAMPLETRANSFER;
                        DidLiftSampleUp = false;
                        break;
                }
                break;
            case SPECIMENS:


                break;
        }

        RobotInitializers.Dashtelemetry.addData("length current tasks",currentTasks.tasks.size());

        currentTasks.update();
        prev1.copy(gm1);
        prev2.copy(gm2);
    }
}
