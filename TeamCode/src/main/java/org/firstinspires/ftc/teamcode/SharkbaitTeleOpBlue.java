/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.Limelight3A;


import org.firstinspires.ftc.teamcode.constants.TeleOpServoConstants;


import java.lang.Math;
import java.util.List;
import java.util.TimerTask;
import java.util.Timer;




/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */


@TeleOp(name="Sharkbait TeleOp BLUE")


public class SharkbaitTeleOpBlue extends LinearOpMode {


    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FLMotor = null;
    private DcMotor FRMotor = null;
    private DcMotor BLMotor = null;
    private DcMotor BRMotor = null;


    private DcMotor InSlide = null;
    private DcMotor OtSlideF = null;
    private DcMotor OtSlideM = null;
    private DcMotor OtSlideB = null;


    private Servo InLDiffyServo = null;
    private Servo InRDiffyServo = null;
    private Servo InLeftElbowServo = null;
    private Servo InRightElbowServo = null;
    private Servo InClawServo = null;


    private Servo OtClawServo = null;
    private Servo OtLeftElbowServo = null;
    private Servo OtRightElbowServo = null;
    private Servo OtCoaxialServo = null;
    private Servo OtWristServo = null;
    private Servo OtLinkageLServo = null;
    private Servo OtLinkageRServo = null;
    private Limelight3A litty = null;


    private int intakeIndex = 0;
    private int outtakeIndex = 1;


    private int diffyRotateIndex = 0;


    private boolean inClawDelay = false;
    private boolean otClawDelay = false;
    private boolean diffyRotateDelay = false;
    private boolean inClawOpened = true;
    private boolean otClawOpened = true;


    private boolean oldCrossPressed = true;
    private boolean oldTrianglePressed = true;
    private boolean oldCirclePressed = true;
    private boolean oldSquarePressed = true;
    private boolean oldUpDpadPressed = true;
    private boolean oldDownDpadPressed = true;
    private boolean oldLeftDpadPressed = true;
    private boolean oldRightDpadPressed = true;
    private boolean oldLeftBumper = true;
    private boolean oldRightBumper = true;
    private double oldLeftTrigger = 0.0;
    private double oldRightTrigger = 0.0;
    private boolean babyMode = false;
    private boolean oldLeftStickButton = true;
    private boolean oldRightStickButton = true;


     private boolean intakeManualMode = true;
     private boolean outtakeManualMode = true;


    private boolean wasRightDPadPressed = false;
    private boolean wasUpDPadPressed = false;
    private boolean wasTrianglePressed = false;
    private boolean wasCrossPressed = false;


    private boolean isInArmMoving = false;
    private boolean isOtArmMoving = false;


    private boolean isInDiffyMoving = false;
    private boolean isOtCoaxialMoving = false;


    private double[] ILEServoPositions = TeleOpServoConstants.ILEServoPositions;
    private double[] IREServoPositions = TeleOpServoConstants.IREServoPositions;
    private double[] OLEServoPositions = TeleOpServoConstants.OLEServoPositions;
    private double[] OREServoPositions = TeleOpServoConstants.OREServoPositions;
    private double[] LDServoPositions = TeleOpServoConstants.LDServoPositions;
    private double[] RDServoPositions = TeleOpServoConstants.RDServoPositions;
    private double[] OAServoPositions = TeleOpServoConstants.OAServoPositions;
    private double[] ICServoPositions = TeleOpServoConstants.ICServoPositions;
    private double[] OCServoPositions = TeleOpServoConstants.OCServoPositions;
    private double[] OKLServoPositions = TeleOpServoConstants.OKLServoPositions;
    private double[] OKRServoPositions = TeleOpServoConstants.OKRServoPositions;
    private double[] OWServoPositions = TeleOpServoConstants.OWServoPositions;


    private int[] OtSlideFPositions = TeleOpServoConstants.OtSlideFPositions;
   private int[] OtSlideMPositions = TeleOpServoConstants.OtSlideMPositions;
   private int[] OtSlideBPositions = TeleOpServoConstants.OtSlideBPositions;
   private int[] InSlidePositions = TeleOpServoConstants.InSlidePositions;


    private double[] SlowModeSpeed = TeleOpServoConstants.SlowModeSpeed;
    private double speedfor = SlowModeSpeed[1];
    private double speedbac = SlowModeSpeed[0];


    private int inPosition = 1;




    private final int DELAY_BETWEEN_MOVES = 100;




    @Override
    public void runOpMode() {
        double degrees = 0;


        class setIsInArmMoving extends TimerTask {
            boolean val;
            public setIsInArmMoving(boolean v) {
                this.val = v;
            }
            public void run() {
                isInArmMoving = val;
            }
        }
        class setIsOtArmMoving extends TimerTask {
            boolean val;
            public setIsOtArmMoving(boolean v) {
                this.val = v;
            }
            public void run() {
                isOtArmMoving = val;
            }
        }
        class setIsOtCoaxialMoving extends TimerTask {
            boolean val;


            public setIsOtCoaxialMoving(boolean v) {
                this.val = v;
            }
            public void run() {
                isOtCoaxialMoving = val;
            }
        }
        class setInClawDelay extends TimerTask {
            boolean val;
            public setInClawDelay(boolean v) { this.val = v; }
            public void run() {
                inClawDelay = val;
            }
        }
        class OpenIn extends TimerTask {
            public OpenIn(){}
            public void run(){ InClawServo.setPosition(ICServoPositions[1]); }
        }
        class CloseIn extends TimerTask {
            public CloseIn(){}
            public void run(){ InClawServo.setPosition(ICServoPositions[0]); }
        }




        class setIsInDiffyMoving extends TimerTask {
            boolean val;
            public setIsInDiffyMoving(boolean v) {
                this.val = v;
            }
            public void run() {
                isInDiffyMoving = val;
            }
        }


         class setIntakeManualMode extends TimerTask {
         boolean val;
          public setIntakeManualMode(boolean v) {
             this.val = v;
          }
           public void run() {
              intakeManualMode = val;
               if (wasRightDPadPressed) wasRightDPadPressed = !val;
             if (wasUpDPadPressed) wasUpDPadPressed = !val;
          }
        }


       class setOuttakeManualMode extends TimerTask {
          boolean val;
           public setOuttakeManualMode(boolean v) {
              this.val = v;
            }
           public void run() {
               outtakeManualMode = val;
             if (wasTrianglePressed) wasTrianglePressed = !val;
              if (wasCrossPressed) wasCrossPressed = !val;
           }
        }


             class OtSlidesPosition extends TimerTask {
               int i1;
               int i2;
               int i3;
               double power;


             public OtSlidesPosition(int i1, int i2, int i3, double power) {
              this.i1 = i1;
              this.i2 = i2;
              this.i3 = i3;
              this.power = power;
            }
             public void run() {
        OtSlideF.setTargetPosition(OtSlideFPositions[i1]);
        OtSlideF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        OtSlideF.setPower(power);


        OtSlideM.setTargetPosition(OtSlideFPositions[i2]);
        OtSlideM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        OtSlideM.setPower(power);


        OtSlideB.setTargetPosition(OtSlideFPositions[i3]);
        OtSlideB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        OtSlideB.setPower(power);
               }
          }


            class InSlidePosition extends TimerTask {
            int i;
            double power;

             public InSlidePosition(int i, double power) {
                 this.i = i;
                 this.power = power;
            }
            public void run() {
        InSlide.setTargetPosition(InSlidePositions[i]);
        InSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        InSlide.setPower(power);
            }
           }


        class setOtClawDelay extends TimerTask {
            boolean val;
            public setOtClawDelay(boolean v) {
                this.val = v;
            }
            public void run() {
                otClawDelay = val;
            }
        }




        class OuttakeExtension extends TimerTask {
            double i;
            public OuttakeExtension(double i) { this.i = i; }
            public void run() {
                OtSlideF.setPower(i);
                OtSlideM.setPower(i);
                OtSlideB.setPower(i);
            }
        }


        class MoveInElbowServosPosition extends TimerTask {
            int i;
            public MoveInElbowServosPosition(int i) {
                this.i = i;
            }
            public void run() {
                InLeftElbowServo.setPosition(ILEServoPositions[i]);
                InRightElbowServo.setPosition(IREServoPositions[i]);


                telemetry.addData("intakeIndex", i);
                telemetry.update();
                intakeIndex = i;
            }
        }
        class MoveOtElbowServosPosition extends TimerTask {
            int i;
            public MoveOtElbowServosPosition(int i) {
                this.i = i;
            }
            public void run() {
                OtLeftElbowServo.setPosition(OLEServoPositions[i]);
                OtRightElbowServo.setPosition(OREServoPositions[i]);


                telemetry.addData("intakeIndex", i);
                telemetry.update();
                outtakeIndex = i;
            }
        }


        class MoveInDiffyServoPosition extends TimerTask {
            int i;
            double k;
            public MoveInDiffyServoPosition(int i) { this.i = i; }
            public MoveInDiffyServoPosition(double k) { this.k = k; }
            public void run() {
                InLDiffyServo.setPosition(LDServoPositions[i]);
                InRDiffyServo.setPosition(RDServoPositions[i]);




                //InLDiffyServo.setPosition(InLDiffyServo.getPosition() + i);
                //InRDiffyServo.setPosition(InRDiffyServo.getPosition() + i);
            }
        }


        class MoveOtCoaxialServoPosition extends TimerTask {
            int i;
            public MoveOtCoaxialServoPosition(int i) {
                this.i = i;
            }
            public void run() {
                OtCoaxialServo.setPosition(OAServoPositions[i]);
            }
        }
        class MoveOtWristServoPosition extends TimerTask {
            int i;
            public MoveOtWristServoPosition(int i) {
                this.i = i;
            }
            public void run() {
                OtWristServo.setPosition(OWServoPositions[i]);
            }
        }


        class MoveOtLinkageServoPosition extends TimerTask {
            int i;
            public MoveOtLinkageServoPosition(int i) {
                this.i = i;
            }
            public void run() {
                OtLinkageLServo.setPosition(OKLServoPositions[i]);
                OtLinkageRServo.setPosition(OKRServoPositions[i]);
            }
        }


        class MoveOtClawServoPosition extends TimerTask {
            int i;
            public MoveOtClawServoPosition(int i) {
                this.i = i;
            }
            public void run() {
                OtClawServo.setPosition(OCServoPositions[i]);
            }
        }

        class MoveInClawServoPosition extends TimerTask {
            int i;
            public MoveInClawServoPosition(int i) {
                this.i = i;
            }
            public void run() {
                InClawServo.setPosition(ICServoPositions[i]);
            }
        }




        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        FLMotor = hardwareMap.get(DcMotor.class, "FL");
        FRMotor = hardwareMap.get(DcMotor.class, "FR");
        BLMotor = hardwareMap.get(DcMotor.class, "BL");
        BRMotor = hardwareMap.get(DcMotor.class, "BR");
        InSlide = hardwareMap.get(DcMotor.class, "IS");
        OtSlideF = hardwareMap.get(DcMotor.class, "OF");
        OtSlideM = hardwareMap.get(DcMotor.class, "OM");
        OtSlideB = hardwareMap.get(DcMotor.class, "OB");


        InLeftElbowServo = hardwareMap.get(Servo.class, "ILE");
        InRightElbowServo = hardwareMap.get(Servo.class, "IRE");
        InClawServo = hardwareMap.get(Servo.class, "IC");
        InLDiffyServo = hardwareMap.get(Servo.class, "LD");
        InRDiffyServo = hardwareMap.get(Servo.class, "RD");


        OtLeftElbowServo = hardwareMap.get(Servo.class, "OLE");
        OtRightElbowServo = hardwareMap.get(Servo.class, "ORE");
        OtWristServo = hardwareMap.get(Servo.class, "OW");
        OtClawServo = hardwareMap.get(Servo.class, "OC");
        OtCoaxialServo = hardwareMap.get(Servo.class, "OA");
        OtLinkageLServo = hardwareMap.get(Servo.class, "OKL");
        OtLinkageRServo = hardwareMap.get(Servo.class, "OKR");
        Timer timer = new Timer();


        FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        InSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        OtSlideF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        OtSlideM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        OtSlideB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //In the future will need code for Encoders on the Motors


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        FLMotor.setDirection(DcMotor.Direction.REVERSE);
        FRMotor.setDirection(DcMotor.Direction.FORWARD);
        BLMotor.setDirection(DcMotor.Direction.REVERSE);
        BRMotor.setDirection(DcMotor.Direction.FORWARD);


        InSlide.setDirection(DcMotor.Direction.REVERSE);
        OtSlideF.setDirection(DcMotor.Direction.REVERSE); //all 3 of these are backwards
        OtSlideM.setDirection(DcMotor.Direction.FORWARD);
        OtSlideB.setDirection(DcMotor.Direction.REVERSE);


        OtSlideB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        OtSlideB.setTargetPosition(0);
        OtSlideB.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        OtSlideF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        OtSlideF.setTargetPosition(0);
        OtSlideF.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        OtSlideM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        OtSlideM.setTargetPosition(0);
        OtSlideM.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        InSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        InSlide.setTargetPosition(0);
        InSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);




        InLeftElbowServo.setDirection(Servo.Direction.REVERSE);
        InRightElbowServo.setDirection(Servo.Direction.FORWARD);
        OtLeftElbowServo.setDirection(Servo.Direction.FORWARD);
        OtRightElbowServo.setDirection(Servo.Direction.REVERSE);


        InClawServo.setDirection(Servo.Direction.REVERSE);
        OtClawServo.setDirection(Servo.Direction.REVERSE);


        OtCoaxialServo.setDirection(Servo.Direction.REVERSE);
        OtWristServo.setDirection(Servo.Direction.FORWARD);
        OtLinkageLServo.setDirection(Servo.Direction.REVERSE);
        OtLinkageRServo.setDirection(Servo.Direction.REVERSE);


        InLDiffyServo.setDirection(Servo.Direction.FORWARD);
        InRDiffyServo.setDirection(Servo.Direction.REVERSE);


        OtSlideF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        OtSlideM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        OtSlideB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        InSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        litty = hardwareMap.get(Limelight3A.class, "litty");
        litty.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        telemetry.setMsTransmissionInterval(11);
        litty.pipelineSwitch(2);
        litty.start();


        RDServoPositions[1] = 0.5;
        LDServoPositions[1] = 0.5;


        InLDiffyServo.setPosition(LDServoPositions[1]);//3 no auto
        InRDiffyServo.setPosition(RDServoPositions[1]);//3 no auto
        InClawServo.setPosition(ICServoPositions[0]); //0
        OtClawServo.setPosition(OCServoPositions[0]); //0
        OtWristServo.setPosition(OWServoPositions[1]); //0
        OtLinkageLServo.setPosition(OKLServoPositions[1]); //0
        OtLinkageRServo.setPosition(OKRServoPositions[1]); //0
        InLeftElbowServo.setPosition(ILEServoPositions[1]);//3 no auto
        InRightElbowServo.setPosition(IREServoPositions[1]);//3 no auto
        OtLeftElbowServo.setPosition(OLEServoPositions[1]); //0
        OtRightElbowServo.setPosition(OREServoPositions[1]); //0ea+
        OtCoaxialServo.setPosition(OAServoPositions[1]); //0




        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();





        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            double FLPower = r * Math.cos(robotAngle) + rightX;
            double FRPower = r * Math.sin(robotAngle) - rightX;
            double BLPower = r * Math.sin(robotAngle) + rightX;
            double BRPower = r * Math.cos(robotAngle) - rightX;

            double divider = Math.abs(Math.max(FLPower, FRPower));
            divider = Math.abs(Math.max(divider, BLPower));
            divider = Math.abs(Math.max(divider, BRPower));
            if(gamepad2.right_stick_x < 0.1 && divider != 1){
                FLPower = FLPower / divider;
                FRPower = FRPower / divider;
                BLPower = BLPower / divider;
                BRPower = BRPower / divider;
            }


            // Send calculated power to wheels
            double rt = gamepad1.right_trigger;
            double lt = gamepad1.left_trigger;
            boolean rB = gamepad1.right_bumper;
            boolean lB = gamepad1.left_bumper;
            if(gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
                if(gamepad1.dpad_up) { //all positive
                    FLMotor.setPower(SlowModeSpeed[1]);
                    FRMotor.setPower(SlowModeSpeed[1]);
                    BLMotor.setPower(SlowModeSpeed[1]);
                    BRMotor.setPower(SlowModeSpeed[1]);
                } else if (gamepad1.dpad_down){ //all negative
                    FLMotor.setPower(SlowModeSpeed[0]);
                    FRMotor.setPower(SlowModeSpeed[0]);
                    BLMotor.setPower(SlowModeSpeed[0]);
                    BRMotor.setPower(SlowModeSpeed[0]);
                } else if (gamepad1.dpad_left){
                    FLMotor.setPower(SlowModeSpeed[0]);
                    FRMotor.setPower(SlowModeSpeed[1]);
                    BLMotor.setPower(SlowModeSpeed[0]);
                    BRMotor.setPower(SlowModeSpeed[1]);
                } else if (gamepad1.dpad_right){
                    FLMotor.setPower(SlowModeSpeed[1]);
                    FRMotor.setPower(SlowModeSpeed[0]);
                    BLMotor.setPower(SlowModeSpeed[1]);
                    BRMotor.setPower(SlowModeSpeed[0]);
                }
            } else if (rt > 0.2 || rB || lB || lt > 0.2) {
                if (-gamepad1.left_stick_y > 0) {
                    if (rt > 0.2 || rB) {
                        FLMotor.setPower(speedfor);
                        FRMotor.setPower(speedbac + (Math.abs(gamepad1.left_stick_y) * 0.5));
                        BLMotor.setPower(speedbac + (Math.abs(gamepad1.left_stick_y) * 0.5));
                        BRMotor.setPower(speedfor);
                    } else {
                        FLMotor.setPower(speedbac + (Math.abs(gamepad1.left_stick_y) * 0.5));
                        FRMotor.setPower(speedfor);
                        BLMotor.setPower(speedfor);
                        BRMotor.setPower(speedbac + (Math.abs(gamepad1.left_stick_y) * 0.5));
                    }
                } else {
                    if (rt > 0.2 || rB) {
                        FLMotor.setPower(speedfor - (Math.abs(gamepad1.left_stick_y) * 0.5));
                        FRMotor.setPower(speedbac);
                        BLMotor.setPower(speedbac);
                        BRMotor.setPower(speedfor - (Math.abs(gamepad1.left_stick_y) * 0.5));
                    } else {
                        FLMotor.setPower(speedbac);
                        FRMotor.setPower(speedfor - (Math.abs(gamepad1.left_stick_y) * 0.5));
                        BLMotor.setPower(speedfor - (Math.abs(gamepad1.left_stick_y) * 0.5));
                        BRMotor.setPower(speedbac);
                    }
                }
            /*} else if (rB || lB) {
                if(rB){
                    FLMotor.setPower(SlowModeSpeed[1]);
                    FRMotor.setPower(SlowModeSpeed[0]);
                    BLMotor.setPower(SlowModeSpeed[0]);
                    BRMotor.setPower(SlowModeSpeed[1]);
                } else {
                    FLMotor.setPower(SlowModeSpeed[0]);
                    FRMotor.setPower(SlowModeSpeed[1]);
                    BLMotor.setPower(SlowModeSpeed[1]);
                    BRMotor.setPower(SlowModeSpeed[0]);
                }*/
            } else {
                FLMotor.setPower(FLPower);
                FRMotor.setPower(FRPower);
                BLMotor.setPower(BLPower);
                BRMotor.setPower(BRPower);
            }


            telemetry.addData("praying for a new life", OtSlideB.getCurrentPosition());


            boolean circlePressed = gamepad2.circle;
            boolean trianglePressed = gamepad2.triangle;
            boolean squarePressed = gamepad2.square;
            boolean crossPressed = gamepad2.cross;
            boolean dpadUpPressed = gamepad2.dpad_up;
            boolean dpadDownPressed = gamepad2.dpad_down;
            boolean dpadLeftPressed = gamepad2.dpad_left;
            boolean dpadRightPressed = gamepad2.dpad_right;
            boolean leftBumper = gamepad2.left_bumper;
            boolean rightBumper = gamepad2.right_bumper;
            double leftTrigger = gamepad2.left_trigger;
            double rightTrigger = gamepad2.right_trigger;




            if (leftBumper && intakeManualMode && !inClawDelay && !oldLeftBumper) { //close Intake claw
                new setInClawDelay(true).run();
                if (inClawOpened) { InClawServo.setPosition(ICServoPositions[1]); } //open Intake Claw
                else { InClawServo.setPosition(ICServoPositions[0]); }//close Intake Claw
                timer.schedule(new setInClawDelay(false), 3 * DELAY_BETWEEN_MOVES);
                inClawOpened = !inClawOpened;
            }


            if (rightBumper && !otClawDelay && !oldRightBumper) { //close Outtake claw
                new setOtClawDelay(true).run();

                 if (outtakeIndex == 3 && otClawOpened) {
                    timer.schedule(new MoveOtClawServoPosition(1), 0 * DELAY_BETWEEN_MOVES);
                    if (inClawOpened) timer.schedule(new MoveInClawServoPosition(1), 2 *DELAY_BETWEEN_MOVES);
                    else timer.schedule(new MoveInClawServoPosition(0), 2 *DELAY_BETWEEN_MOVES);
                } else {
                    if (otClawOpened) {
                        OtClawServo.setPosition(OCServoPositions[1]);
                    } // close Outtake claw
                    else {
                        OtClawServo.setPosition(OCServoPositions[0]);
                    } //open Outtake claw

                }
                timer.schedule(new setOtClawDelay(false), 3 * DELAY_BETWEEN_MOVES);
                otClawOpened = !otClawOpened;
            }

            if (outtakeManualMode) {
            OtSlideF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            OtSlideM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            OtSlideB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            OtSlideF.setPower(gamepad2.right_stick_y);
            OtSlideM.setPower(gamepad2.right_stick_y);
            OtSlideB.setPower(gamepad2.right_stick_y);
            }
           if (crossPressed && !oldCrossPressed && !isOtArmMoving) { //pickup sample from intake outtakeIndex = 3
               new setIsOtArmMoving(true).run();
               new setIsOtCoaxialMoving(true).run();
               new setOtClawDelay(true).run();
               timer.schedule(new MoveOtClawServoPosition(1), 0 * DELAY_BETWEEN_MOVES);
               timer.schedule(new MoveOtLinkageServoPosition(2), 0 * DELAY_BETWEEN_MOVES);
               timer.schedule(new MoveOtCoaxialServoPosition(4), 4 * DELAY_BETWEEN_MOVES);
               timer.schedule(new MoveOtElbowServosPosition(3), 8 * DELAY_BETWEEN_MOVES);
               if (!inClawOpened) timer.schedule(new MoveOtWristServoPosition(0), 6 *DELAY_BETWEEN_MOVES);
               else timer.schedule(new MoveOtWristServoPosition(1), 6 *DELAY_BETWEEN_MOVES);
               if (otClawOpened) timer.schedule(new MoveOtClawServoPosition(0), 6 * DELAY_BETWEEN_MOVES);


               timer.schedule(new setIsOtArmMoving(false), 10 *DELAY_BETWEEN_MOVES);
               timer.schedule(new setIsOtCoaxialMoving(false), 10 *DELAY_BETWEEN_MOVES);
               timer.schedule(new setOtClawDelay(false), 10 *DELAY_BETWEEN_MOVES);


           }else if (circlePressed && !oldCirclePressed && !isOtArmMoving && outtakeIndex != 0) { //pickup specimen from wall outtakeIndex = 0
                new setIsOtArmMoving(true).run();
                new setOuttakeManualMode(false).run();
                new setIsOtCoaxialMoving(true).run();
                new setOtClawDelay(true).run();
                timer.schedule(new OtSlidesPosition(0,0,0,1), 0*DELAY_BETWEEN_MOVES);
                if (outtakeIndex == 2) {
                    timer.schedule(new MoveOtClawServoPosition(1), 0 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new MoveOtLinkageServoPosition(0), 0 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new MoveOtCoaxialServoPosition(0), 0 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new MoveOtElbowServosPosition(0), 2 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new MoveOtWristServoPosition(0), 6 * DELAY_BETWEEN_MOVES);
                    if (otClawOpened) timer.schedule(new MoveOtClawServoPosition(0), 7 * DELAY_BETWEEN_MOVES);


                    timer.schedule(new setIsOtArmMoving(false), 8 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new setIsOtCoaxialMoving(false), 8 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new setOtClawDelay(false), 8 *DELAY_BETWEEN_MOVES);
                }

                if (outtakeIndex == 1 || outtakeIndex == 3) {
                    timer.schedule(new MoveOtClawServoPosition(1), 0 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new MoveOtLinkageServoPosition(0), 0 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new MoveOtCoaxialServoPosition(2), 4 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new MoveOtWristServoPosition(0), 6 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new MoveOtElbowServosPosition(2), 6 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new MoveOtCoaxialServoPosition(0), 7 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new MoveOtElbowServosPosition(0), 9 * DELAY_BETWEEN_MOVES);
                    if (otClawOpened) timer.schedule(new MoveOtClawServoPosition(0), 10 * DELAY_BETWEEN_MOVES);


                    timer.schedule(new setIsOtArmMoving(false), 11 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new setIsOtCoaxialMoving(false), 11 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new setOtClawDelay(false), 11 *DELAY_BETWEEN_MOVES);
                }
                timer.schedule(new setOuttakeManualMode(true), 15 * DELAY_BETWEEN_MOVES);


            } else if (trianglePressed && !oldTrianglePressed && !isOtArmMoving) { //outtake from back of robot outtakeIndex = 2
                new setIsOtArmMoving(true).run();
                new setOuttakeManualMode(false).run();
                new setIsOtCoaxialMoving(true).run();
               new setOtClawDelay(true).run();

               timer.schedule(new MoveOtClawServoPosition(1), 0 * DELAY_BETWEEN_MOVES);
                timer.schedule(new OtSlidesPosition(1,1,1,1), 0*DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveOtLinkageServoPosition(0), 0 *DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveOtCoaxialServoPosition(2), 4 * DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveOtElbowServosPosition(2), 6 *DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveOtWristServoPosition(0), 6 *DELAY_BETWEEN_MOVES);
               if (otClawOpened) timer.schedule(new MoveOtClawServoPosition(0), 7 * DELAY_BETWEEN_MOVES);


               timer.schedule(new setIsOtArmMoving(false), 9 *DELAY_BETWEEN_MOVES);
               timer.schedule(new setIsOtCoaxialMoving(false), 9 *DELAY_BETWEEN_MOVES);
               timer.schedule(new setOuttakeManualMode(true), 15 * DELAY_BETWEEN_MOVES);
               timer.schedule(new setOtClawDelay(false), 9 *DELAY_BETWEEN_MOVES);



           } else if (squarePressed && !oldSquarePressed && !isOtArmMoving) { //outtake from front of robot outtakeIndex = 1
                new setIsOtArmMoving(true).run();
                new setIsOtCoaxialMoving(true).run();
                new setOtClawDelay(true).run();

                if (inPosition == 3) {
                    new setIsInArmMoving(true).run();
                    new setIsInDiffyMoving(true).run();
                    timer.schedule(new MoveInDiffyServoPosition(2), 4 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new MoveInElbowServosPosition(2), 0 *DELAY_BETWEEN_MOVES);

                    timer.schedule(new setIsInArmMoving(false), 4 *DELAY_BETWEEN_MOVES);
                    timer.schedule(new setIsInDiffyMoving(false), 4 *DELAY_BETWEEN_MOVES);
                    inPosition = 2;
                }
               timer.schedule(new MoveOtClawServoPosition(1), 0 * DELAY_BETWEEN_MOVES);
               timer.schedule(new MoveOtCoaxialServoPosition(2), 0 * DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveOtElbowServosPosition(2), 2 *DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveOtLinkageServoPosition(1), 4 *DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveOtWristServoPosition(1), 4 *DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveOtCoaxialServoPosition(1), 6 * DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveOtElbowServosPosition(1), 8 *DELAY_BETWEEN_MOVES);
               if (otClawOpened) timer.schedule(new MoveOtClawServoPosition(0), 9 * DELAY_BETWEEN_MOVES);


               timer.schedule(new setIsOtArmMoving(false), 10 *DELAY_BETWEEN_MOVES);
                timer.schedule(new setIsOtCoaxialMoving(false), 10 *DELAY_BETWEEN_MOVES);
               timer.schedule(new setOtClawDelay(false), 10 *DELAY_BETWEEN_MOVES);



           }

            if(gamepad2.left_stick_y < 0 && Math.abs(InSlide.getCurrentPosition()) < 1200){
                if(babyMode){
                    InSlide.setPower(gamepad2.left_stick_y * 0.4);
                }else{
                    InSlide.setPower(gamepad2.left_stick_y);
                }
            }else if(gamepad2.right_stick_y >= 0){
                if(babyMode){
                    InSlide.setPower(gamepad2.left_stick_y * 0.4);
                }else{
                    InSlide.setPower(gamepad2.left_stick_y);
                }
            } else {
                InSlide.setPower(0
                );
            }


            if (dpadDownPressed && !oldDownDpadPressed && !isInArmMoving) { //intake position intakeIndex = 0
                new setIsInArmMoving(true).run();
                //new setIsInDiffyMoving(true).run();
                //timer.schedule(new MoveInDiffyServoPosition(0), 0 * DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveInElbowServosPosition(0), 0 *DELAY_BETWEEN_MOVES);
                timer.schedule(new setIsInArmMoving(false), 2 *DELAY_BETWEEN_MOVES);
                //timer.schedule(new setIsInDiffyMoving(false), 4 *DELAY_BETWEEN_MOVES);
                inPosition = 0;
            } else if (dpadRightPressed && !oldRightDpadPressed && !isInArmMoving) { //position for getting over submersible walls intakeIndex = 2
                //new setIntakeManualMode(false).run();
                new setIsInArmMoving(true).run();
                new setIsInDiffyMoving(true).run();
                //timer.schedule(new InSlidePosition(1, 1), 0 * DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveInDiffyServoPosition(2), 0 * DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveInElbowServosPosition(2), 0 *DELAY_BETWEEN_MOVES);

                timer.schedule(new setIsInArmMoving(false), 4 *DELAY_BETWEEN_MOVES);
                timer.schedule(new setIsInDiffyMoving(false), 4 *DELAY_BETWEEN_MOVES);
                //timer.schedule(new setIntakeManualMode(true), 12 *DELAY_BETWEEN_MOVES);
                inPosition = 2;
            } else if (dpadLeftPressed && !oldLeftDpadPressed && !isInArmMoving) { //position for scanning for samples intakeIndex = 1
                new setIsInArmMoving(true).run();
                new setIsInDiffyMoving(true).run();
                LDServoPositions[1] = 0.5;
                RDServoPositions[1] = 0.5;
                timer.schedule(new MoveInElbowServosPosition(5), 0 * DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveInDiffyServoPosition(1), 2 * DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveInElbowServosPosition(1), 1 *DELAY_BETWEEN_MOVES);


                timer.schedule(new setIsInArmMoving(false), 4 *DELAY_BETWEEN_MOVES);
                timer.schedule(new setIsInDiffyMoving(false), 4 *DELAY_BETWEEN_MOVES);
                inPosition = 1;
            } else if (dpadUpPressed && !oldUpDpadPressed && !isInArmMoving) { //position for transferring sample to outtake intakeIndex = 3
               new setIsInArmMoving(true).run();
               new setIsInDiffyMoving(true).run();
               timer.schedule(new MoveInDiffyServoPosition(4), 0 * DELAY_BETWEEN_MOVES);
               timer.schedule(new MoveInElbowServosPosition(4), 2 *DELAY_BETWEEN_MOVES);

               timer.schedule(new setIsInArmMoving(false), 5 *DELAY_BETWEEN_MOVES);
               timer.schedule(new setIsInDiffyMoving(false), 5 *DELAY_BETWEEN_MOVES);
               inPosition = 3;

                /*
                new setIsOtArmMoving(true).run();
                new setIsOtCoaxialMoving(true).run();
                new setOtClawDelay(true).run();
                timer.schedule(new MoveOtClawServoPosition(1), 0 * DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveOtLinkageServoPosition(2), 5 * DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveOtCoaxialServoPosition(4), 9 * DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveOtClawServoPosition(0), 18 * DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveOtElbowServosPosition(3), 14 * DELAY_BETWEEN_MOVES);
                if (!inClawOpened) timer.schedule(new MoveOtWristServoPosition(2), 10 *DELAY_BETWEEN_MOVES);
                else timer.schedule(new MoveOtWristServoPosition(1), 10 *DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveOtClawServoPosition(1), 22 * DELAY_BETWEEN_MOVES);
                timer.schedule(new OpenIn(), 22 * DELAY_BETWEEN_MOVES);


                timer.schedule(new MoveOtLinkageServoPosition(0), 25 * DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveOtCoaxialServoPosition(2), 29 * DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveOtWristServoPosition(0), 29 * DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveOtElbowServosPosition(2), 29 * DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveOtCoaxialServoPosition(0), 30 * DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveOtElbowServosPosition(0), 32 * DELAY_BETWEEN_MOVES);

                timer.schedule(new setIsOtArmMoving(false), 30 *DELAY_BETWEEN_MOVES);
                timer.schedule(new setIsOtCoaxialMoving(false), 30 *DELAY_BETWEEN_MOVES);
                timer.schedule(new setOtClawDelay(false), 30 *DELAY_BETWEEN_MOVES);
                */
           }

            if ((inPosition == 1 || inPosition == 0) && intakeManualMode) {
                if (leftTrigger > 0.1 /*&& oldLeftTrigger < 0.2 && !isInDiffyMoving*/) {
                    new setIsInDiffyMoving(true).run();
                    if (LDServoPositions[1] >= 0.02 && RDServoPositions[1] <= 0.98) {
                        LDServoPositions[1] = LDServoPositions[1] - 0.01;
                        RDServoPositions[1] = RDServoPositions[1] + 0.01;
                    }
//                    LDServoPositions[1] = Math.max(0, LDServoPositions[1] - 0.15);
//                    RDServoPositions[1] = Math.min(1, RDServoPositions[1] + 0.15);
                    new MoveInDiffyServoPosition(1).run();
                    timer.schedule(new setIsInDiffyMoving(false), 0);
                } else if (rightTrigger > 0.1 /*&& oldRightTrigger < 0.2 && !isInDiffyMoving*/) {
                    new setIsInDiffyMoving(true).run();
                    if (LDServoPositions[1] <= 0.98 && RDServoPositions[1] >= 0.02) {
                        LDServoPositions[1] = LDServoPositions[1] + 0.01;
                        RDServoPositions[1] = RDServoPositions[1] - 0.01;
                    }
//                    LDServoPositions[1] = Math.min(1, LDServoPositions[1] + 0.15);
//                    RDServoPositions[1] = Math.max(0, RDServoPositions[1] - 0.15);
                    new MoveInDiffyServoPosition(1).run();
                    timer.schedule(new setIsInDiffyMoving(false), 0);
                }
            }

            if(leftBumper && !intakeManualMode && inPosition == 1 && !isInDiffyMoving && !oldLeftBumper) {
                litty.pipelineSwitch(1);//blue
                LLResult seeingStuff = litty.getLatestResult();
                LLResultTypes.ColorResult colorResult = null;
                if (seeingStuff != null && seeingStuff.isValid()) {
                    List<LLResultTypes.ColorResult> colorResults = seeingStuff.getColorResults();
                    if (colorResults != null && !colorResults.isEmpty()) {
                        colorResult = colorResults.get(0);
                    }
                }
                else if (seeingStuff == null || !seeingStuff.isValid() || colorResult == null) {
                    litty.pipelineSwitch(2);//yellow
                    seeingStuff = litty.getLatestResult();
                    if (seeingStuff != null && seeingStuff.isValid()) {
                        List<LLResultTypes.ColorResult> colorResults = seeingStuff.getColorResults();
                        colorResults = seeingStuff.getColorResults();
                        if (colorResults != null && !colorResults.isEmpty()) {
                            colorResult = colorResults.get(0);
                        }
                    }
                }
                if (colorResult != null) {
                    List<List<Double>> targetCorners = colorResult.getTargetCorners();
                    if (!targetCorners.isEmpty() && targetCorners.size() > 3) {
                        List<Double> c1 = targetCorners.get(0);
                        List<Double> c2 = targetCorners.get(1);
                        List<Double> c3 = targetCorners.get(2);
                        List<Double> c4 = targetCorners.get(3);
                        telemetry.addData("Corner 1 X", targetCorners.get(0).get(0));
                        telemetry.addData("Corner 1 Y", targetCorners.get(0).get(1));
                        telemetry.addData("Corner 2 X", targetCorners.get(1).get(0));
                        telemetry.addData("Corner 2 Y", targetCorners.get(1).get(1));
                        telemetry.addData("Corner 3 X", targetCorners.get(2).get(0));
                        telemetry.addData("Corner 3 Y", targetCorners.get(2).get(1));
                        telemetry.addData("Corner 4 X", targetCorners.get(3).get(0));
                        telemetry.addData("Corner 4 Y", targetCorners.get(3).get(1));
                        double c1c2XOffset = c1.get(0) - c2.get(0);
                        double c1c2YOffset = c1.get(1) - c2.get(1);
                        double c3c2XOffset = c3.get(0) - c2.get(0);
                        double c3c2YOffset = c3.get(1) - c2.get(1);
                        double c4c3XOffset = c4.get(0) - c3.get(0);
                        double c3c4YOffset = c3.get(1) - c4.get(1);
                        double c1c2D = Math.sqrt(Math.pow(c1c2XOffset, 2) + Math.pow(c1c2YOffset, 2));
                        double c2c3D = Math.sqrt(Math.pow(c3c2XOffset, 2) + Math.pow(c3c2YOffset, 2));
                        double Angleish = 0;
                        boolean rightSideUp = true;
                        boolean isHorizontal = true;


                        if (c1c2D > c2c3D) {
                            rightSideUp = true;
                        } else if (c1c2D <= c2c3D) {
                            rightSideUp = false;
                        }
                        if (rightSideUp) {
                            Angleish = Math.toDegrees(Math.atan2(c3c4YOffset, c4c3XOffset));
                            telemetry.addData("x:", c4c3XOffset);
                            telemetry.addData("y:", c3c4YOffset);
                        }
                        if (!rightSideUp) {
                            Angleish = Math.toDegrees(-Math.atan2(c3c2YOffset, c3c2XOffset));
                            telemetry.addData("x:", c3c2XOffset);
                            telemetry.addData("y:", c3c2YOffset);
                        }
                        if (Angleish > 90) {
                            Angleish = -180 + Angleish;
                        } else if (Angleish < -90) {
                            Angleish = 180 + Angleish;
                        }
                        telemetry.addData("Angle:", Angleish);
                        degrees = colorResult.getTargetYDegrees();

                        telemetry.addData("Angle Down", colorResult.getTargetYDegrees());
                        telemetry.addData("Angle Across", colorResult.getTargetXDegrees());
                        telemetry.addData("Whatever this is", colorResult.getTargetYPixels());
                        telemetry.addData("Whatever this is across", colorResult.getTargetXPixels());


                        if (Angleish > 5 && !isInDiffyMoving) {
                            if (RDServoPositions[1] + ((Angleish + 90) / 270) <= 0.75 && LDServoPositions[1] - ((Angleish + 90) / 270) >= 0.25) {//spin 180 so no suicide
                                new setIsInArmMoving(true).run();
                                new setIsInDiffyMoving(true).run();
                                LDServoPositions[1] = LDServoPositions[1] - ((90 + Angleish) / 270);
                                RDServoPositions[1] = RDServoPositions[1] + ((90 + Angleish) / 270);
                            } else {
                                new setIsInArmMoving(true).run();
                                new setIsInDiffyMoving(true).run();
                                LDServoPositions[1] = LDServoPositions[1] + ((90 - Angleish) / 270);
                                RDServoPositions[1] = RDServoPositions[1] - ((90 - Angleish) / 270);
                            }
                            new MoveInDiffyServoPosition(1).run();
                            timer.schedule(new setIsInDiffyMoving(false), 200);

                        } else if (Angleish < -5 && !isInDiffyMoving) {
                            if (RDServoPositions[1] - ((-Angleish + 90) / 270) >= 0.25 && LDServoPositions[1] + ((-Angleish + 90) / 270) <= 0.75) { //spin 180 so no suicide
                                new setIsInArmMoving(true).run();
                                new setIsInDiffyMoving(true).run();
                                LDServoPositions[1] = LDServoPositions[1] + ((-Angleish + 90) / 270);
                                RDServoPositions[1] = RDServoPositions[1] - ((-Angleish + 90) / 270);
                            } else {
                                new setIsInArmMoving(true).run();
                                new setIsInDiffyMoving(true).run();
                                LDServoPositions[1] = LDServoPositions[1] - ((90 - Angleish) / 270);
                                RDServoPositions[1] = RDServoPositions[1] + ((90 - Angleish) / 270);
                            }

                            new MoveInDiffyServoPosition(1).run();
                            timer.schedule(new setIsInDiffyMoving(false), 200);

                        }
                        //none of this works
                      /* new setIsInArmMoving(true);
                       timer.schedule(new IntakeExtension((int) ((colorResult.getTargetYDegrees()+6.5) * 10)), 0)*/
                       timer.schedule(new MoveInElbowServosPosition(0), 8 * DELAY_BETWEEN_MOVES);
                       new setInClawDelay(true);
                        timer.schedule(new CloseIn(), 0);
                       timer.schedule(new OpenIn(), 12 * DELAY_BETWEEN_MOVES);
                       timer.schedule(new MoveInDiffyServoPosition(2), 16 * DELAY_BETWEEN_MOVES);
                       timer.schedule(new MoveInElbowServosPosition(2), 16 * DELAY_BETWEEN_MOVES);
                       timer.schedule(new setInClawDelay(false), 18 * DELAY_BETWEEN_MOVES);
                       timer.schedule(new setIsInArmMoving(false), 18 * DELAY_BETWEEN_MOVES);
                       timer.schedule(new setIsInDiffyMoving(false), 18 * DELAY_BETWEEN_MOVES);
                       inPosition = 2;
                    }
                }
            }





            telemetry.addData("Intake Length:", InSlide.getCurrentPosition());
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("L Dif Pos:", + LDServoPositions[1]);
            telemetry.addData("R Dif Pos:", + RDServoPositions[1]);
            telemetry.addData("degrees", + degrees);
            //   telemetry.addData("INDEX", index % LEServoPositions.length);



            if(gamepad2.left_stick_button && !oldLeftStickButton){
                babyMode = !babyMode;
            }
            if(gamepad2.right_stick_button && !oldRightStickButton){
                intakeManualMode = !intakeManualMode;
            }
            telemetry.update();
            oldLeftStickButton = gamepad2.left_stick_button;
            oldRightStickButton = gamepad2.right_stick_button;
            oldCrossPressed = crossPressed;
            oldCirclePressed = circlePressed;
            oldSquarePressed = squarePressed;
            oldTrianglePressed = trianglePressed;
            oldUpDpadPressed = dpadUpPressed;
            oldDownDpadPressed = dpadDownPressed;
            oldLeftDpadPressed = dpadLeftPressed;
            oldRightDpadPressed = dpadRightPressed;
            oldLeftBumper = leftBumper;
            oldRightBumper = rightBumper;
            oldLeftTrigger = leftTrigger;
            oldRightTrigger = rightTrigger;




        }
    }
}

