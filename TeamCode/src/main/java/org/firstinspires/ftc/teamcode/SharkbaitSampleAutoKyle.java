package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.constants.AutoServoConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import java.util.List;
import java.util.TimerTask;

@Autonomous(name="Samples AutoKyle")

public class SharkbaitSampleAutoKyle extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;


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

    private double[] ILEServoPositions = AutoServoConstants.ILEServoPositions;
    private double[] IREServoPositions = AutoServoConstants.IREServoPositions;
    private double[] OLEServoPositions = AutoServoConstants.OLEServoPositions;
    private double[] OREServoPositions = AutoServoConstants.OREServoPositions;
    private double[] LDServoPositions = AutoServoConstants.LDServoPositions;
    private double[] RDServoPositions = AutoServoConstants.RDServoPositions;
    private double[] OAServoPositions = AutoServoConstants.OAServoPositions;
    private double[] ICServoPositions = AutoServoConstants.ICServoPositions;
    private double[] OCServoPositions = AutoServoConstants.OCServoPositions;
    private double[] OKLServoPositions = AutoServoConstants.OKLServoPositions;
    private double[] OKRServoPositions = AutoServoConstants.OKRServoPositions;
    private double[] OWServoPositions = AutoServoConstants.OWServoPositions;


    private int[] OtSlideFPositions = AutoServoConstants.OtSlideFPositions;
    private int[] OtSlideMPositions = AutoServoConstants.OtSlideMPositions;
    private int[] OtSlideBPositions = AutoServoConstants.OtSlideBPositions;
    private int[] InSlidePositions = AutoServoConstants.InSlidePositions;
    java.util.Timer timer = new java.util.Timer();

    private boolean otSlidesMoving = false;
    private boolean inSlidesMoving = false;

    private int DELAY_BETWEEN_MOVES = 100;
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

    class OpenIn extends TimerTask {
        public OpenIn(){}
        public void run(){ InClawServo.setPosition(ICServoPositions[1]); }
    }
    class CloseIn extends TimerTask {
        public CloseIn(){}
        public void run(){ InClawServo.setPosition(ICServoPositions[0]); }
    }
    class Als extends TimerTask {
        int pipe = 2;
        public Als(int i){ pipe = i; }
        public void run(){
            double Angle = LimeLightPipeline.Align(pipe);
            if (Angle > 5) {
                if (RDServoPositions[1] + (Angle / 255) <= 0.75 && LDServoPositions[1] - (Angle / 255) >= 0.25) {//spin 180 so no suicide
                    LDServoPositions[1] = LDServoPositions[1] - (Angle / 270);
                    RDServoPositions[1] = RDServoPositions[1] + (Angle / 270);
                } else {
                    LDServoPositions[1] = LDServoPositions[1] + ((180 - Angle) / 270);
                    RDServoPositions[1] = RDServoPositions[1] - ((180 - Angle) / 270);
                }
                new MoveInDiffyServoPosition(1).run();

            } else if (Angle < -5) {
                Angle = -Angle;
                if (RDServoPositions[1] >= 0.25 && LDServoPositions[1] <= 0.75) { //spin 180 so no suicide
                    LDServoPositions[1] = LDServoPositions[1] + (Angle / 270);
                    RDServoPositions[1] = RDServoPositions[1] - (Angle / 270);
                } else {
                    LDServoPositions[1] = LDServoPositions[1] - ((180 - Angle) / 270);
                    RDServoPositions[1] = RDServoPositions[1] + ((180 - Angle) / 270);
                }
                new MoveInDiffyServoPosition(1).run();
            }
        }
    }
    class Align extends TimerTask {
        public Align(){}
        public void run(){
            litty.pipelineSwitch(0);//red
            LLResult seeingStuff = litty.getLatestResult();
            LLResultTypes.ColorResult colorResult = null;
            if (seeingStuff != null && seeingStuff.isValid()) {
                List<LLResultTypes.ColorResult> colorResults = seeingStuff.getColorResults();
                if (colorResults != null && !colorResults.isEmpty()) {
                    colorResult = colorResults.get(0);
                }
            }
            if (colorResult != null) {
                List<List<Double>> targetCorners = colorResult.getTargetCorners();
                if (!targetCorners.isEmpty() && targetCorners.size() > 3) {
                    List<Double> c1 = targetCorners.get(0);
                    List<Double> c2 = targetCorners.get(1);
                    List<Double> c3 = targetCorners.get(2);
                    List<Double> c4 = targetCorners.get(3);

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

                    if (Angleish > 5) {
                        if (RDServoPositions[1] + (Angleish / 255) <= 0.75 && LDServoPositions[1] - (Angleish / 255) >= 0.25) {//spin 180 so no suicide
                            LDServoPositions[1] = LDServoPositions[1] - (Angleish / 270);
                            RDServoPositions[1] = RDServoPositions[1] + (Angleish / 270);
                        } else {
                            LDServoPositions[1] = LDServoPositions[1] + ((180 - Angleish) / 270);
                            RDServoPositions[1] = RDServoPositions[1] - ((180 - Angleish) / 270);
                        }
                        new MoveInDiffyServoPosition(1).run();

                    } else if (Angleish < -5) {
                        if (RDServoPositions[1] >= 0.25 && LDServoPositions[1] <= 0.75) { //spin 180 so no suicide
                            LDServoPositions[1] = LDServoPositions[1] + (-Angleish / 270);
                            RDServoPositions[1] = RDServoPositions[1] - (-Angleish / 270);
                        } else {
                            LDServoPositions[1] = LDServoPositions[1] - ((180 + Angleish) / 270);
                            RDServoPositions[1] = RDServoPositions[1] + ((180 + Angleish) / 270);
                        }
                        new MoveInDiffyServoPosition(1).run();
                    }
                }
            }
        }
    }

    private final Pose startPose = new Pose(7, 103, Math.toRadians(0));  // Starting position

    private final Pose scorePose1 = new Pose(18, 121, Math.toRadians(-45)); // Scoring position 1
    private final Pose scorePose2 = new Pose(18, 121, Math.toRadians(-45)); // Scoring position 2
    private final Pose scorePose3 = new Pose(18, 121, Math.toRadians(-45)); // Scoring position 3
    private final Pose scorePose4 = new Pose(18, 121, Math.toRadians(-45)); // Scoring position 3
    private final Pose scorePose5 = new Pose(18, 121, Math.toRadians(-45)); // Scoring position 3

    private final Pose pickupSample1Pose = new Pose(18.5, 121.5, Math.toRadians(9));
    private final Pose pickupSample2Pose = new Pose(18.5, 121.5, Math.toRadians(24));
    private final Pose pickupSample3Pose = new Pose(18.5, 121.5, Math.toRadians(40));


    private final Pose parkPose = new Pose(61, 96, Math.toRadians(-90));    // Parking position

    private Path scorePreload, park;
    private PathChain pickupSample1, scoreSample1, pickupSample2, scoreSample2, pickupSample3, scoreSample3;
    private PathChain doEverything;
    public void buildPaths() {
//worth a shot        doEverything = follower.pathBuilder().addPath(new Path(new BezierLine(new Point(startPose), new Point(scorePose1)))).setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading()).addPath(new BezierCurve(new Point(scorePose1), new Point(15,63, Point.CARTESIAN), new Point(37,30, Point.CARTESIAN), new Point(pickupPreload2Pose))).setLinearHeadingInterpolation(scorePose1.getHeading(), pickupPreload2Pose.getHeading()).build();
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose1)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading());

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        pickupSample1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose1), new Point(pickupSample1Pose)))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), pickupSample1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scoreSample1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickupSample1Pose), new Point(scorePose2)))
                .setLinearHeadingInterpolation(pickupSample1Pose.getHeading(), scorePose2.getHeading())
                .build();
        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        pickupSample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose2), new Point(pickupSample2Pose)))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), pickupSample2Pose.getHeading())
                .build();
        follower.holdPoint(new Point(scorePose1), scorePose1.getHeading());

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scoreSample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickupSample2Pose), new Point(scorePose3)))
                .setLinearHeadingInterpolation(pickupSample2Pose.getHeading(), scorePose3.getHeading())
                .build();

        pickupSample3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose3), new Point(pickupSample3Pose)))
                .setLinearHeadingInterpolation(scorePose3.getHeading(), pickupSample3Pose.getHeading())
                .build();

        scoreSample3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickupSample3Pose), new Point(scorePose4)))
                .setLinearHeadingInterpolation(pickupSample3Pose.getHeading(), scorePose4.getHeading())
                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = new Path(new BezierCurve(new Point(scorePose4),
                new Point(61, 128, Point.CARTESIAN),
                /* Control Point */ new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose4.getHeading(), parkPose.getHeading());
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                timer.schedule(new InSlidePosition(0, 1), 0 * DELAY_BETWEEN_MOVES);
                timer.schedule(new OtSlidesPosition(1, 1, 1, 1), 0 * DELAY_BETWEEN_MOVES);
                follower.followPath(scorePreload, true);

                if (InSlide.getCurrentPosition() < 10 && OtSlideB.getCurrentPosition() < -2760)  setPathState(1);
                break;
            case 1:
                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end
                     point while we are grabbing the sample */
//                    follower.holdPoint(new Point(pickupSample1Pose), pickupSample1Pose.getHeading());
                    timer.schedule(new InSlidePosition(3, 1), 0 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new OtSlidesPosition(0, 0, 0, 1), 0 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new OpenIn(), 0 * DELAY_BETWEEN_MOVES);

                    follower.followPath(pickupSample1);
                    if (InSlide.getCurrentPosition() > 1190 && OtSlideB.getCurrentPosition() > -10) setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                    timer.schedule(new Als(2), 0 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new CloseIn(), 2 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new MoveInElbowServosPosition(0), 2 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new InSlidePosition(0, 1), 4 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new OtSlidesPosition(1, 1, 1, 1), 0 * DELAY_BETWEEN_MOVES);
                    follower.followPath(scoreSample2);
                    if (InSlide.getCurrentPosition() < 10 && OtSlideB.getCurrentPosition() < -2760) setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if( !follower.isBusy()) {
                    /* Grab Sample */
                    timer.schedule(new InSlidePosition(3, 1), 0 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new OtSlidesPosition(0, 0, 0, 1), 0 * DELAY_BETWEEN_MOVES);
                    follower.followPath(pickupSample2);
                    if (InSlide.getCurrentPosition() > 1190 && OtSlideB.getCurrentPosition() < -10) setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    timer.schedule(new Als(2), 0 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new CloseIn(), 2 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new MoveInElbowServosPosition(0), 2 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new InSlidePosition(0, 1), 4 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new OtSlidesPosition(1, 1, 1, 1), 0 * DELAY_BETWEEN_MOVES);
                    if (InSlide.getCurrentPosition() < 10 && OtSlideB.getCurrentPosition() < -2760) follower.followPath(scoreSample2);

                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    timer.schedule(new InSlidePosition(3, 1), 0 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new OtSlidesPosition(0, 0, 0, 1), 0 * DELAY_BETWEEN_MOVES);
                    if (InSlide.getCurrentPosition() > 1190 && OtSlideB.getCurrentPosition() < -10) follower.followPath(pickupSample3);

                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    timer.schedule(new Als(2), 0 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new CloseIn(), 2 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new MoveInElbowServosPosition(0), 2 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new OtSlidesPosition(1, 1, 1, 1), 0 * DELAY_BETWEEN_MOVES);
                    if (OtSlideB.getCurrentPosition() < -2760) follower.followPath(scoreSample3);

                    setPathState(15);
                }
                break;
            case 15:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    timer.schedule(new InSlidePosition(0, 0), 0 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new OtSlidesPosition(0, 0, 0, 0), 0 * DELAY_BETWEEN_MOVES);
                    follower.followPath(park,true);
                    if (InSlide.getCurrentPosition() < 10 && OtSlideB.getCurrentPosition() > -10) setPathState(16);
                }
                break;
            case 16:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Level 1 Ascent */

                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/

    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    public void init() {


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


        FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        InSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        OtSlideF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        OtSlideM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        OtSlideB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FLMotor.setDirection(DcMotor.Direction.REVERSE);
        FRMotor.setDirection(DcMotor.Direction.FORWARD);
        BLMotor.setDirection(DcMotor.Direction.REVERSE);
        BRMotor.setDirection(DcMotor.Direction.FORWARD);


        InSlide.setDirection(DcMotor.Direction.FORWARD);
        OtSlideF.setDirection(DcMotor.Direction.REVERSE);
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

        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LimeLightPipeline.makeLight(hardwareMap, "litty");
        litty = hardwareMap.get(Limelight3A.class, "litty");
        litty.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        telemetry.setMsTransmissionInterval(11);
        litty.pipelineSwitch(2);
        litty.start();


        RDServoPositions[1] = 0.5;
        LDServoPositions[1] = 0.5;


        InLDiffyServo.setPosition(LDServoPositions[3]);//3 no auto
        InRDiffyServo.setPosition(RDServoPositions[3]);//3 no auto
        InClawServo.setPosition(ICServoPositions[1]);
        OtClawServo.setPosition(OCServoPositions[1]);
        OtWristServo.setPosition(OWServoPositions[0]);
        OtLinkageLServo.setPosition(OKLServoPositions[0]);
        OtLinkageRServo.setPosition(OKRServoPositions[0]);
        InLeftElbowServo.setPosition(ILEServoPositions[3]);//3 no auto
        InRightElbowServo.setPosition(IREServoPositions[3]);//3 no auto
        OtLeftElbowServo.setPosition(OLEServoPositions[0]);
        OtRightElbowServo.setPosition(OREServoPositions[0]);
        OtCoaxialServo.setPosition(OAServoPositions[1]);




        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/

    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/

    public void stop() {
    }
}

