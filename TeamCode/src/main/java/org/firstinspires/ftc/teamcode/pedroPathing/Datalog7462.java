package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Datalogger;

public class Datalog7462 {
    Datalog datalogAuto;
    Datalog getDatalogTeleop;
    private void logOneSample(Pose pose) {
        //datalogAuto.runTime.set(runtime.seconds());
        datalogAuto.xPose.set(pose.getX());
        datalogAuto.yPose.set(pose.getY());
        datalogAuto.heading.set(pose.getHeading());
        //datalogAuto.tx.set(limelight.getTx());
        //datalogAuto.ty.set(limelight.getTy());
        datalogAuto.writeLine();
    }
    public static class Datalog {
        /*
         * The underlying datalogger object - it cares only about an array of loggable fields
         */
        private final Datalogger datalogger;
        /*
         * These are all of the fields that we want in the datalog.
         * Note: Order here is NOT important. The order is important
         *       in the setFields() call below
         */

        /* Auto Fields */
        public Datalogger.GenericField runTime = new Datalogger.GenericField("runTime");
        public Datalogger.GenericField xPose   = new Datalogger.GenericField("X");
        public Datalogger.GenericField yPose   = new Datalogger.GenericField("Y");
        public Datalogger.GenericField heading = new Datalogger.GenericField("Heading");
//        public Datalogger.GenericField tx = new Datalogger.GenericField("tx");
//        public Datalogger.GenericField ty = new Datalogger.GenericField("ty");

        /* TeleOp Fields */

        public Datalog(String name) {
            datalogger = new Datalogger.Builder()
                    .setFilename(name)
                    .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)
                    /*
                     * Tell it about the fields we care to log.
                     * Note: Order *IS* important here! The order in which we list the
                     *       fields is the order in which they will appear in the log.
                     */
                    .setFields( runTime, xPose, yPose, heading)
                    .build();
        }

        // Tell the datalogger to gather the values of the fields
        // and write a new line in the log.
        public void writeLine() {
            datalogger.writeLine();
        }
    }
}
