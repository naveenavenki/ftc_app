package org.firstinspires.ftc.teamcode;

import android.util.Log;

/**
 * Created by Krishna Saxena on 10/3/2017.
 */

public class BoKAutoPosition extends BoKAutoCommon
{
    private static final double TIMEOUT_RIGHT = 4;
    private static final double TIMEOUT_CENTER = 5;
    private static final double TIMEOUT_LEFT = 6;

    private static final double DISTANCE_TO_LEFT_COL = 26; // inches!!
    private static final double DISTANCE_TO_CENTER_COL = 33;
    private static final double DISTANCE_TO_RIGHT_COL = 40;

    // Constructor
    public BoKAutoPosition()
    {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_BLUE;
    }

    @Override
    public void runSoftware()
    {
        if(robot.isPositionTrackingEnabled()) {
            robot.getCurrentPosition();
        }

        double[] goToPosition = {0,5}; //(x,y)
        goToPosition(goToPosition, 0.3, .5);
        Log.v("BOK", "Stage 1 passed");
        robot.getCurrentPosition();
        opMode.sleep(1000);

        goToPosition[0] = 20;
        goToPosition[1] = 40;
        goToPosition(goToPosition, 0.3, .5);

        Log.v("BOK", "Stage 2 passed");
        robot.getCurrentPosition();
        opMode.sleep(1000);

        goToPosition[0] = 0;
        goToPosition[1] = 30;
        goToPosition(goToPosition, 0.3, .5);

        Log.v("BOK", "Stage 3 passed");

        robot.getCurrentPosition();
        opMode.sleep(1000);

        goToPosition[0] = 0;
        goToPosition[1] = 6;
        goToPosition(goToPosition, 0.3, .5);
        Log.v("BOK", "Stage 4 passed");
        opMode.sleep(10000);
    }
}
