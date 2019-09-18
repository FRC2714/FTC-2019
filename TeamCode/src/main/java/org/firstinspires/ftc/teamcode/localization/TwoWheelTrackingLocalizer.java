package org.firstinspires.ftc.teamcode.localization;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;

import java.util.ArrayList;
import java.util.List;

public abstract class TwoWheelTrackingLocalizer {
    private Pose2d poseEstimate;
    private List<Double> lastWheelPositions;
    private double lastHeading;

    private DecompositionSolver forwardSolver;


    public TwoWheelTrackingLocalizer(List<Pose2d> wheelPoses) {
        lastWheelPositions = new ArrayList<>();
        lastHeading = Double.NaN;

        Array2DRowRealMatrix inverseMatrix = new Array2DRowRealMatrix(3,3);

        for (int i = 0; i < 2; i++) {
            Vector2d orientationVector = wheelPoses.get(i).headingVec();
            Vector2d positionVector = wheelPoses.get(i).vec();
            inverseMatrix.setEntry(i, 0, orientationVector.x);
            inverseMatrix.setEntry(i, 0, orientationVector.y);
            inverseMatrix.setEntry(i, 2, positionVector.x * orientationVector.y - positionVector.y * orientationVector.x);
        }

        inverseMatrix.setEntry(2, 2, 1.0);
        forwardSolver = new LUDecomposition(inverseMatrix).getSolver();
    }

    public void update() {
        
    }
}




