package org.firstinspires.ftc.teamcode.drive

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.util.epsilonEquals

/**
 * Tank drive kinematic equations based upon the unicycle model. All wheel positions and velocities are given in
 * (left, right) tuples. Robot poses are specified in a coordinate system with positive x pointing forward, positive y
 * pointing left, and positive heading measured counter-clockwise from the x-axis.
 *
 * [This page](http://rossum.sourceforge.net/papers/DiffSteer/) gives a motivated derivation.
 */
object TricycleKinematics {

    /**
     * Computes the wheel velocities corresponding to [robotVel] given [trackWidth].
     *
     * @param robotVel velocity of the robot in its reference frame
     * @param trackWidth lateral distance between pairs of wheels on different sides of the robot
     */
    @JvmStatic
    fun robotToWheelVelocities(robotVel: Pose2d): Double {
        require((robotVel.y epsilonEquals 0.0)) { "Lateral (robot y) velocity must be zero for tank drives" }

        return robotVel.x
    }

    /**
     * Computes the wheel accelerations corresponding to [robotAccel] given [trackWidth].
     *
     * @param robotAccel velocity of the robot in its reference frame
     * @param trackWidth lateral distance between pairs of wheels on different sides of the robot
     */
    // follows from linearity of the derivative
    @JvmStatic
    fun robotToWheelAccelerations(robotAccel: Pose2d) =
        robotToWheelVelocities(robotAccel)

    /**
     * Computes the robot velocity corresponding to [wheelVelocities] and the given drive parameters.
     *
     * @param wheelVelocities wheel velocities (or wheel position deltas)
     * @param steeringAngle angle that the steering wheel is at
     */
    @JvmStatic
    fun wheelToRobotVelocities(wheelVelocities: List<Double>, steeringAngle: Double): Pose2d {
        val (left, right) = wheelVelocities
        return Pose2d(
                (left + right) / 2.0, 0.0,
                    ((left + right) / 2)/(7.75 * Math.tan(Math.PI/2 - Math.abs(Math.toRadians(steeringAngle)))))
    }
}
