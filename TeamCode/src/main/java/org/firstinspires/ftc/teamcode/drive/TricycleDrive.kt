package org.firstinspires.ftc.teamcode.drive

import com.acmerobotics.roadrunner.drive.Drive
import com.acmerobotics.roadrunner.drive.DriveSignal

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.kinematics.TankKinematics
import com.acmerobotics.roadrunner.localization.Localizer
import com.acmerobotics.roadrunner.util.Angle
import com.acmerobotics.roadrunner.util.epsilonEquals
import kotlin.math.PI
import kotlin.math.atan

/**
 * This class provides the basic functionality of a Tricycle drive using [TricycleKinematics].
 *
 * @param kV velocity feedforward
 * @param kA acceleration feedforward
 * @param kStatic additive constant feedforward
 * @param trackWidth lateral distance between pairs of wheels on different sides of the robot
 */
abstract class TricycleDrive constructor(
        private val kV: Double,
        private val kA: Double,
        private val kStatic: Double,
        private val trackWidth: Double,
) : Drive() {

    /**
     * Default localizer for tank drives based on the drive encoders and (optionally) a heading sensor.
     *
     * @param drive drive
     * @param useExternalHeading use external heading provided by an external sensor (e.g., IMU, gyroscope)
     */
    class TricycleLocalizer @JvmOverloads constructor(
            private val drive: TricycleDrive,
            private val useExternalHeading: Boolean = true
    ) : Localizer {
        private var _poseEstimate = Pose2d()
        override var poseEstimate: Pose2d
            get() = _poseEstimate
            set(value) {
                lastWheelPositions = emptyList()
                lastExtHeading = Double.NaN
                if (useExternalHeading) drive.externalHeading = value.heading
                _poseEstimate = value
            }
        override var poseVelocity: Pose2d? = null
            private set
        private var lastWheelPositions = emptyList<Double>()
        private var lastExtHeading = Double.NaN

        override fun update() {
            val wheelPositions = drive.getWheelPositions()
            val extHeading = if (useExternalHeading) drive.externalHeading else Double.NaN
            if (lastWheelPositions.isNotEmpty()) {
                val wheelDeltas = wheelPositions
                        .zip(lastWheelPositions)
                        .map { it.first - it.second }
                val robotPoseDelta = TricycleKinematics.wheelToRobotVelocities(wheelDeltas, drive.getSteeringAngle())

                val finalHeadingDelta = if (useExternalHeading) {
                    Angle.normDelta(extHeading - lastExtHeading)
                } else {
                    robotPoseDelta.heading
                }
                _poseEstimate = Kinematics.relativeOdometryUpdate(
                        _poseEstimate,
                        Pose2d(robotPoseDelta.vec(), finalHeadingDelta)
                )
            }

            val wheelVelocities = drive.getWheelVelocities()
            val extHeadingVel = drive.getExternalHeadingVelocity()
            if (wheelVelocities != null) {
                poseVelocity = TricycleKinematics.wheelToRobotVelocities(wheelVelocities, drive.getSteeringAngle())
                if (useExternalHeading && extHeadingVel != null) {
                    poseVelocity = Pose2d(poseVelocity!!.vec(), extHeadingVel)
                }
            }

            lastWheelPositions = wheelPositions
            lastExtHeading = extHeading
        }
    }

    override var localizer: Localizer = TricycleLocalizer(this)

    override fun setDriveSignal(driveSignal: DriveSignal) {
        val velocities = TricycleKinematics.robotToWheelVelocities(driveSignal.vel)
        val accelerations = TricycleKinematics.robotToWheelAccelerations(driveSignal.accel)
        val power = Kinematics.calculateMotorFeedforward(velocities, accelerations, kV, kA, kStatic)
        if(driveSignal.vel.heading != 0.0 || driveSignal.vel.x != 0.0){
            val turnAngle: Double = atan((-(driveSignal.vel.heading/driveSignal.vel.x)) * 7.700905)
            arcadeDrive(power, turnAngle)
        }
        else{
            arcadeDrive(power, 0.0)
        }
    }

    override fun setDrivePower(drivePower: Pose2d) {
        val signal : DriveSignal = DriveSignal(drivePower)
        setDriveSignal(signal)
    }

    /**
     * Uses arcadeDrive from HhsTricycleDriveBase to set the motor powers
     */
    abstract fun arcadeDrive(drivePower: Double, turnAngle: Double)

    /**
     * Gets the steering angle in radians from the front wheel based on the encoder count of the front wheel
     */
    abstract fun getSteeringAngle(): Double

    /**
     * Sets the following motor powers (normalized voltages). All arguments are on the interval `[-1.0, 1.0]`.
     */
    abstract fun setMotorPowers(left: Double, right: Double)

    /**
     * Returns the positions of the wheels in linear distance units. Positions should exactly match the ordering in
     * [setMotorPowers].
     */
    abstract fun getWheelPositions(): List<Double>

    /**
     * Returns the velocities of the wheels in linear distance units. Positions should exactly match the ordering in
     * [setMotorPowers].
     */
    open fun getWheelVelocities(): List<Double>? = null
}
