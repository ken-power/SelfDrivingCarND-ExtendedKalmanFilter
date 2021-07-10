#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF()
{
    is_initialized_ = false;

    previous_timestamp_ = 0;

    // initializing matrices
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    H_laser_ = MatrixXd(2, 4);
    Hj_ = MatrixXd(3, 4);

    //measurement covariance matrix - laser
    R_laser_ << 0.0225, 0,
            0, 0.0225;

    //measurement covariance matrix - radar
    R_radar_ << 0.09, 0, 0,
            0, 0.0009, 0,
            0, 0, 0.09;

    //measurement matrix - laser
    H_laser_ << 1, 0, 0, 0,
            0, 1, 0, 0;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF()
= default;

void FusionEKF::ProcessMeasurement(const MeasurementPackage & measurement_pack)
{
    /**
     *  Initialization
     */
    if(!is_initialized_)
    {
        // first measurement
        // cout << "EKF: " << endl;
        ekf_.x_ = VectorXd(4);
        ekf_.x_ << 1, 1, 1, 1;

        //Create state covariance matrix
        ekf_.P_ = MatrixXd(4, 4);
        ekf_.P_ << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1000, 0,
                0, 0, 0, 1000;

        //initial transition matrix
        ekf_.F_ = MatrixXd(4, 4);
        ekf_.F_ << 1, 0, 1, 0,
                0, 1, 0, 1,
                0, 0, 1, 0,
                0, 0, 0, 1;

        if(measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
        {
            double rho = measurement_pack.raw_measurements_(0);
            double phi = measurement_pack.raw_measurements_(1);
            double rhodot = measurement_pack.raw_measurements_(2);

            // polar to cartesian - r * cos(angle) for x and r * sin(angle) for y
            ekf_.x_ << rho * cos(phi), rho * sin(phi), rhodot * cos(phi), rhodot * sin(phi);
        }
        else if(measurement_pack.sensor_type_ == MeasurementPackage::LASER)
        {
            // Initialize state.
            ekf_.x_ << measurement_pack.raw_measurements_(0), measurement_pack.raw_measurements_(1), 0.0, 0.0;
        }
        previous_timestamp_ = measurement_pack.timestamp_;

        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }

    /*****************************************************************************
     *  Prediction
     ****************************************************************************/

    //compute the time elapsed between the current and previous measurements
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;    //dt - expressed in seconds
    previous_timestamp_ = measurement_pack.timestamp_;

    float dt_2 = dt * dt;
    float dt_3 = dt_2 * dt;
    float dt_4 = dt_3 * dt;

    //Modify the F matrix so that the time is integrated
    ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;

    //process noise
    float noise_ax = 9;
    float noise_ay = 9;

    //set the process covariance matrix Q
    ekf_.Q_ = MatrixXd(4, 4);
    ekf_.Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
            0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
            dt_3 / 2 * noise_ax, 0, dt_2 * noise_ax, 0,
            0, dt_3 / 2 * noise_ay, 0, dt_2 * noise_ay;

    //predict
    ekf_.Predict();

    /*****************************************************************************
     * Update
     *
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
     ****************************************************************************/

    if(measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
        // Radar updates
        ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
        ekf_.R_ = R_radar_;
        ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    }
    else
    {
        // Laser updates
        ekf_.H_ = H_laser_;
        ekf_.R_ = R_laser_;
        ekf_.Update(measurement_pack.raw_measurements_);
    }

    // print the output
    std::cout << "x_ = " << ekf_.x_ << std::endl;
    std::cout << "P_ = " << ekf_.P_ << std::endl;
}