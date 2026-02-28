#include "./procedures.h"

void run_procedures(){
    if(perform_procedure){

        // Do  accelerometer ellipsoid
        if(perform_procedure == 1 ){

            if(!calibrate_accelerometer_step_logged){
                float accelerometer_readings[3] = {0};
                
                mpu6050_get_accelerometer_readings_gravity_raw_old(accelerometer_readings);

                running_average_update(&average_accelerometer_x, accelerometer_readings[0]);
                running_average_update(&average_accelerometer_y, accelerometer_readings[1]);
                running_average_update(&average_accelerometer_z, accelerometer_readings[2]);

                calibrate_accelerometer_step_logged_amount++;
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // Disable the led

                if(calibrate_accelerometer_step_logged_amount >= calibrate_accelerometer_step_logged_amount_required){
                    calibrate_accelerometer_step_logged = 1;
                    if(calibrate_accelerometer_step > 0 && calibrate_accelerometer_step <= calibrate_accelerometer_ellipsoid_steps){
                        printf("STEP %d %d\n", calibrate_accelerometer_step, calibrate_accelerometer_step_logged_amount);
                        calibrate_accelerometer_values_x[calibrate_accelerometer_step-1] = running_average_get_average(&average_accelerometer_x);
                        calibrate_accelerometer_values_y[calibrate_accelerometer_step-1] = running_average_get_average(&average_accelerometer_y);
                        calibrate_accelerometer_values_z[calibrate_accelerometer_step-1] = running_average_get_average(&average_accelerometer_z);

                        
                        if(calibrate_accelerometer_step == 6){
                            printf("Perform ellipsoid fit on these values: in a .csv file\n");
                            printf("Acc X;Acc Y;Acc Z;\n");
                            printf("%f;%f;%f;\n", calibrate_accelerometer_values_x[0], calibrate_accelerometer_values_y[0], calibrate_accelerometer_values_z[0]);
                            printf("%f;%f;%f;\n", calibrate_accelerometer_values_x[1], calibrate_accelerometer_values_y[1], calibrate_accelerometer_values_z[1]);
                            printf("%f;%f;%f;\n", calibrate_accelerometer_values_x[2], calibrate_accelerometer_values_y[2], calibrate_accelerometer_values_z[2]);
                            printf("%f;%f;%f;\n", calibrate_accelerometer_values_x[3], calibrate_accelerometer_values_y[3], calibrate_accelerometer_values_z[3]);
                            printf("%f;%f;%f;\n", calibrate_accelerometer_values_x[4], calibrate_accelerometer_values_y[4], calibrate_accelerometer_values_z[4]);
                            printf("%f;%f;%f;\n", calibrate_accelerometer_values_x[5], calibrate_accelerometer_values_y[5], calibrate_accelerometer_values_z[5]);

                            printf("\nOr a tab delimited .txt\n");
                            printf("%f\t%f\t%f\t\n", calibrate_accelerometer_values_x[0], calibrate_accelerometer_values_y[0], calibrate_accelerometer_values_z[0]);
                            printf("%f\t%f\t%f\t\n", calibrate_accelerometer_values_x[1], calibrate_accelerometer_values_y[1], calibrate_accelerometer_values_z[1]);
                            printf("%f\t%f\t%f\t\n", calibrate_accelerometer_values_x[2], calibrate_accelerometer_values_y[2], calibrate_accelerometer_values_z[2]);
                            printf("%f\t%f\t%f\t\n", calibrate_accelerometer_values_x[3], calibrate_accelerometer_values_y[3], calibrate_accelerometer_values_z[3]);
                            printf("%f\t%f\t%f\t\n", calibrate_accelerometer_values_x[4], calibrate_accelerometer_values_y[4], calibrate_accelerometer_values_z[4]);
                            printf("%f\t%f\t%f\t\n", calibrate_accelerometer_values_x[5], calibrate_accelerometer_values_y[5], calibrate_accelerometer_values_z[5]);
                        }
                    }

                }
            }else{
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // Indicate that it is ready
            }

        }else if(perform_procedure == 2){

            if(!calibrate_accelerometer_step_logged){
                float accelerometer_readings[3] = {0};
                
                mpu6050_get_accelerometer_readings_gravity_no_level_old(accelerometer_readings);

                running_average_update(&average_accelerometer_x, accelerometer_readings[0]);
                running_average_update(&average_accelerometer_y, accelerometer_readings[1]);
                running_average_update(&average_accelerometer_z, accelerometer_readings[2]);

                calibrate_accelerometer_step_logged_amount++;
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // Disable the led

                if(calibrate_accelerometer_step_logged_amount >= calibrate_accelerometer_step_logged_amount_required){
                    calibrate_accelerometer_step_logged = 1;

                    if(calibrate_accelerometer_step > 0 && calibrate_accelerometer_step <= calibrate_accelerometer_level_steps){
                        printf("STEP %d %d\n", calibrate_accelerometer_step, calibrate_accelerometer_step_logged_amount);

                        calibrate_accelerometer_values_x[calibrate_accelerometer_step-1] = running_average_get_average(&average_accelerometer_x);
                        calibrate_accelerometer_values_y[calibrate_accelerometer_step-1] = running_average_get_average(&average_accelerometer_y);
                        calibrate_accelerometer_values_z[calibrate_accelerometer_step-1] = running_average_get_average(&average_accelerometer_z);


                        
                        if(calibrate_accelerometer_step == 4){
                            float final_average_accelerometer_x = (calibrate_accelerometer_values_x[0] + calibrate_accelerometer_values_x[1] + calibrate_accelerometer_values_x[2] + calibrate_accelerometer_values_x[3]) / 4.0f;
                            float final_average_accelerometer_y = (calibrate_accelerometer_values_y[0] + calibrate_accelerometer_values_y[1] + calibrate_accelerometer_values_y[2] + calibrate_accelerometer_values_y[3]) / 4.0f;
                            float final_average_accelerometer_z = (calibrate_accelerometer_values_z[0] + calibrate_accelerometer_values_z[1] + calibrate_accelerometer_values_z[2] + calibrate_accelerometer_values_z[3]) / 4.0f;

                            printf(
                                "ACCELEROMETER errors:  X,Y,Z  %f, %f, %f\n",
                                final_average_accelerometer_x,
                                final_average_accelerometer_y,
                                final_average_accelerometer_z
                            );

                        }
                    }
                }
            }else{
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // Indicate that it is ready
            }

        }else if(perform_procedure == 3){
            // Do the 4 yaw rotations thing for roll and pitch
            
            if(!calibrate_roll_pitch_step_logged){
                running_average_update(&average_roll, imu_orientation[0] - base_accelerometer_roll_offset);
                running_average_update(&average_pitch, imu_orientation[1] - base_accelerometer_pitch_offset);

                calibrate_roll_pitch_step_logged_amount++;
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // Disable the led

                if(calibrate_roll_pitch_step_logged_amount >= calibrate_roll_pitch_step_logged_amount_required){
                    calibrate_roll_pitch_step_logged = 1;

                    if(calibrate_roll_pitch_step > 0 && calibrate_roll_pitch_step <= calibrate_roll_pitch_steps){
                        printf("STEP %d %d\n", calibrate_roll_pitch_step, calibrate_roll_pitch_step_logged_amount);

                        calibrate_roll_pitch_values_roll[calibrate_roll_pitch_step-1] = running_average_get_average(&average_roll);
                        calibrate_roll_pitch_values_pitch[calibrate_roll_pitch_step-1] = running_average_get_average(&average_pitch);
                        
                        if(calibrate_roll_pitch_step == 4){
                            float final_average_roll = (calibrate_roll_pitch_values_roll[0] + calibrate_roll_pitch_values_roll[1] + calibrate_roll_pitch_values_roll[2] + calibrate_roll_pitch_values_roll[3]) / 4.0f;
                            float final_average_pitch = (calibrate_roll_pitch_values_pitch[0] + calibrate_roll_pitch_values_pitch[1] + calibrate_roll_pitch_values_pitch[2] + calibrate_roll_pitch_values_pitch[3]) / 4.0f;

                            printf(
                                "Accelerometer roll and pitch offsets:  Roll %f Pitch %f\n",
                                final_average_roll,
                                final_average_pitch
                            );

                        }
                    }
                }
            }else{
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // Indicate that it is ready
            }
        }
    }
}