#ifndef UNIFORM_H
#define UNIFORM_H
#include <cmath>
#include <iostream>

class Uniform
{

public:
    Uniform(const int & rows_,
            const int & columns_,
            const double & input_rows_=1.0,
            const double & input_columns_=1.0,
            const double & input_sensor_radius_=1000000);
protected:
    int rows; // number of rows
    int columns; // number of columns

    double output_row_center;
    double output_column_center;

    double input_rows;
    double input_columns;

    double row_scale;
    double column_scale;

    double input_row_center;
    double input_column_center;

    double input_sensor_radius; // CIRCULAR SENSOR
    double input_sensor_squared_radius; // CIRCULAR SENSOR

    void getInputSensorCoordinates(const double & output_row,
                                   const double & output_column,
                                   double & input_row,
                                   double & input_column);

    void getOutputSensorCoordinates(const double & input_row,
                                    const double & input_column,
                                    double & output_row,
                                    double & output_column);

    bool checkInsideSensor(const double & input_row,
                           const double & input_column);
};

#endif // UNIFORM_H
