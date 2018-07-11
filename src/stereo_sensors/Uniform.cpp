#include "Uniform.h"

Uniform::Uniform(const int & rows_,
                 const int & columns_,
                 const double & input_rows_,
                 const double & input_columns_,
                 const double & input_sensor_radius_) :
    rows(rows_),
    columns(columns_),
    output_row_center((rows-1)/2.0),
    output_column_center((columns-1)/2.0),
    input_rows(input_rows_),
    input_columns(input_columns_),
    input_row_center((input_rows-1)/2.0),
    input_column_center((input_columns-1)/2.0),
    input_sensor_radius(input_sensor_radius_),
    input_sensor_squared_radius(input_sensor_radius*input_sensor_radius),
    row_scale((rows-output_row_center)/input_sensor_radius_),
    column_scale((columns-output_column_center)/input_sensor_radius_)
{
    std::cout << "rows: " << rows << std::endl;
    std::cout << "columns: " << columns << std::endl;
    std::cout << "output_row_center: " << output_row_center << std::endl;
    std::cout << "output_column_center: " << output_column_center << std::endl;
    std::cout << "input_rows: " << input_rows << std::endl;
    std::cout << "input_columns: " << input_columns << std::endl;
    std::cout << "input_row_center: " << input_row_center << std::endl;
    std::cout << "input_column_center: " << input_column_center << std::endl;
    std::cout << "input_sensor_radius: " << input_sensor_radius << std::endl;
    std::cout << "row_scale: " << row_scale << std::endl;
    std::cout << "column_scale: " << column_scale << std::endl;
}

void Uniform::getInputSensorCoordinates(const double & output_row,
                                        const double & output_column,
                                        double & input_row,
                                        double & input_column)
{
    input_row=(output_row-output_row_center)/row_scale+input_row_center;
    input_column=(output_column-output_column_center)/column_scale+input_column_center;
}


void Uniform::getOutputSensorCoordinates(const double & input_row,
                                         const double & input_column,
                                         double & output_row,
                                         double & output_column)
{
    output_row=(input_row-input_row_center)*row_scale+output_row_center;
    output_column=(input_column-input_column_center)*column_scale+output_column_center;
}

bool Uniform::checkInsideSensor(const double & input_row,
                                const double & input_column)
{
    // Check if input coordinates are inside the output sensor
    if( std::pow(input_row-input_row_center,2)+std::pow(input_column-input_column_center,2) > input_sensor_squared_radius)
    {
        return false;
    }
    return true;
}
