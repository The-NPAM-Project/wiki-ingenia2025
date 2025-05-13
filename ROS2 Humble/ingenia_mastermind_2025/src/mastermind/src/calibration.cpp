/*
Archivo: calibration.cpp
Versión: V0.0
Autores: Miguel Ángel García de Vicente
*/


#include "mastermind/calibration.hpp"



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CalibrationNode>());
    rclcpp::shutdown();
    return 0;
}