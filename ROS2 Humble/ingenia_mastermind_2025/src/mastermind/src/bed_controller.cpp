/*
Archivo: bed_controller.cpp
Versión: V0.0
Autores: Miguel Ángel García de Vicente
*/


#include "mastermind/bed_controller.hpp"



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BedControllerNode>());
    rclcpp::shutdown();
    return 0;
}