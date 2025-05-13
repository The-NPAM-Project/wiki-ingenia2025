/*
Archivo: master.cpp
Versión: V0.0
Autores: Miguel Ángel García de Vicente
*/


#include "mastermind/master.hpp"



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MasterNode>());
    rclcpp::shutdown();
    return 0;
}