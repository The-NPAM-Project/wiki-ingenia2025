/*
Archivo: calibration.hpp
Versión: V0.0
Autores: Miguel Ángel García de Vicente
*/

#include "rclcpp/rclcpp.hpp"


class CalibrationNode : public rclcpp::Node
{
    public:

        CalibrationNode() : Node("calibration") {
            ;
        }

};
