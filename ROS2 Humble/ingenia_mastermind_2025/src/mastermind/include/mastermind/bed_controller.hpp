/*
Archivo: bed_controller.hpp
Versión: V0.0
Autores: Miguel Ángel García de Vicente
*/

#include "rclcpp/rclcpp.hpp"


class BedControllerNode : public rclcpp::Node
{
    public:

        BedControllerNode() : Node("bed_controller") {
            ;
        }

};
