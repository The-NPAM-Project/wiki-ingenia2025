#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/msg/string.hpp>
#include <ur_msgs/IOStates.h> // Para leer E/S del ur (deberia poder leer el laser desde el PLC)

#include <cmath>
#include <array>
#include <new>

#include <thread>
#include <atomic>

#define SLEEPTIME 3 // Tiempo de espera entre movimientos

#define LIMSUPMM 370 // Limite superior laser en milimetros
#define LIMINFMM 330 // Limite superior laser en milimetros

#define LIMSUPLSR 0.02  // Limite superior laser en unidades del sensor
#define LIMINFLSR 0.004 // Limite superior laser en unidades del sensor

using namespace std;

//typedef uint8_t bool;

// Corrección de coordenadas
double correccionX = 0;
double correccionY = 0;
double correccionZ = 0;

// Variables para los threads
double limA = 0.015;
std::atomic<bool> threadRunning = true;

// Variable
ur_msgs::IOStates io_states;

// Estructura para representar una pose (posición y orientación)
struct Pose {
  double x, y, z, rx, ry, rz;
};

// Función para sumar poses
Pose pose_add(const Pose& p1, const Pose& p2) {
  return {p1.x + p2.x, p1.y + p2.y, p1.z + p2.z,
    p1.rx + p2.rx, p1.ry + p2.ry, p1.rz + p2.rz};
}

// Función para restar poses 
Pose pose_sub(const Pose& p1, const Pose& p2) {
  return {p1.x - p2.x, p1.y - p2.y, p1.z - p2.z,
    p1.rx - p2.rx, p1.ry - p2.ry, p1.rz - p2.rz};
}

// Multiplica una matriz 3x3 por un vector
array<double, 3> multiply_matrix_vector(double** matrix, double* vector) {
  array<double, 3> mult = {
    matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2],
    matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2],
    matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2]
  };

  return mult;
}

// Genera una matriz de rotación 3x3 a partir de ángulos en radianes
double** rotation_matrix(double rx, double ry, double rz) {
  double cx = cos(rx), sx = sin(rx);
  double cy = cos(ry), sy = sin(ry);
  double cz = cos(rz), sz = sin(rz);

  double** rot = new double*[3];
  rot[0] = new double[3];
  rot[0][0] = cy*cz;
  rot[0][1] = -cy*sz;
  rot[0][2] = sy;
  
  rot[1] = new double[3];
  rot[1][0] = sx*sy*cz + cx*sz;
  rot[1][1] = -sx*sy*sz + cx*cz;
  rot[1][2] = -sx*cy;
  
  rot[2] = new double[3];
  rot[2][0] = -cx*sy*cz + sx*sz;
  rot[2][1] = cx*sy*sz + sx*cz;
  rot[2][2] = cx*cy;

  return rot;
}

// Función para transformar poses considerando la orientación (pose_trans de URScript)
Pose pose_trans(const Pose& base, const Pose& offset) {
  // Matriz de rotación
  double** R = rotation_matrix(base.rx, base.ry, base.rz);
  double offset_pos[3] = {offset.x, offset.y, offset.z};
  auto transformed_offset = multiply_matrix_vector(R, offset_pos);

  return {base.x + transformed_offset[0],
          base.y + transformed_offset[1],
          base.z + transformed_offset[2],
          base.rx + offset.rx,
          base.ry + offset.ry,
          base.rz + offset.rz};
}

// Pausa la ejecución durante un tiempo
void sleep_time(int t){
  rclcpp::sleep_for(std::chrono::seconds(t));
}

// Define el punto de referencia de la herramienta
void set_tcp(const Pose& tcp){ // ----------------------------------------- ! Función sin acabar
    // Transformacion en tiempo de ejecución y establecer el utlimo TCP al URDF
}

// Mueve la herramienta del brazo a la posicción idicada
void movel(const Pose& location){ // ----------------------------------------- ! Función sin acabar
  ; // Placeholder para el movel.
}

// Encargado de leer la señal del laser
double analog_read(int n) { // ----------------------------------------- ! Función deshabilitada temporalmente
  return io_states.analog_out_states[n].state; // Se lee el valor asignado en el pin n del UR
}

// Devuelve el valor de las articulaciones del robot
vector<double> get_actual_joint_positions(moveit::planning_interface::MoveGroupInterface& mgi) {
  return mgi.getCurrentJointValues(); // Otras alternativas pueden ser usar .getCurrentPosition() o leer la posición que devuelve el superlogger
}

// Devuelve el pose de la herramienta
Pose get_forward_kin(vector<double> joints) { // ----------------------------------------- ! Función sin acabar
  return {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
}

// Genera un mensaje que puede ser o no bloqueante
void message(const rclcpp::Logger &logger, const string &msg, bool blocking = false) {
  RCLCPP_INFO(logger, "%s", msg.c_str());

  if (blocking) {
    RCLCPP_WARN(logger, "Bloqueo activo: espera la confirmación del usuario...");
    while (std::getchar() != '\n');  // Espera hasta que el usuario presione Enter
  }
}

// Conversor de miliamperios que da el laser a milimetros
inline double laser2meters(double dist) {
  return ((LIMSUPMM - LIMINFMM) / (LIMSUPLSR - LIMINFLSR) * (dist - LIMINFLSR) + LIMINFMM) / 1000.0;
}

// Normaliza un vector de 3 dimensiones
inline void normalize(vector<double>& n) {
  double norm = sqrt(n[0]*n[0] + n[1]*n[1] + n[2]*n[2]);
  n[0] /= norm;
  n[1] /= norm;
  n[2] /= norm;
}

// Hilo para mover el láser a una posición final
void utillaje_i(const Pose &cubo, const Pose &pos_fin, const Pose &SR) {
  threadRunning = true;
  while (threadRunning.load()) { // Load es una función de atomic que asegura que la variable está en tiempo real.
    set_tcp(pose_trans(cubo,pos_fin)); // Ajustar TCP
    movel(SR);  // Ejecutar movimiento
  }
}

// Función prinicpal con la que se calibra el UR
bool calibration(rclcpp::Node::SharedPtr node, 
                  moveit::planning_interface::MoveGroupInterface& move_group_interface, 
                  rclcpp::Logger logger) {

  message(logger, "En espera para iniciar la calibración.", true);

  message(logger, "Iniciando paso 1...");

  // Sistema de referencia del laser
  Pose SRlaser_PLA = {0.10338, -0.95558, 0.004, -0.0054, -0.0053, 0.0116};

  Pose cubo1 = {-0.01366, 0.00366, 0.133, -0.91423, -1.58349, 1.58345};

  cubo1 = pose_add(Pose{0, 0, 0, 0, 0, -0.7853982}, cubo1);
  cubo1 = pose_trans(cubo1, Pose{0.0110, -0.0270, -0.0038, 0, 0, 0});
  set_tcp(cubo1);

  // Definición de puntos locales de referencia
  Pose p01_local = {0, 0, 0, 0, 0, 0};
  Pose p02_local = {-0.018, -0.018, 0, 0, 0, 0};

  // Mueve el TCP a la posición inicial y luego mueve el láser a SRlaser_PLA
  set_tcp(pose_trans(cubo1, p01_local));
  movel(SRlaser_PLA);//movel(SRlaser_PLA, a = 1, v = 0.02);
  sleep_time(SLEEPTIME);

  message(logger, "Cama colocada aproximadamente.", true);

  // Creación de nuevos TCPs locales en función de p01_local
  Pose p1_local = pose_add(p01_local, Pose{-0.005, -0.005, 0, 0, 0, 0});
  Pose p2_local = pose_add(p01_local, Pose{-0.015, -0.005, 0, 0, 0, 0});
  Pose p3_local = pose_add(p01_local, Pose{-0.005, -0.075, 0, 0, 0, 0});

  // Medición de distancias en estos puntos para calcular la inclinación del plano
  set_tcp(pose_trans(cubo1, p1_local));
  movel(SRlaser_PLA);
  sleep_time(SLEEPTIME);
  double d1 = analog_read(0);

  set_tcp(pose_trans(cubo1, p2_local));
  movel(SRlaser_PLA);
  sleep_time(SLEEPTIME);
  double d2 = analog_read(0);

  set_tcp(pose_trans(cubo1, p3_local));
  movel(SRlaser_PLA);
  sleep_time(SLEEPTIME);
  double d3 = analog_read(0);

  // Conversión de las distancias medidas a metros
  d1 = laser2meters(d1);
  d2 = laser2meters(d2);
  d3 = laser2meters(d3);

  // Cálculo de la inclinación del plano utilizando las distancias medidas
  double dzx = ((p2_local.y - p1_local.y) * (d3 - d1) - (p3_local.y - p1_local.y) * (d2 - d1)) / ((p2_local.x - p1_local.x) * (p3_local.y - p1_local.y) - (p3_local.x - p1_local.x) * (p2_local.y - p1_local.y));
  double dzy = ((p3_local.x - p1_local.x) * (d2 - d1) - (p2_local.x - p1_local.x) * (d3 - d1)) / ((p2_local.x - p1_local.x) * (p3_local.y - p1_local.y) - (p3_local.x - p1_local.x) * (p2_local.y - p1_local.y));
  double normdz = sqrt(dzx*dzx + dzy*dzy);

  // Vector normal al plano
  vector<double> g = {dzx / normdz, dzy / normdz, 0.0};

  // Vector m utilizado para el cálculo del plano
  vector<double> m = {dzx, dzy, dzx*dzx + dzy*dzy};
  normalize(m);

  // Vector n utilizado para el cálculo del plano
  vector<double> n = {(p2_local.y - p1_local.y) * (p3_local.z - p1_local.z) - (p3_local.y - p1_local.y) * (p2_local.z - p1_local.z), (p2_local.z - p1_local.z) * (p3_local.x - p1_local.x) - (p3_local.z - p1_local.z) * (p2_local.x - p1_local.x), (p2_local.x - p1_local.x) * (p3_local.y - p1_local.y) - (p3_local.x - p1_local.x) * (p2_local.y - p1_local.y)};
  normalize(n);

  // Cálculo del vector ur1
  vector<double> ur1xyz = {m[1] * n[2] - n[1] * m[2], m[2] * n[0] - n[2] * m[0], m[0] * n[1] - n[0] * m[1]};
  normalize(ur1xyz);
  Pose ur1 = {0, 0, 0, ur1xyz[0], ur1xyz[1], ur1xyz[2]};

  // Cálculo del ángulo alpha1
  double alpha1 = acos(g[0]*m[0] + g[1]*m[1] + g[2]*m[2]);
  Pose r1 = {0, 0, 0, alpha1*ur1.rx, alpha1*ur1.ry, alpha1*ur1.rz};

  // Aplicación del paso 1
  Pose cubo2 = pose_add(cubo1, r1);
  set_tcp(pose_trans(cubo2, p01_local));
  movel(SRlaser_PLA);
  message(logger, "Check cubo2", true);
  double distlaser = analog_read(0);
  distlaser = laser2meters(distlaser);
  double correccion = -(350 - distlaser) / 1000.0;

  // Ajuste de la posición del láser
  Pose SRlaser_PLA2 = SRlaser_PLA;
  SRlaser_PLA2.z = SRlaser_PLA2.z + correccion;
  set_tcp(pose_trans(cubo2, p01_local));
  movel(SRlaser_PLA2);
  message(logger, "Check cubo2 corregido", true);

  // PASO 2
  // Muestra un mensaje emergente indicando el inicio del paso 2

  message(logger, "Iniciando paso 2...");

  //v12 = vector de la diagonal del cubo en coordenadas locales
  Pose v12 = {p02_local.x - p01_local.x, p02_local.y - p01_local.y, p02_local.z - p01_local.z, 0, 0, 0};
  
  // amedio y bmedio como puntos en la diagonal para hacer el barrido paramétrico
  Pose amedio = pose_add(p01_local, Pose{v12.x/16, v12.y/16, 0, 0, 0, 0});
  Pose bmedio = pose_add(p01_local, Pose{6*v12.x/16, 6*v12.y/16, 0, 0, 0, 0});

  // Mueve el TCP a la posición amedio y luego mueve el láser a SRlaser_PLA2
  set_tcp(pose_trans(cubo2, amedio));
  movel(SRlaser_PLA2);

  // Matemáticas para puntos de inicio y fin del barrido
  Pose v12rot = {-v12.y / 2, v12.x / 2, 0, 0, 0, 0};

  Pose  a_inic_xy = {amedio.x + 1.5 * v12rot.x, amedio.y + 1.5 * v12rot.y, 0, 0, 0, 0};
  Pose  a_fin_xy  = {amedio.x - 1.5 * v12rot.x, amedio.y - 1.5 * v12rot.y, 0, 0, 0, 0};
  Pose  b_inic_xy = {bmedio.x + 1.6 * v12rot.x, bmedio.y + 1.6 * v12rot.y, 0, 0, 0, 0};
  Pose  b_fin_xy  = {bmedio.x - 1.6 * v12rot.x, bmedio.y - 1.6 * v12rot.y, 0, 0, 0, 0};

  Pose a_inic = amedio;
  a_inic.x = a_inic_xy.x;
  a_inic.y = a_inic_xy.y;

  Pose  a_fin = amedio;
  a_fin.x = a_fin_xy.x;
  a_fin.y = a_fin_xy.y;

  Pose  b_inic = bmedio;
  b_inic.x = b_inic_xy.x;
  b_inic.y = b_inic_xy.y;

  Pose  b_fin = bmedio;
  b_fin.x = b_fin_xy.x;
  b_fin.y = b_fin_xy.y;

  // Iniciar los valores p_cubo
  Pose p4_cubo = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  
  Pose p5_cubo = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  Pose p6_cubo = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  Pose p7_cubo = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  

  // Mueve el TCP a la posición a_inic y luego mueve el láser a SRlaser_PLA2
  set_tcp(pose_trans(cubo2, a_inic));
  movel(SRlaser_PLA2);

  // Si el laser no está sobre la cama, se empieza a desplazar hacia la posición final
  if (analog_read(0) > limA) {
    thread laser_thread(utillaje_i, cubo2, a_fin, SRlaser_PLA2);
    while (analog_read(0) > limA) { // Se desplaza hasta que el laser está sobre la cama
      sleep_time(0.5); // En el codigo original hay un sync() que equivale a un pequeño delay
    }
    threadRunning = false;
    laser_thread.join(); // Espera a que el bucle del thread finalice correctamente.
    sleep_time(SLEEPTIME);
    vector<double> p4_joint = get_actual_joint_positions(move_group_interface);
    set_tcp(cubo2);
    Pose p4_flange = get_forward_kin(p4_joint);
    p4_cubo = pose_sub(SRlaser_PLA2, p4_flange);
  }

  set_tcp(pose_trans(cubo2, amedio));
  movel(SRlaser_PLA2);

  // Mientras que el laser esté sobre la cama, se cumple la condición
  if (analog_read(0) < limA) {
    thread laser_thread(utillaje_i, cubo2, a_fin, SRlaser_PLA2);
    while (analog_read(0) < limA) { // Se desplaza hasta que el laser deje de estar sobre la cama
      sleep_time(0.5); // En el codigo original hay un sync() que equivale a un pequeño delay
    }
    threadRunning = false;
    laser_thread.join(); // Espera a que el bucle del thread finalice correctamente.
    sleep_time(SLEEPTIME);
    vector<double> p5_joint = get_actual_joint_positions(move_group_interface);
    set_tcp(cubo2);
    Pose p5_flange = get_forward_kin(p5_joint);
    p5_cubo = pose_sub(SRlaser_PLA2, p5_flange);
  }

  // Mueve el TCP a la posición b_inic y luego mueve el láser a SRlaser_PLA2
  set_tcp(pose_trans(cubo2, b_inic));
  movel(SRlaser_PLA2);

  // Mismo proceso que en el caso de los puntos A pero con los B

  if (analog_read(0) > limA) {
    thread laser_thread(utillaje_i, cubo2, b_fin, SRlaser_PLA2);
    while (analog_read(0) > limA) {
      sleep_time(0.5); // En el codigo original hay un sync() que equivale a un pequeño delay
    }
    threadRunning = false;
    laser_thread.join(); // Espera a que el bucle del thread finalice correctamente.
    sleep_time(SLEEPTIME);
    vector<double> p6_joint = get_actual_joint_positions(move_group_interface);
    set_tcp(cubo2);
    Pose p6_flange = get_forward_kin(p6_joint);
    p6_cubo = pose_sub(SRlaser_PLA2, p6_flange);
  }

  // Mueve el TCP a la posición bmedio y luego mueve el láser a SRlaser_PLA2
  set_tcp(pose_trans(cubo2, bmedio));
  movel(SRlaser_PLA2);

  // Si la entrada analógica es menor que el límite, inicia el hilo utillaje2_i
  if (analog_read(0) < limA) {
    thread laser_thread(utillaje_i, cubo2, bmedio, SRlaser_PLA2);
    while (analog_read(0) < limA) {
      sleep_time(0.5); // En el codigo original hay un sync() que equivale a un pequeño delay
    }
    threadRunning = false;
    laser_thread.join(); // Espera a que el bucle del thread finalice correctamente.
    sleep_time(SLEEPTIME);
    vector<double> p7_joint = get_actual_joint_positions(move_group_interface);
    set_tcp(cubo2);
    Pose p7_flange = get_forward_kin(p7_joint);
    p7_cubo = pose_sub(SRlaser_PLA2, p7_flange);
  }

  // Asignación de las posiciones p4, p5, p6 y p7
  Pose p4 = p4_cubo;
  Pose p5 = p5_cubo;
  Pose p6 = p6_cubo;
  Pose p7 = p7_cubo;

  // Cálculo del determinante y los parámetros t1 y t2
  double det = ((p4.x - p6.x) * (p5.y - p7.y) - (p4.y - p6.y) * (p5.x - p7.x));
  double t1  = ((p5.x - p4.x) * (p5.y - p7.y) - (p5.y - p4.y) * (p5.x - p7.x)) / det;
  //double t2  = ((p4.x - p6.x) * (p5.y - p4.y) - (p4.y - p6.y) * (p5.x - p4.x)) / det;


  // Cálculo del origen
  Pose origen = pose_sub(p4, p6);
  origen.x = origen.x * t1;
  origen.y = origen.y * t1;
  origen.z = origen.z * t1;
  origen = pose_add(p4, origen);
  Pose Torigen = Pose{origen.x - correccionX, origen.y - correccionY, 0 - correccionZ, 0, 0, 0};

  // Cálculo del eje x
  Pose eje_x = pose_sub(p5, p7);
  vector<double> eje_x2 = {eje_x.x, eje_x.y, eje_x.z};
  normalize(eje_x2);

  // Cálculo del ángulo alpha2
  double alpha2 = acos(eje_x2[0]);

  // Cálculo del vector y_gorro
  Pose y_gorro = pose_sub(p4, p6);
  double y0 = y_gorro.x;
  double y1 = y_gorro.y; 
  double y2 = y_gorro.z;

  // Cálculo del vector ur2
  vector<double> ur2 = {0, 0, 0};
  ur2[0] = (-1) * y2 * eje_x2[1] + y1 * eje_x2[2];
  ur2[1] = y2 * eje_x2[0] - y0 * eje_x2[2];
  ur2[2] = (-1) * y1 * eje_x2[0] + y0 * eje_x2[1];
  normalize(ur2);

  // Cálculo de la rotación r2
  Pose r2 = {0, 0, 0, 0, 0, alpha2};
  //# global r2=p[0,0,0,alpha2*ur2[0],alpha2*ur2[1],alpha2*ur2[2]]

  // Aplicación de la transformación al cubo2 para obtener cubo3 y cubo4
  Pose cubo3 = pose_trans(cubo2, Torigen);
  Pose cubo4 = pose_trans(cubo3, r2);
  //# # # global SRlaser=pose_add(SRmesa2,r2)
  //# # # SRlaser=pose_add(SRlaser,Torigen)

  //# # # movel(pose_trans(SRlaser,p01_local),a=1.2,v=0.003)

  //# # global r2 = p[Torigen[0], Torigen[1], Torigen[2], alpha2 * ur2[0], alpha2 * ur2[1], alpha2 * ur2[2]]
  //# # global r2_inv = pose_inv(r2)

  //# # global SRlaser = pose_trans(SRmesa2, r2)

  //# # movel(pose_trans(SRlaser, p01_local), a = 1.2, v = 0.003)

  //# popup("check paso 2, apagar laser", "Mensaje", False, False, True)

  // Establece el TCP en la posición de cubo4 y mueve el láser a SRlaser_PLA2
  set_tcp(cubo4);
  movel(SRlaser_PLA2);

  // Actualización de la posición del láser
  //# global distlaser = analog_in[0]
  //# distlaser = (distlaser-0.004) * (35-25) / (0.02-0.004) + 25
  //# global correccion = -(30 - distlaser) / 1000
  //# # cubo2=pose_trans(cubo2,p[0,0,-correccion,0,0,0])
  //Pose SRlaser_PLA3 = SRlaser_PLA2;
  //# SRlaser_PLA3[2] = SRlaser_PLA3[2] + correccion
  //# set_tcp(cubo4)
  //# movel(SRlaser_PLA3, a = 1, v = 0.003)

  //# global cubo4 = cubo2
  //# global SRlaser_PLA3 = SRlaser_PLA2

  message(logger, "FIN DE LA CALIBRACIÓN", true);
  
  return true;
}

int main(int argc, char * argv[]) { 
  // Inicialización de ROS
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "calibration_node", 
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  RCLCPP_INFO(node->get_logger(), "Nodo de ROS creado.");

  // Crear ROS logger
  auto const logger = rclcpp::get_logger("calibration_node");

  // Interfaz MoveIt
  static const std::string PLANNING_GROUP = "ur_manipulator";

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, PLANNING_GROUP);

  RCLCPP_INFO(node->get_logger(), "Moveit inicializado.");

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  //moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState(10);
  
  bool success = calibration(node, move_group_interface, node->get_logger());

  if (success) {
    RCLCPP_INFO(node->get_logger(), "Calibración completada con éxito.");
  } else {
    RCLCPP_ERROR(node->get_logger(), "Error en la calibración.");
  }

  rclcpp::shutdown();

}
