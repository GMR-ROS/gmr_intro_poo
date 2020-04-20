#include <gmr_intro_poo/gmr_intro_poo.hpp>

int main(int argc, char **argv)
{
    // Inicialização da ROS no contexto deste nó
    ros::init(argc, argv, "gmr_intro_poo_node");
    ros::NodeHandle nh("~");
    ros::Rate loop_rate(50); //loop rate in Hz
    // Criação de uma instância de um objeto da classe RobotClass
    // em que um ponteiro a NodeHandle é passado como argumento
    RobotClass robot(&nh);
    // Loop de ROS para manter o nó vivo
    while(ros::ok())
    {
        robot.calculateOdom();
        loop_rate.sleep();
        ros::spinOnce();
    }
}
