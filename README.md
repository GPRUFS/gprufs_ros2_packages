# Biblioteca ROS 2 
Esse repositório contém alguns pacotes para o ROS 2 desenvolvidos para realizar interfaces com os robôs do Laboratório de Pesquisa em Robótica da Universidade Federal de Sergipe.

## Requisitos

* [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
* [Colcon](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html#install-colcon) (Para *buildar* os códigos)
* [git](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git)

Se você está começando no ROS 2, recomendo fortemente que você siga os [tutoriais oficiais](https://docs.ros.org/en/humble/Tutorials.html) do ROS 2. 

## Como usar?

  1. O primeiro passa é apontar o local onde o arquivo de configuração do ROS 2 está ([Source ROS 2 environment](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#source-ros-2-environment)):

No windows:

   * `call C:\dev\ros2\local_setup.bat` (troque `C:\dev` pelo dieretório em que o ROS 2 foi instalado, caso não tenha sido nesse)
     
No Linux:

   * `source /opt/ros/humble/setup.bash`

  2. criar um *workspace* (veja [Creating a Workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)):

No windows:

<pre>
md \ros2_ws\src
cd \ros2_ws\src
</pre>
     
No Linux:

<pre>
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
</pre>
