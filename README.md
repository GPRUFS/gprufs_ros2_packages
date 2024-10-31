# Biblioteca ROS 2 
Esse repositório contém alguns pacotes para o ROS 2 desenvolvidos para realizar interfaces com os robôs do Laboratório de Pesquisa em Robótica da Universidade Federal de Sergipe. Aqui contém tanto os códigos a serem rodados no PC, quando os que serão executados no computador de bordo do robô.

## Índice
- [Requisitos](#requisitos)
- [Como usar?](#como-usar)
- [Descreição dos Pacotes](#descrição-dos-pacotes)
- [Estrutura Básica de um Nó ROS 2](#estrutura-básica-de-um-nó-ros-2)


## Requisitos

* [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
* [Colcon](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html#install-colcon) (Para *buildar* os códigos)
* [git](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git)

Se você está começando no ROS 2, recomendo fortemente que você siga os [tutoriais oficiais](https://docs.ros.org/en/humble/Tutorials.html) do ROS 2. 

## Como usar?

  1. O primeiro passa é apontar o local onde o arquivo de configuração do ROS 2 está ([Source ROS 2 environment](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#source-ros-2-environment)):

No windows:

   * `call C:\dev\ros2\local_setup.bat` (troque `C:\dev` pelo dieretório em que o ROS 2 foi instalado, caso não tenha sido nesse.)
     
No Linux:

   * `source /opt/ros/humble/setup.bash`

Obs.: Essa etapa deve ser realizada sempre que um terminal novo for aberto.

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

Nesse ponto o diretírio `ros_ws` possui apenas um diretório:
<pre>
.
└── src

1 directory, 0 files
</pre>


  3. Clonar o repositório em `~\ros2_ws\src`. Considerado que você já está no diretório `~\ros2_ws\src`:
<pre>
git clone https://github.com/rodrigopassoss/gprufs_ros2_packages.git
</pre>     

  4. Vai para o diretório `~\ros2_ws`, e builda o *workspace* usando colcon (veja [Build the workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#build-the-workspace-with-colcon)):

No windows:

<pre>
colcon build --merge-install
</pre>
     
No Linux:

<pre>
colcon build
</pre>

Nesse ponto o diretírio `ros_ws` possui os seguintes diretórios:
<pre>
.
├── build
├── install
├── log
└── src

4 directories, 0 files
</pre>

Obs.: Para instalar apenas um dos pacotes, por exemplo, apenas o `py_calmaN`, use o comando `colcon build --packages-select py_calmaN`.

  4. Após concluir essas etapas, pode fechar o terminal e abrir um novo. No novo terminal, no diretório `ros_ws`:

No windows:

<pre>
call C:\dev\ros2\local_setup.bat
call install\local_setup.bat
</pre>
     
No Linux:

<pre>
source /opt/ros/humble/setup.bash
source install/local_setup.bash
</pre>

  5. Agora você pode executar qualquer pacote usando o comando `ros2 run`, por exemplo:

<pre>
ros2 run py_calmaN walk
</pre>

## Descrição dos Pacotes

| Pacote       | Nó | Dispositivo |  Descrição      |
|----------------------|-------------------------|-------------------------|-------------------------|
|[py_bubbleRobotController](https://github.com/rodrigopassoss/gprufs_ros2_packages/tree/main/py_bubbleRobotController/py_bubbleRobotController) | [walk](https://github.com/rodrigopassoss/gprufs_ros2_packages/blob/main/py_bubbleRobotController/py_bubbleRobotController/walk.py) | PC | Nó usado para o controle do [Bobble Robot do CoppeliaSim](https://github.com/rodrigopassoss/gprufs_v-rep_projects/tree/main/scenes). O nó recebe os dados sensoriais (uma flag que indica se há obstáculo a frente do robô) por meio do tópico `/robot/sensorTrigger`, e publica as velocidades das rodas esquerda e direita, respectivamente pelos tópicos `/robot/leftMotorSpeed` e `/robot/rightMotorSpeed`|
|[py_calmaN](https://github.com/rodrigopassoss/gprufs_ros2_packages/tree/main/py_calmaN/py_calmaN) | [encoder_test](https://github.com/rodrigopassoss/gprufs_ros2_packages/blob/main/py_calmaN/py_calmaN/encoder_test.py) | PC | Esse nó realiza apenas a aquisição dos dados do enconder com proposito de teste. Os dados são recebidos pelo tópico `/robot/encoder`. Esse nó foi testado apenas com o [modelo do calmaN no Coppelia](https://github.com/rodrigopassoss/gprufs_v-rep_projects/tree/main/scenes), mas pode ser aplicado ao robô real|
|[py_calmaN](https://github.com/rodrigopassoss/gprufs_ros2_packages/tree/main/py_calmaN/py_calmaN) | [lidar_test](https://github.com/rodrigopassoss/gprufs_ros2_packages/blob/main/py_calmaN/py_calmaN/lidar_test.py) | PC | Esse nó realiza apenas a aquisição dos dados do lidar, e tbm plota os pontos adiquiridos, com proposito de teste. Os dados são recebidos pelo tópico `/robot/lidar`. Esse nó foi testado tanto com a [simuação](https://github.com/rodrigopassoss/gprufs_v-rep_projects/tree/main/scenes) quanto com o Lidar real. Esse mesmo teste também foi implementado com o [MATLAB/Octave](https://github.com/rodrigopassoss/gprufs_ros2_udp).|
|[py_calmaN](https://github.com/rodrigopassoss/gprufs_ros2_packages/tree/main/py_calmaN/py_calmaN) | [walk](https://github.com/rodrigopassoss/gprufs_ros2_packages/blob/main/py_calmaN/py_calmaN/walk.py) | PC | Controlador simples para o robô, que usa as informações do lidar para navegar pelo ambiente sem colidir com os obstáculos. As informações do Lidar são lidas por meio do tópico `/robot/lidar`, e as informações de velocidade são publicadas no tópico `/robot/cmd_vel`. Esse mesmo nó também foi implementado com o [MATLAB/Octave](https://github.com/rodrigopassoss/gprufs_ros2_udp).|
|[py_calmaN](https://github.com/rodrigopassoss/gprufs_ros2_packages/tree/main/py_calmaN/py_calmaN) | [matlab_upd_link](https://github.com/rodrigopassoss/gprufs_ros2_packages/blob/main/py_calmaN/py_calmaN/matlab_udp_link.py) | PC | A função desse nó é gerar um [link de comunicação via UDP entre o MATLAB/Octave com o ROS 2](https://github.com/rodrigopassoss/gprufs_ros2_udp). Esse nó é subscrito no tópico `/robot/lidar`, de modo que sempre que uma informação chega, é enviada via UDP. No [MATLAB/Octave](https://github.com/rodrigopassoss/gprufs_ros2_udp), a informação do Lidar é processada e enviada via UDP. O nó `matlab_upd_link` recebe as velocidades e publica no tópico `/robot/cmd_vel`. Esse nó pode ser usado como interface tanto para o simulador quanto para o robô real. |
|[py_calmaN](https://github.com/rodrigopassoss/gprufs_ros2_packages/tree/main/py_calmaN/py_calmaN) | [walk_pub](https://github.com/rodrigopassoss/gprufs_ros2_packages/blob/main/py_calmaN/py_calmaN/walk_pub.py) | Raspbarry Pi (Computador de Bordo do Robô) | Esse nó publica os dados do lidar no tópico `robot/lidar` e recebe dados pelo tópico `robot/cmd_vel`, e o dados recebidos são enviados via serial para um microcontrolador resposável por controlar os motores do robô.|
|[py_camera](https://github.com/rodrigopassoss/gprufs_ros2_packages/tree/main/py_camera/py_camera) | [camera_pub](https://github.com/rodrigopassoss/gprufs_ros2_packages/blob/main/py_camera/py_camera/publisher_camera_function.py) | PC/Raspbarry Pi (Computador de Bordo do Robô) | Esse nó realiza uma interface com a câmera e publica as imagens no tópico `robot/camera`.|
|[py_camera](https://github.com/rodrigopassoss/gprufs_ros2_packages/tree/main/py_camera/py_camera) | [camera_pub2](https://github.com/rodrigopassoss/gprufs_ros2_packages/blob/main/py_camera/py_camera/publisher_camera_function2.py) | PC/Raspbarry Pi (Computador de Bordo do Robô) | Esse nó realiza uma interface com a câmera e publica as imagens no tópico `robot/camera`, com uma abordagem usando temporizador, que envia as imagem em intervalos de tempo definidos.|
|[py_camera](https://github.com/rodrigopassoss/gprufs_ros2_packages/tree/main/py_camera/py_camera) | [camera_sub](https://github.com/rodrigopassoss/gprufs_ros2_packages/blob/main/py_camera/py_camera/subscriber_camera_function.py) | PC | Esse nó recebe as imagens da câmera por meio do tópico `robot/camera` e exibe a imagem junto com o FPS.|
|[py_lidar](https://github.com/rodrigopassoss/gprufs_ros2_packages/tree/main/py_lidar/py_lidar) | [lidar_pub](https://github.com/rodrigopassoss/gprufs_ros2_packages/blob/main/py_lidar/py_lidar/publisher_lidar_function.py) | PC/Raspbarry Pi (Computador de Bordo do Robô)  |  Esse nó realiza uma interface com o [Lidar X2](https://www.ydlidar.com/products/view/6.html) e publica as informações no tópico `robot/lidar`.|
|[py_lidar](https://github.com/rodrigopassoss/gprufs_ros2_packages/tree/main/py_lidar/py_lidar) | [lidar_pub2](https://github.com/rodrigopassoss/gprufs_ros2_packages/blob/main/py_lidar/py_lidar/publisher_lidar_function2.py) | PC/Raspbarry Pi (Computador de Bordo do Robô)  |  Esse nó realiza uma interface com o [Lidar X4](https://www.ydlidar.com/products/view/5.html) e publica as informações no tópico `robot/lidar`.|
|[py_lidar](https://github.com/rodrigopassoss/gprufs_ros2_packages/tree/main/py_lidar/py_lidar) | [lidar_sub](https://github.com/rodrigopassoss/gprufs_ros2_packages/blob/main/py_lidar/py_lidar/subscriber_lidar_function.py) | PC/Raspbarry Pi (Computador de Bordo do Robô)  |  Esse nó recebe as informações do Lidar por meio do tópico `robot/lidar` e exibe o FPS.|

Para executar qualquer pacote deve-se utilizar o comando `ros2 run <Pacote> <Nó>`.

## Estrutura Básica de um Nó ROS 2

Todo nó do ROS 2 possui a estrutura básica apresentada na tabela abaixo.

<div align="center">
  
||Etapa|Função do Python|
|----------------------|-------------------------|-------------------------|
| 1 | Inicialização | `rclpy.init(args=args)` | 
| 2 | Criação de um ou mais nós | `node = Node()` | 
| 3 | Processamento e Callbacks | `rclpy.spin(node)` |
| 4 | Finalização | `node.destroy_node()` e `rclpy.shutdown()`|

</div>

No ROS 2 os nós podem interagir por meio de 3 abordagens principais: [Tópicos](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html) (`ros2 topic`), [Serviços](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html) (`ros2 service`), [Ações](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html) (`ros2 action`).

Até então, a interação entre os nós presentes nesse repositório, foi implementada usando a abordagem de tópicos, com as conexões ilustradas no grafo abaixo.

<img src="https://github.com/user-attachments/assets/39d7f2dc-d3e5-48fc-b13c-8209e0aad4d0" alt="grafo" width="50%">


