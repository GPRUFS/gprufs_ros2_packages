# Biblioteca ROS 2 
Esse repositório contém alguns pacotes para o ROS 2 desenvolvidos para realizar interfaces com os robôs do Laboratório de Pesquisa em Robótica da Universidade Federal de Sergipe. Aqui contém tanto os códigos a serem rodados no PC, quando os que serão executados no computador de bordo do robô.

## Índice
- [Requisitos](#requisitos)
- [Como usar?](#como-usar)


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
|[py_bubbleRobotController](https://github.com/rodrigopassoss/gprufs_ros2_packages/tree/main/py_bubbleRobotController) | walk | PC | Nó usado para o controle do Bobble Robot do CoppeliaSim. O nó recebe os dados sensoriais (uma flag que indica se há obstáculo a frente do robô) por meio do tópico `/robot/sensorTrigger`, e publica as velocidades das rodas esquerda e direita, respectivamente pelos tópicos `/robot/leftMotorSpeed` e `/robot/rightMotorSpeed`|

