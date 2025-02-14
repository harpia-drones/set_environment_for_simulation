#!/bin/bash

echo "=================================================================="
echo "Iniciando setup.sh..."
echo "=================================================================="

# Atualizar e fazer upgrade do sistema
echo "=================================================================="
echo "Atualizando o sistema..."
echo "=================================================================="
apt-get upgrade -y
apt-get update

# Criar um ambiente virtual Python
echo "=================================================================="
echo "Criando um ambiente virtual python..."
echo "==================================================================" 
apt-get install -y python3-venv 
cd /root/ && python3 -m venv harpia 
source harpia/bin/activate 

# Instalar a toolchain de desenvolvimento do PX4 para usar o simulador
echo "=================================================================="
echo "Instalando PX4..."
echo "==================================================================" 
cd /root/ && git clone https://github.com/PX4/PX4-Autopilot.git --recursive 
bash /root/PX4-Autopilot/Tools/setup/ubuntu.sh 

# Instalar algumas dependências para ROS
echo "=================================================================="
echo "Instalando dependências do ROS..."
echo "=================================================================="
pip3 install -U empy pyros-genmsg setuptools catkin_pkg lark 
apt-get install -y ros-dev-tools 

# Instalar o XRCE-DDS Agent
echo "=================================================================="
echo "Instalando Micro-XRCE-DDS-Agent..."
echo "==================================================================" 
cd /root/ && git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git 
cd /root/Micro-XRCE-DDS-Agent 
mkdir build 
cd build 
cmake .. 
make 
make install 
ldconfig /usr/local/lib/ 

# Clonar os repositórios necessários
echo "=================================================================="
echo "Clonando repositórios..."
echo "==================================================================" 
cd /root/estudos_ws/src/ 
git clone https://github.com/PX4/px4_msgs.git 
git clone https://github.com/PX4/px4_ros_com.git

# Função para tentar rodar o colcon build
build_with_retry() {
    while true; do
        echo "Executando colcon build..."
        cd /root/estudos_ws/ && colcon build --packages-ignore bringup description interfaces
        if [ $? -eq 0 ]; then  # Verifica se o comando anterior foi bem-sucedido
            echo "colcon build executado com sucesso!"
            break  # Sai do loop se o comando for bem-sucedido
        else
            echo "colcon build falhou. Tentando novamente..."
            sleep 2  # Espera 2 segundos antes de tentar novamente (opcional)
        fi
    done
}

# Chama a função
build_with_retry

echo "=================================================================="
echo "setup.sh concluído com sucesso!"
echo "=================================================================="