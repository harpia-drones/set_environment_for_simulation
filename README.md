# Repositório de Estudos - Docker, ROS2 e Gazebo

Bem-vindo ao repositório de estudos da **Equipe Harpia**. Este repositório centraliza o conhecimento sobre Docker, ROS2 e Gazebo.

## Como usar este repositório

### 1. Clonar o repositório

    git clone git@github.com:harpia-drones/Estudos.git
    cd Estudos

### 2. Construir e iniciar o container

Use o Docker Compose para construir e iniciar o ambiente:

    docker-compose up -d

### 3. Acessar o container

Para acessar o terminal do container, utilize:

    docker exec -it harpia bash

### 4. Usando multiplos terminais

Ao acessar o terminal bash do container, inicie uma nova seção do tmux, executando:

    tmux

Para permitir o ROS2 identifique os packages existentes no volume /estudos_ws/src, assim que entrar no container execute:

    colcon build 
    bashrc

Faça isso toda vez que acessar um novo container. 

> Comandos tmux:

- **Dividir horizontalmente**: ctrl + b, %
- **Dividir verticalmente**: ctrl + b, "
- **Fechar um terminal**: exit
- **Navegar entre os terminais de uma mesma sessão**: crtl + seta

## Estrutura do diretório

```
    Estudos/
    ├── estudos_ws/
    │   └── src/ 
    │        ├── depencies/
    │        │   ├── bringup/
    │        │   │    ├── CMakeLists.txt.txt
    │        │   │    └── package.xml.txt
    │        │   └── interfaces/
    │        │       ├── CMakeLists.txt.txt
    │        │       └── package.xml.txt
    │        └── Makefile
    ├── compose.yaml
    └── Dockerfile
```

## Descrição dos arquivos

- **compose.yaml**: Configuração do Docker Compose para o container.
- **Dockerfile**: Especificações para construir a imagem do ROS2 e Gazebo.

