# TEMPLATE PARA A CRIAÇÃO DE LAUNCH FILES BÁSICOS
# Para entender esse arquivo, leia o documento "Launch File" na documentação da equipe
# disponível no Drive.

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    node_A = Node(
        package="pkg_name",
        executable="exec_name",    # == original_node_name
        name="nome_name",          # nome com que o nó será iniciado
        namespace="namespace",
        remappings=[
            ("original_topic_name", "new_topic_name"),
            ("original_service_name", "new_topic_name"),
        ],
        parameters=[
            {"param_name": param_value}
        ]
    )

    ld.add_action(node_A)

    return ld