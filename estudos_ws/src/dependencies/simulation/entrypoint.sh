#!/bin/bash

# Caminho para um arquivo de flag indicando que o script já rodou
FLAG_FILE='/root/.setup_done'

# Se o arquivo de flag não existir, executa o script e cria o flag
if [ ! -f "$FLAG_FILE" ]; then
    chmod -R +x /root/estudos_ws/src/dependencies && \
    /root/estudos_ws/src/dependencies/simulation/setup.sh
    touch "$FLAG_FILE"

else
    echo "=================================================================="
    echo "Environment setup concluded succesfully!"
    echo "=================================================================="
fi

# Executa o comando original do container
exec "$@"