#!/bin/bash

# Caminho para um arquivo de flag indicando que o script já rodou
FLAG_FILE='/root/setup/.setup_done'

# Se o arquivo de flag não existir, executa o script e cria o flag
if [ ! -f "$FLAG_FILE" ]; then
    chmod -R +x /root/setup/ && \
    /root/setup/setup.sh
    touch "$FLAG_FILE"

else
    echo "=================================================================="
    echo "Environment setup concluded succesfully!"
    echo "=================================================================="
fi