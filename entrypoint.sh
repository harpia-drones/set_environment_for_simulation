#!/bin/bash

# Caminho para um arquivo de flag indicando que o script já rodou
FLAG_FILE='/root/setup/.setup_done'

echo "=================================================================="
echo "Iniciando entrypoint.sh..."
echo "=================================================================="

# Se o arquivo de flag não existir, executa o script e cria o flag
if [ ! -f "$FLAG_FILE" ]; then
    echo "Executando setup.sh pela primeira vez..."
    chmod -R +x /root/setup/ && \
    /root/setup/setup.sh
    if [ $? -eq 0 ]; then
        echo "setup.sh concluído com sucesso."
        touch "$FLAG_FILE"
    else
        echo "Erro ao executar setup.sh."
        exit 1
    fi
else
    echo "=================================================================="
    echo "Environment setup concluído com sucesso!"
    echo "=================================================================="
fi

echo "=================================================================="
echo "Finalizando entrypoint.sh."
echo "=================================================================="