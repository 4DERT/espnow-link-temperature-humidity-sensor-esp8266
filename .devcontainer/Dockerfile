FROM mcr.microsoft.com/devcontainers/base:noble
RUN userdel -r ubuntu

RUN apt-get update && apt-get install -y \
    gcc git wget make libncurses-dev flex bison gperf python3 python3-serial python3-pip \
    socat

RUN ln -s /bin/python3 /bin/python

USER vscode

RUN wget --directory-prefix=/home/vscode https://dl.espressif.com/dl/xtensa-lx106-elf-gcc8_4_0-esp-2020r3-linux-amd64.tar.gz
RUN mkdir -p /home/vscode/esp
RUN tar -xzf /home/vscode/xtensa-lx106-elf-gcc8_4_0-esp-2020r3-linux-amd64.tar.gz --directory=/home/vscode/esp
RUN git clone --recursive https://github.com/espressif/ESP8266_RTOS_SDK.git /home/vscode/esp/ESP8266_RTOS_SDK

RUN echo "export PATH=\"$PATH:/home/vscode/esp/xtensa-lx106-elf/bin\"" >> /home/vscode/.bashrc
RUN echo "export IDF_PATH=\"/home/vscode/esp/ESP8266_RTOS_SDK\"" >> /home/vscode/.bashrc
RUN python -m pip install --user -r "/home/vscode/esp/ESP8266_RTOS_SDK/requirements.txt" --break-system-packages
