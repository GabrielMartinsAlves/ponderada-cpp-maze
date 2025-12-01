# Use a imagem oficial do ROS 2 Humble como base.
FROM ros:humble-ros-base

# Define o shell padrão para bash para garantir a consistência dos comandos.
SHELL ["/bin/bash", "-c"]

# Atualiza a lista de pacotes e instala todas as dependências de sistema
# necessárias para compilar o ROS e o seu projeto.
# Estas são as contrapartes Debian/Ubuntu das dependências listadas no seu
# guia para Fedora.
RUN apt-get update && apt-get install -y \
    build-essential \
    g++ \
    git \
    cmake \
    cppcheck \
    python3-dev \
    python3-pip \
    python3-colcon-common-extensions \
    python3-vcstool \
    libtinyxml-dev \
    libtinyxml2-dev \
    libpoco-dev \
    libasio-dev \
    xauth \
    x11-apps \
    # Limpa o cache do apt para manter a imagem pequena.
    && rm -rf /var/lib/apt/lists/*

# Instala as ferramentas Python necessárias.
RUN pip3 install -U rosdep flake8-docstrings pygame

# Inicializa o rosdep, a ferramenta de dependências do ROS.
# É uma boa prática tê-lo pronto, caso precise dele mais tarde.
RUN rosdep init || true && rosdep update

# Cria e define o diretório de trabalho principal dentro do contêiner.
# O seu projeto será montado aqui.
WORKDIR /project_ws

# Adiciona o script de setup do ROS ao .bashrc.
# Isso garante que qualquer terminal interativo que você abrir no contêiner
# já terá os comandos do ROS disponíveis.
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
