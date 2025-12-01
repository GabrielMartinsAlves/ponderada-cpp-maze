# Projeto Culling Games (cg)

Este documento fornece um tutorial sobre como construir, executar e interagir
com o projeto Culling Games ROS 2.

## 1. Construindo o Workspace

Antes de executar qualquer parte do projeto, você precisa construir os pacotes.
Navegue até a raiz do workspace e execute:

```bash
colcon build
```

Este comando irá compilar todos os pacotes (`cg`, `cg_interfaces`, `cg_teleop`).
Lembre-se de "source" o workspace em qualquer novo terminal que você abrir:

```bash
source install/setup.bash
```

## 2. Executando o Jogo

O jogo principal é uma janela Pygame que exibe o labirinto e o movimento do
robô.

Para iniciar o jogo, execute o seguinte comando em um terminal:

```bash
ros2 run cg maze
```

### Opções de Carregamento do Labirinto

Você pode especificar como o labirinto é carregado usando argumentos adicionais:

*   **Carregar um Labirinto Aleatório (Padrão):** Se nenhum argumento for fornecido, o jogo selecionará um labirinto aleatório do diretório `src/cg/maps`.
    ```bash
    ros2 run cg maze
    ```
*   **Carregar um Labirinto Específico:** Para carregar um labirinto pelo nome do arquivo (por exemplo, `test.csv`):
    ```bash
    ros2 run cg maze -- --map test.csv
    ```
*   **Gerar um Novo Labirinto:** Para gerar um novo labirinto aleatório e usá-lo imediatamente (esta opção tem precedência sobre `--map`):
    ```bash
    ros2 run cg maze -- --generate
    ```

## 3. Controlando o Robô

Existem duas maneiras de controlar o robô: usando o nó de teleoperação fornecido
ou enviando chamadas de serviço.

### Método A: Usando o Nó de Teleoperação por Teclado

Esta é a maneira mais fácil de jogar.

1.  Em um **terminal separado** (enquanto o comando `ros2 run cg maze` ainda
    estiver em execução), inicie o nó de teleoperação:
    ```bash
    ros2 run cg_teleop teleop_keyboard
    ```
2.  O terminal exibirá as teclas de atalho. Use as seguintes teclas neste
    terminal para mover o robô na janela do jogo:
    *   **Cima:** `w`, `k`, ou a tecla Seta para Cima
    *   **Baixo:** `s`, `j`, ou a tecla Seta para Baixo
    *   **Esquerda:** `a`, `h`, ou a tecla Seta para Esquerda
    *   **Direita:** `d`, `l`, ou a tecla Seta para Direita

### Método B: Enviando Chamadas de Serviço Manuais

Você também pode enviar comandos de movimento individuais usando o serviço
`/move_command`. Isso é útil para scripts ou depuração.

Para mover o robô um passo, use o comando `ros2 service call`. Por exemplo, para
mover para cima:

```bash
ros2 service call /move_command cg_interfaces/srv/MoveCmd "{direction: 'up'}"
```

Substitua `'up'` por `'down'`, `'left'` ou `'right'` para outras direções.

## 4. Sensoriamento do Ambiente

O robô publica continuamente seus arredores imediatos em um tópico. Isso simula
dados de sensores, mostrando o que está nas 8 células adjacentes (incluindo
diagonais).

*   **Tópico:** `/culling_games/robot_sensors`
*   **Tipo de Mensagem:** `cg_interfaces/msg/RobotSensors`

Para ver esses dados em tempo real, abra um novo terminal e execute:

```bash
ros2 topic echo /culling_games/robot_sensors
```

Você verá um fluxo de mensagens mostrando o que está nas células `up`, `down`,
`left`, `right`, `up_left`, etc., em relação ao robô.

## 5. Reiniciando o Jogo

O serviço `/reset` permite reiniciar o tabuleiro do jogo. Ele suporta dois
modos.

### Reiniciando o Labirinto Atual

Se você quiser tentar o *mesmo* labirinto novamente desde o início.

*   **Com Teleoperação:** Pressione a tecla `r` no terminal `cg_teleop`.
*   **Comando Manual:**
    ```bash
    ros2 service call /reset cg_interfaces/srv/Reset "{is_random: false}"
    ```

### Carregando um Novo Labirinto Aleatório

Se você quiser um novo desafio com um labirinto novo e selecionado
aleatoriamente.

*   **Com Teleoperação:** Pressione a tecla `n` no terminal `cg_teleop`.
*   **Comando Manual:**
    ```bash
    ros2 service call /reset cg_interfaces/srv/Reset "{is_random: true}"
    ```
A resposta do serviço informará o nome do arquivo do novo labirinto que foi
carregado.

## 6. Obtendo os Dados Completos do Labirinto

Se você quiser obter o layout de todo o labirinto atual (por exemplo, para
construir um mapa externo), você pode usar o serviço `/get_map`.

```bash
ros2 service call /get_map cg_interfaces/srv/GetMap
```

Isso retornará uma representação "achatada" da grade do labirinto e suas
dimensões.

## 7. Navegação Autônoma (`cg_solver`)

Esta seção descreve como executar o nó de navegação autônoma em C++ que resolve o labirinto.

### Visão Geral

O pacote `cg_solver` contém o nó `autonomous_navigator_node`, que implementa a lógica para resolver os desafios do labirinto.

*   **Parte 1 (Navegação com Mapa):** O nó utiliza o serviço `/get_map` para obter o mapa completo do labirinto. Ele então representa o labirinto como um grafo e emprega o **algoritmo de busca A*** (A-Star) para calcular o caminho ótimo do robô até o alvo. Uma vez que o caminho é calculado, ele o executa passo a passo, enviando comandos para o serviço `/move_command`.

*   **Parte 2 (Mapeamento):** A estrutura para o mapeamento está presente. O nó pode ser configurado para um modo de exploração (`map_unknown_`). Neste modo, a intenção é usar um algoritmo de busca (como BFS, implementado no código) para visitar sistematicamente as células do labirinto. O nó se inscreve no tópico `/culling_games/robot_sensors` para construir um mapa interno (`partial_map_`), que pode então ser usado pelo algoritmo A* para encontrar o caminho.

### Ambiente de Desenvolvimento (Docker)

Para garantir um ambiente consistente e contornar problemas de compatibilidade (especialmente com a exibição de GUI em sistemas Wayland como o Fedora), é **altamente recomendado** o uso do Docker.

**1. Construir a Imagem Docker:**

A partir da raiz do projeto, construa a imagem que contém todas as dependências do ROS 2, do simulador e das bibliotecas de GUI.

```bash
docker build -t culling_games_ros .
```

**2. Configurar Permissões de Exibição (Apenas na Primeira Vez):**

Para permitir que a janela do jogo (GUI) de dentro do contêiner apareça no seu desktop, execute o seguinte comando no seu terminal **host** (fora do contêiner):

```bash
xhost +local:docker
```

**3. Executar o Contêiner de Desenvolvimento:**

Execute o seguinte comando no seu terminal **host** para iniciar o contêiner. Este comando mapeia seu diretório de projeto, o soquete de exibição gráfica e as configurações de autenticação para dentro do contêiner.

```bash
docker run -it --rm --network host --ipc=host \
    --user "$(id -u):$(id -g)" \
    -e DISPLAY=:0 \
    -e XAUTHORITY=/tmp/.Xauthority \
    -v "$HOME/.Xauthority":/tmp/.Xauthority:Z \
    -v /tmp/.X11-unix:/tmp/.X11-unix:Z \
    -v "$(pwd)":/project_ws:Z \
    culling_games_ros bash
```
*Após executar este comando, seu terminal estará dentro do ambiente Docker, pronto para compilar e executar o código.*

### Executando a Solução Autônoma

Você precisará de **dois terminais** dentro do contêiner.

**Terminal 1: Iniciar o Simulador**

*   Use seu terminal de contêiner atual ou abra um novo com o comando `docker exec` (descrito abaixo).
*   Compile o workspace e inicie o simulador do labirinto:
    ```bash
    colcon build
    source install/setup.bash
    ros2 run cg maze
    ```
    A janela do Pygame com o labirinto deve aparecer.

**Terminal 2: Iniciar o Solver Autônomo**

1.  Abra um **novo terminal no seu computador host**.
2.  Encontre o nome do seu contêiner em execução:
    ```bash
    docker ps
    ```
3.  Abra um shell nesse contêiner. Por exemplo, se o nome do contêiner for `nostalgic_ganguly`:
    ```bash
    docker exec -it nostalgic_ganguly bash
    ```
4.  Dentro deste segundo terminal do contêiner, execute o nó solver, **especificando o modo desejado**:

    *   **Modo `pathfind` (Parte 1 - Padrão):** O robô usa o mapa completo para encontrar o caminho mais rápido.
        ```bash
        source install/setup.bash && export ROS_LOG_DIR=/project_ws/log && ros2 run cg_solver autonomous_navigator_node
        ```
        ou explicitamente:
        ```bash
        source install/setup.bash && export ROS_LOG_DIR=/project_ws/log && ros2 run cg_solver autonomous_navigator_node --ros-args -p mode:=pathfind
        ```

    *   **Modo `explore` (Parte 2 - Mapeamento e Navegação):** O robô constrói o mapa primeiro, retorna ao início e depois busca o alvo.
        ```bash
        source install/setup.bash && export ROS_LOG_DIR=/project_ws/log && ros2 run cg_solver autonomous_navigator_node --ros-args -p mode:=explore
        ```

Ao executar o último comando, você verá o robô começar a se mover na janela do Pygame, executando a tarefa conforme o modo selecionado.
