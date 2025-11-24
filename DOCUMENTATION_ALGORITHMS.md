DOCUMENTAÇÃO TÉCNICA: ALGORITMOS E USO DO REPOSITÓRIO ponderada-cpp-maze

Sumário
-------
1. Introdução
2. Visão geral da arquitetura do projeto
3. Contratos de mensagens e serviços ROS2 utilizados
4. Descrição detalhada dos algoritmos implementados
   4.1. A* (A-star) — algoritmo de planejamento de caminhos
   4.2. BFS (Breadth-First Search) — algoritmo de mapeamento exploratório
   4.3. Lógica de movimentação e integração com serviços
5. Representação do mapa e convenções de dados
6. Critérios de correção e casos de borda
7. Requisitos de ambiente, dependências e pré-requisitos
8. Procedimentos de construção e execução (comandos principais)
9. Depuração e resolução de problemas comuns
10. Recomendações para extensão e manutenção

1. Introdução
--------------
Este documento fornece uma descrição abrangente, técnica e formal dos algoritmos implementados no repositório "ponderada-cpp-maze" e das instruções necessárias para compilar, executar e estender os componentes existentes. O texto foi elaborado para servir como referência para engenheiros de software, pesquisadores e integradores que necessitem compreender o comportamento algorítmico, o contrato entre componentes (mensagens/serviços ROS2) e as implicações práticas ao executar este software.

2. Visão geral da arquitetura do projeto
---------------------------------------
O repositório implementa um simulador de labirintos e um nó de navegação autônoma que interage por meio de serviços ROS2. Os componentes principais são:
- Simulador/Servidor (implementado em Python): provê serviços ROS2 para obtenção do mapa (GetMap), execução de comandos de movimento (MoveCmd) e reset do simulador (Reset). Publica regularmente informações de sensores do robô via `RobotSensors`.
- Nó de navegação autônoma (implementado em C++): consome os serviços do simulador para obter o mapa e enviar comandos de movimento. Implementa algoritmos de mapeamento (BFS) e planejamento de trajetória (A*).
- Interfaces ROS2: mensagens e serviços definidos em `cg_interfaces` (GetMap, MoveCmd, Reset, RobotSensors).

A interação básica segue o padrão cliente/servidor do ROS2:
- O nó de navegação requisita o mapa via `/get_map` (serviço GetMap).
- O simulador responde com uma grade de ocupação flattenada e sua forma (altura, largura).
- O nó de navegação calcula um caminho até o alvo (A*) e emite comandos ao serviço `/move_command` (MoveCmd).
- O simulador publica, no tópico `/culling_games/robot_sensors`, leituras sensoriais locais do entorno, usadas para mapeamento incremental (BFS/exploração).

3. Contratos de mensagens e serviços ROS2 utilizados
----------------------------------------------------
Esta secção descreve o formato esperado das mensagens e serviços com os quais os nós interagem. As implementações no repositório assumem os seguintes contratos (nomes de campo conforme usados no código):

GetMap (serviço)
- Request: vazio (sem campos relevantes no uso atual).
- Response:
  - occupancy_grid_flattened: array/lista de strings; cada elemento é um caractere representando o estado de uma célula: 'b' => parede (blocked), 'f' => livre (free), 'r' => posição do robô (robot), 't' => posição do alvo/target (target). Elementos adicionais ou símbolos desconhecidos são tratados como célula desconhecida.
  - occupancy_grid_shape: lista/array com dois inteiros: [altura, largura] (em unidades de células). A interpretação usada no código é occupancy_grid_shape[0] = altura (número de linhas) e occupancy_grid_shape[1] = largura (número de colunas).

MoveCmd (serviço)
- Request:
  - direction: string; valores esperados: 'up', 'down', 'left', 'right' (case-insensitive no código Python, comparado em minúsculas).
- Response:
  - success: boolean indicando se o movimento foi efetuado.
  - robot_pos: posição atual do robô após tentativa de movimento (geralmente par [x,y] ou estrutura equivalente conforme implementação do simulador).
  - target_pos: posição do alvo (ou posição suposta), retornada pelo simulador no momento da resposta.

Reset (serviço)
- Request:
  - is_random: boolean (se verdadeiro, o simulador escolherá um mapa aleatório do diretório de mapas)
  - map_name: string (nome do mapa a ser carregado)
- Response:
  - success: boolean
  - loaded_map_name: string (nome do mapa efetivamente carregado)

RobotSensors (mensagem publicada no tópico `/culling_games/robot_sensors`)
- Campos de string para cada direção relativa ao robô: up, down, left, right, up_left, up_right, down_left, down_right.
- Cada campo contém um caractere ('b','f','t' etc.) representando o que o sensor detectou nessa direção.

Observação: Os contratos acima foram inferidos a partir do código presente no repositório (arquivos anexados). Caso existam definições formais em `cg_interfaces` com campos adicionais, deve-se considerar essas definições como fonte de verdade. Contudo, o comportamento do nó de navegação e do simulador obedece estritamente aos campos referenciados no código.

4. Descrição detalhada dos algoritmos implementados
----------------------------------------------------
Nesta secção são explicitadas as implementações algorítmicas, estruturas de dados utilizadas, complexidade temporal/espacial e pontos importantes de robustez.

4.1 A* (A-star) — algoritmo de planejamento de caminhos
-------------------------------------------------------
Descrição geral
- A implementação de A* presente no arquivo `cg_autonomous_navigator.cpp` executa o planejamento de um caminho entre duas células discretas em uma grade (grid). As células são representadas pela estrutura `Cell { int x, y; }`.
- A heurística empregada é a distância de Manhattan (L1): h(a,b) = |a.x - b.x| + |a.y - b.y|.

Representações e estruturas de dados
- `came_from`: unordered_map<Cell, Cell> — mapeia cada célula visitada para a célula predecessora no caminho buscado. Usado para reconstruir o caminho quando o objetivo for alcançado.
- `cost_so_far`: unordered_map<Cell, int> — custo mínimo encontrado até o momento para alcançar cada célula.
- `open`: priority_queue<pair<int, Cell>, ...> — fila de prioridade que ordena células por f = g + h (custo acumulado g + heurística h). O comparador prioriza menores valores de f.

Operação (resumida)
1. Inicializa o conjunto aberto com o estado inicial (start) e custo 0.
2. Enquanto houver nós no conjunto aberto:
   a. Extrai o nó com menor f.
   b. Se for o objetivo, interrompe a busca.
   c. Para cada vizinho livre, calcula novo custo (g') = g(current) + custo de transição (aqui uniformemente 1).
   d. Se g' for melhor que custo registrado, atualiza `cost_so_far`, calcula prioridade f = g' + h(vizinho, objetivo) e insere/atualiza o vizinho na fila aberta; armazena `came_from[vizinho] = current`.
3. Ao finalizar, reconstrói o caminho de goal até start via `came_from`.

Complexidade
- Complexidade temporal: O(A * log A) em termos do número de nós expandidos A, devido à manutenção da fila de prioridades; para grades MxN, no pior caso A = O(M*N).
- Complexidade espacial: O(M*N) armazenamento nos mapas `came_from` e `cost_so_far` no pior caso.

Pontos de atenção / implementações específicas
- A heurística de Manhattan é admissível e consistente para movimentos em 4 direções (up/down/left/right) com custo uniforme 1, garantindo que o caminho encontrado seja ótimo.
- O código considera células livres com valores 0 (livre), 2 (robô), 3 (alvo). A presença de outros valores é tratada como obstrução.
- A função `is_free` usa a variável `flat_map_` (mapa completo recebido do serviço). Se estiverem a ser feitos passos de mapeamento incremental (mapa parcial), é necessário garantir que `flat_map_` represente corretamente os obstáculos no momento da busca ou adaptar `is_free` para usar o mapa parcial (`partial_map_`) quando apropriado.
- A reconstrução de caminho assume que `came_from[goal]` existe; caso contrário, é tratada como falha (caminho inexistente).

4.2 BFS (Breadth-First Search) — algoritmo de mapeamento exploratório
--------------------------------------------------------------------
Descrição geral
- A implementação BFS serve como motor de exploração do ambiente quando o mapa completo possui células desconhecidas. A BFS é utilizada para construir uma ordem de exploração a partir da posição atual do robô.
- A BFS emprega uma fila (`std::queue<Cell> bfs_queue_`) e um conjunto `explored_` (std::set<Cell>) para marcar células já enfileiradas ou visitadas.

Operação (resumida)
1. Inserir a célula inicial (posição do robô) na fila e marcar como explorada.
2. Repetidamente retirar a frente da fila, enfileirar os vizinhos livres que ainda não foram explorados.
3. Enquanto existirem células na fila, o nó de navegação tenta deslocar o robô para a próxima célula da fila (método `move_to`), atualizando o mapa parcial com as leituras sensoriais publicadas pelo simulador.
4. Quando a fila esvazia, verifica-se se o mapa parcial contém alguma célula desconhecida (-1). Se não houver, o mapeamento é considerado completo.

Complexidade
- Temporal: O(A) para expandir A células exploradas.
- Espacial: O(M*N) para a fila e conjunto de visitados no pior caso.

Pontos de atenção
- BFS aqui é usado para ordenar movimentos exploratórios (visitar células adjacentes) e não para produzir o caminho mínimo até um alvo (A* realiza esse papel quando o mapa é conhecido).
- Operações de movimento podem falhar (serviço MoveCmd pode retornar success=false); o código deve tratar tais falhas (atualmente há avisos via logs e tentativa de continuar).

4.3 Lógica de movimentação e integração com serviços
----------------------------------------------------
- O método `move_to` no nó C++ traduz uma célula destino em uma string de direção (up/down/left/right) por comparação coordenada com a posição atual do robô. Se a célula destino não for exatamente adjacente, `get_direction` retorna string vazia e o movimento não será solicitado.
- O comando de movimento é enviado ao serviço `move_command` (MoveCmd). O nó aguarda a disponibilidade do serviço e invoca `async_send_request`. Se a chamada completar dentro do tempo limite (2 segundos no código), considera-se o movimento bem sucedido e a posição do robô é atualizada localmente.
- No simulador Python, `handle_move_cmd` aplica a lógica de movimentação sobre a estrutura `Robot`, atualiza `self.win` se o alvo for alcançado e retorna `robot_pos` e `target_pos` no `response`.

5. Representação do mapa e convenções de dados
----------------------------------------------
- Mapas são representados como matrizes 2D (lista de listas) quando manipulados no simulador Python. Para a comunicação via serviço GetMap, o mapa é enviado em formato "flattened": uma lista unidimensional `occupancy_grid_flattened` e a forma `occupancy_grid_shape = [height, width]`.
- Convenções de valores/etiquetas:
  - 'b' ou valor 1: parede / obstáculo.
  - 'f' ou valor 0: célula livre.
  - 'r' ou valor 2: posição do robô.
  - 't' ou valor 3: posição do alvo.
  - '-1' (na representação interna do nó de navegação): célula desconhecida.
- Indexação: o código usa indexação row-major (linha * width + coluna).

6. Critérios de correção e casos de borda
-----------------------------------------
6.1 Casos de ausência de mapa ou serviço indisponível
- O nó de navegação aguarda o serviço `/get_map` por um tempo (10s em `request_map`) e registra avisos se o serviço não estiver disponível. Em ambientes distribuídos, garantir que o simulador esteja ativo antes de iniciar o nó de navegação é requisito operacional.

6.2 Célula alvo inacessível
- Se A* não encontrar caminho (nenhuma entrada em `came_from` para o objetivo), o nó registra erro e encerra a execução. Alternativas possíveis de tratamento: acionar procedimento de replanejamento, ampliar a área conhecida via mapeamento ou notificar operador.

6.3 Falhas na execução de movimentos
- Se o serviço MoveCmd responde com sucesso=false ou a chamada expira, o nó loga uma advertência. Recomenda-se implementar tentativa com recuo (retry/backoff) ou lógica alternativa (ex.: recalcular caminho, marcar célula como obstáculo temporário).

6.4 Mapas parcialmente desconhecidos
- O código atual distingue mapa completo (`flat_map_`) e mapa parcial (`partial_map_`). Ao planejar com A*, recomenda-se garantir coerência entre o mapa usado para is_free() e o conhecimento real do agente. Planejar sobre células rotuladas como desconhecidas pode produzir caminhos inválidos.

6.5 Subsituições e consistência de coordenadas
- A conversão entre o array flattenado e coordenadas (x,y) segue a regra: index = y * width + x. Erros de off-by-one e inverções de largura/altura devem ser verificados quando se integra com mapas externos.

7. Requisitos de ambiente, dependências e pré-requisitos
--------------------------------------------------------
Ambiente mínimo requerido (diretrizes):
- ROS 2 instalado e configurado (uma distribuição compatível com as dependências do projeto). Recomenda-se uma distribuição estável e recente do ROS 2.
- Ferramentas de construção: `colcon` (para compilar pacotes ROS2 C++ e Python).
- Compilador C++ compatível com o projeto (por exemplo, g++ com suporte a C++14/C++17 se requerido pelas dependências).
- Python 3.x (compatível com as bibliotecas usadas no pacote Python do simulador).
- Bibliotecas Python adicionais (para o simulador): `pygame`, `rclpy`, `cg_interfaces` (interfaces geradas), e outras dependências transitivas listadas em `package.xml` e nos requisitos do pacote Python.

Instalação de dependências Python (exemplo):
- Instale ROS2 e crie/ative o ambiente ROS2:
  - source /opt/ros/<sua-distro>/setup.bash
- Instale pacotes Python necessários para o simulador (exemplo):
  - python3 -m pip install --user pygame

Nota: o projeto pode conter botstrap e scripts específicos (por exemplo, `ros2_env`) — consulte os arquivos do repositório para detalhes adicionais. Ajuste as instruções conforme a distribuição ROS 2 que você utiliza.

8. Procedimentos de construção e execução (comandos principais)
---------------------------------------------------------------
A sequência mínima para compilar e executar o simulador e o nó de navegação é a seguinte. Execute-os no diretório raiz do espaço de trabalho (`/home/.../ponderada-cpp-maze/ponderada-cpp-maze`).

1) Configurar o ambiente ROS2 do workspace (source do setup gerado após `colcon build`):

```bash
source install/setup.bash
```

2) Compilar todos os pacotes do workspace com `colcon`:

```bash
colcon build
```

3) Iniciar o simulador do labirinto (executar em um terminal separado):

```bash
ros2 run cg maze
```

4) Iniciar o nó de navegação autônoma (em outro terminal separado):

```bash
ros2 run cg cg_autonomous_navigator
```

5) Observações operacionais:
- Sempre garanta que `install/setup.bash` (ou `local_setup.bash`) tenha sido sourciado no terminal onde os nós ROS2 serão executados, para que as mensagens/serviços gerados estejam disponíveis.
- Se usar `ros2 run` e receber erro de pacote não encontrado, confirme que `colcon build` concluiu com sucesso e que o terminal atual passou pelo `source install/setup.bash`.

9. Depuração e resolução de problemas comuns
--------------------------------------------
Problema: Serviço `/get_map` indisponível
- Causa provável: simulador (`ros2 run cg maze`) não está em execução ou o nó do simulador falhou ao inicializar.
- Ações: inicie o simulador; verifique logs do nó; confirme com `ros2 service list` se `/get_map` aparece.

Problema: `ros2 run cg cg_autonomous_navigator` falha ao compilar
- Causa provável: dependências C++ ou mensagens `cg_interfaces` não geradas.
- Ações: execute `colcon build --event-handlers console_cohesion+` para obter saída detalhada, verifique `package.xml` e `CMakeLists.txt` por dependências ausentes.

Problema: Movimentos não ocorrem (MoveCmd retorna `success=false`)
- Causa provável: tentativa de mover para célula ocupada, limite de borda do labirinto, ou falha na lógica de tradução de coordenadas.
- Ações: inspecione `robot_pos` e `target_pos` retornados, verifique o estado no simulador e os logs; confirme que a célula destino é adjacente.

Problema: Erros relacionados a submódulos ou histórico Git (após operações de limpeza de histórico)
- Causa provável: reescrita de histórico remoto ou remoção de arquivos grandes do histórico local.
- Ações: se colaboradores existirem, coordene re-clone ou instruções de reset. Verifique se `backup-main` foi criado localmente antes da reescrita.

10. Recomendações para extensão e manutenção
--------------------------------------------
- Modularizar heurísticas: extraia a função heurística e permita seleção dinâmica (Manhattan, Euclidiana) para facilitar testes em movimentos com diagonais.
- Mapeamento probabilístico: adapte `partial_map_` para armazenar probabilidades e não apenas estados discretos; isso permite incorporar incerteza sensorial.
- Robustez no planejamento: implemente reconciliação entre mapa parcial e mapa global (replanejamento contínuo) com tratamento de falhas de movimento e backoff.
- Telemetria e visualização: exponha tópicos ROS2 com o mapa parcial e o caminho planejado para permitir visualização em ferramentas externas (por exemplo, RViz compatível ou um visualizador HTML local).

Apêndice — Notas de implementação observadas
--------------------------------------------
- O nó de navegação implementa uma distinção clara entre `flat_map_` (mapa completo recebido) e `partial_map_` (mapa incremental). Deve-se tomar cuidado para que as funções que testam navegabilidade (`is_free`) utilizem a representação adequada no contexto de planejamento ou exploração.
- Os temporizadores e callbacks ROS2 no simulador Python empregam locks (`state_lock`) para proteger atualizações de estado quando o serviço de reset é invocado e quando a interface gráfica/processo principal manipula o estado do jogo.

Encerramento
-----------
Este documento objetiva prover uma visão técnica e operacional exaustiva sobre os algoritmos contidos no repositório, as convenções de dados e os procedimentos necessários para construir, executar e estender o sistema. Se desejar, posso gerar uma versão complementada com diagramas de sequência, exemplos de mensagens trocadas (payloads JSON/ROS2) e um conjunto de testes unitários que validem as funções críticas de planejamento e mapeamento.
