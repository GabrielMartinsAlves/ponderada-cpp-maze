

#!/usr/bin/env python3
try:
    # Quando instalado via ROS2, o pacote cg estará em share/lib/pythonX.X/site-packages
    from cg.main import game
except ImportError:
    # Execução local (diretório fonte)
    from cg.cg.main import game

if __name__ == '__main__':
    game()
