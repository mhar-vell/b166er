"""Shim mínimo do módulo stdlib `imp` (removido de vez no Python 3.12,
depreciado desde o 3.4). Implementa só o suficiente para o
`rosserial_python.SerialClient.load_pkg_module` (ROS Noetic, pacote
robostack-noetic `ros-noetic-rosserial-python` np2py312hbad1cee_24) rodar
sem crashar em py3.12 — é o único uso de `imp` no pacote, e só checa se o
módulo existe (`except ImportError: roslib.load_manifest(...)`).

Ativar: colocar este diretório na FRENTE do PYTHONPATH antes de subir
qualquer nó que dependa de rosserial (Arduino_1/2/3.py):

    export PYTHONPATH="$HOME/b166er/setup/py312_compat:$PYTHONPATH"

Sem isso, os nós morrem na importação com:
    ModuleNotFoundError: No module named 'imp'

Ver ROADMAP.md (Fase 4, validação py312 em hardware real) e a memória
project_py312_rosserial_imp — mesma classe de problema do PR #16
(py312 + gazebo-ros 2.9.3), desta vez sem build corrigido disponível no
canal robostack-noetic (só existe build py311 para rosserial-python lá).
"""

import importlib.util


def find_module(name, path=None):
    if path is not None:
        raise ImportError(
            f"py312_compat.imp.find_module: busca com 'path' explícito "
            f"não suportada (name={name!r})"
        )
    spec = importlib.util.find_spec(name)
    if spec is None or spec.origin is None:
        raise ImportError(f"No module named {name!r}")
    return (None, spec.origin, None)
