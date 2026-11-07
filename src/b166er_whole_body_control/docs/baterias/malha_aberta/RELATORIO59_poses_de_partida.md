# RELATORIO59 — validação partindo de várias posições — 07 Nov 2026

Marco: "pode seguir com a validação partindo de várias posições". Bateria `poses_nov` na main 9c8aabf9 (até a PR #125),
sem viés de parede: as mesmas 8 poses de partida do RELATORIO9 (04 Set) e da bateria `poses_out` (16 Out), com 3 execuções
cada. As baterias de viés partem sempre da pose "esq".

## Resultado: 22/24

| pose | x | y | yaw | poses_nov (07 Nov) | poses_out (16 Out) |
|---|---|---|---|---|---|
| referência | 0,0 | 1,0 | 0° | 3/3 | 2/2 |
| perto | 0,0 | 1,5 | 0° | 3/3 | 2/2 |
| longe | 0,0 | 0,5 | 0° | 3/3 | 2/2 |
| esquerda | −0,5 | 1,0 | 0° | 3/3 | 1/2 |
| direita | 0,5 | 1,0 | 0° | 3/3 | 1/2 |
| de frente | 0,0 | 1,0 | 90° | 3/3 | 2/2 |
| de costas | 0,0 | 1,0 | 180° | 2/3 | 1/2 |
| diagonal | 0,5 | 1,5 | −45° | 2/3 | 1/2 |
| **total** | | | | **22/24** | **12/16** |

- **Seis das oito poses fecharam 3/3,** incluindo esquerda e direita, que falhavam em 16 Out.
- **Percepção:** refinamento com tag em 23 das 24 execuções e nenhuma aproximação sem tag. A busca do ângulo pelo laser disparou em 8.

## As duas falhas

1. **De costas, execução 1:** o destrava chegou ao limite de descida, a 31,4 mm, e a libera teve uma subida de 14,7 mm, seguida de reassentamento. É uma falha de manipulação, a mesma que aparece de vez em quando nas baterias de viés.
2. **Diagonal, execução 3:** o homing não concluiu em 150 s, e o braço não se mexeu nenhuma vez. O reset deixou o robô na pose certa. No registro do servo do braço há uma falha de conexão TCP com o nó da missão, logo depois de a conexão do comando de homing ser aberta. O comando é publicado uma vez só, como mensagem "latched", e nunca chegou ao servo. É uma falha de comunicação do ROS, independente da pose. **Proposta:** se nenhuma junta se mexer nos primeiros 10 s do homing, a missão reenvia o comando. Isso vale também no robô real, que usa os mesmos nós.

## Conclusão

A missão não depende mais da pose de partida usada nas baterias de viés: as oito poses deram 22/24, contra 12/16 em
16 Out. A única falha ligada à manipulação aconteceu na pose de costas, e a outra foi de comunicação entre nós. O
reenvio do comando de homing é uma correção pequena, a fazer se o Marco quiser.
