# Regressão da missão completa — 2026-09-08 (pose ref, híbrido)

`regressao_missao.sh OUT N:true|false …` (run_once_pose.sh, x=0 y=1 yaw=0).
Logs completos das execuções ficaram na pasta da sessão.

| run | regra | resultado | lâmina | soltura (recuo / deriva) | pose de manipulação (x, y, yaw) | duração |
|---|---|---|---|---|---|---|
| 1 | com | OK | 29,4° | 57,9 / −0,7 mm | 0,19, 2,25, 1,55 | 153 s |
| 2 | com | **ABORT** em "atravessa" (IK: prof −6,9 mm × tol 6, 5 it.) | — | — | **0,21**, 2,25, 1,58 | — |
| 3 | com | OK | 28,5° | 55,6 / −1,5 | 0,18, 2,25, 1,54 | 160 s |
| 4 | sem | OK | 30,1° | 59,2 / −0,8 | 0,19, 2,25, 1,56 | 154 s |
| 5 | com | OK | 30,2° | 46,5 / −0,3 | 0,19, 2,25, 1,56 | 144 s |
| 6 | sem | OK | 29,8° | 50,9 / +1,3 | 0,20, 2,25, 1,57 | 161 s |
| 7 | com | OK | 30,4° | 48,7 / −0,2 | 0,19, 2,25, 1,56 | 148 s |
