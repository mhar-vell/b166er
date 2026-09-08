# Bateria HÍBRIDA por fase + investigação whole-body × keepout — 2026-09-03

Marco: "monta o híbrido por fase e roda a bateria, só não podemos esquecer
de investigar o whole-body e [k]eepout." Modos do YAML: orienta,
aproxima_lateral, atravessa, captura, desengata em **ik**; destrava, libera,
arco1, arco2 em **wb**. Mesma régua por eixo nos dois modos, controlador
Fuzzy servindo até 2 mm. Runs 31–35, seguidas, sem reiniciar o stack.

## Resultado: 5/5 MISSION_OK

| run | lâmina | dur | destrava (wb) | libera (wb) | arco1 | arco2 | desengata (ik) | lingueta ao fim do destrava / máx |
|---|---|---|---|---|---|---|---|---|
| 31 | 29,8° | 152 s | 2,5 s [+8,0/−1,6/−3,0] | 8,6 s | 0,4 s | 0,4 s | 2,3 s | 4,1 / 15,1 mm |
| 32 | 29,0° | 157 s | 3,7 s [+7,9/−1,6/−3,5] | 7,0 s | 0,4 s | 0,4 s | 2,1 s | 5,8 / 17,9 mm |
| 33 | 29,3° | 172 s | 22,6 s [+13,0/−5,6/−1,8] | 13,6 s | 0,4 s | 0,4 s | 2,3 s | 7,5 / 20,1 mm |
| 34 | 29,7° | 152 s | 8,5 s [+15,4/−4,0/+0,9] | 8,3 s | 0,4 s | 0,7 s | 2,4 s | 6,9 / 20,0 mm |
| 35 | 29,6° | 155 s | 2,3 s | 7,3 s | 0,4 s | 0,4 s | 2,1 s | 4,9 / 15,9 mm |

([eixo/prof/alt] = resíduo por eixo ao fechar, mm; tolerâncias do destrava 40/6/4)

Comparação do dia, todas com o mesmo fixture (gatilho) e, salvo a segunda
linha, a mesma régua por eixo:

| modo | runs | OK |
|---|---|---|
| IK iterativa (postura) | 19–20 | 2/2 |
| whole-body, esfera 20 mm | 21–25 | 5/5 |
| whole-body, régua por eixo | 26–30 | 1/5 |
| **híbrido por fase, régua por eixo** | 31–35 | **5/5** |

O Fuzzy moveu o braço em >98 % das amostras das fases wb. A soltura do
gatilho continua acontecendo na `libera` (lingueta ao fim do destrava
4–7,5 mm; máx 15–20 mm na libera).

## whole-body × keepout: o plano NÃO foi o limitador

Sonda da base (`sonda_base.py`, 10 Hz, runs 32–35) nas fases wb:

| fase | base → parede (m) | cmd_v > 5 mm/s | máx cmd_v | J4 máx |
|---|---|---|---|---|
| destrava | 0,644–0,815 | 91 % | 0,010 m/s | **110,0° (limite)** |
| libera | 0,646–0,853 | 92 % | 0,060 m/s | **110,0° (limite)** |
| arco1 | 0,785–0,902 | 72 % | 0,060 m/s | 97,6° |
| arco2 | 0,839–0,908 | 23 % | 0,010 m/s | 96,9° |

- O plano de exclusão está a 0,55 m da parede; a base nunca chegou a menos
  de 0,644 m. `_apply_keepout` só age com d ≤ 0,55 — não agiu. O laser
  (mesmo limiar) tampouco.
- No destrava mais longo (run33, 23,7 s) a base AVANÇOU 0,756 → 0,646 m a
  ~7 mm/s, sinal consistente, no piso `MIN_BASE_LIN` (0,010 m/s), enquanto o
  J4 subiu 90,6° → 110,0° e ficou no batente em 18 % das amostras.
- Diagnóstico: a profundidade converge devagar porque (a) o J4 (punho)
  esgota o curso no batente de 110° e o braço perde autoridade em
  profundidade, e (b) a base, pesada 12× na pseudo-inversa, recebe só o
  piso de 10 mm/s — 11 mm de erro levam 10–20 s. Com o timeout de 40 s e a
  ponta partindo de mais longe (bateria pura, runs 26–30), às vezes não deu.
  No híbrido a IK entrega a profundidade certa na captura e o wb só precisa
  de correções pequenas — daí 5/5.

## Próximos passos sugeridos

- Ponderação da base ciente de batente: quando uma junta do braço está no
  limite, baixar o peso da base (ou subir MIN_BASE_LIN) nessa direção.
- Conferir o limite de 110° do J4 contra o RV-M2 real (URDF vem do
  datasheet) e a postura de partida das fases de contato (J4 já em ~80–95°).
- Critério de `destrava` observável (a soltura ainda é da `libera`).
