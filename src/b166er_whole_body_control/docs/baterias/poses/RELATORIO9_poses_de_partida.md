# Bateria de poses de partida — 2026-09-04

Marco: "vamos seguir com a bateria de poses" (plano anotado antes: depois que a
abertura funcionar, testar de VÁRIAS posições; desconfiar de número calibrado
numa pose só). A chave não muda de lugar no laboratório, então a variação é do
robô. Chave em (0, 3,0) na parede y = 3; pose padrão da missão (0, 1, 0°).
Híbrido por fase, mesma régua, `phase_timeout 60`. 8 poses × 3.
Scripts: `bateria_poses.sh` → `run_once_pose.sh` (reset com `--x/--y/--yaw`,
commit 747fdf4, + `checa_pose.py`: repete o reset se ficar > 5° ou 5 cm fora).

## Resultado: 20/24

| pose | x, y (m), yaw | abriu | lâmina | erro do yaw do SEARCH (°) | falha |
|---|---|---|---|---|---|
| ref | 0, 1,0, 0° | 3/3 | 27,1–29,6° | −6, −11, −13 | — |
| perto (1,5 m) | 0, 1,5, 0° | 3/3 | 28,8–31,1° | −2, −6, +16 | — |
| longe (2,5 m) | 0, 0,5, 0° | 2/3 | 29,4–30,5° | −14, −15, −16 | run8: gatilho não soltou (lâmina 8,9°) |
| esq | −0,5, 1,0, 0° | 2/3 | 30,3–31,8° | **−27**, +6, +8 | run10: perdeu a tag no APPROACH |
| dir | +0,5, 1,0, 0° | 3/3 | 28,5–29,2° | −1, −2, −4 | — |
| frente (tag já no quadro) | 0, 1,0, 90° | **0/3** | — | **−32, −32, −32** | runs 16–18: perdeu a tag no APPROACH |
| costas | 0, 1,0, 180° | 3/3 | 29,3–30,0° | −11, −14, −20 | — |
| diag | 0,5, 1,5, −45° | 3/3 | 28,5–30,6° | −2, −4, −2 | — |

Erro do yaw = estimativa do SEARCH − 180° (a parede real está em yaw π).
Tabela completa por execução em `tabela.txt` (`analisa_poses.py`).

## Leitura

1. **A manipulação não depende da pose de partida.** Todas as 20 execuções que
   chegaram ao REFINE abriram, com lâmina 27–32°, o mesmo intervalo das
   baterias anteriores; o REFINE fechou a parede em ±1,6° em todas (coluna
   yawREF). Duração 138–179 s; a pose "longe" custa +25 s de aproximação.
2. **As 4 falhas de percepção são todas o viés de yaw do SEARCH**, e
   exatamente as execuções com |erro| ≥ 27°: o standoff é calculado com o yaw
   errado, na etapa intermediária a tag sai do quadro (remedida com 0
   amostras: "perdi a tag na etapa intermediária") e a missão aborta antes de
   manipular. Com |erro| ≤ 20° a remedida corrige e nada acontece.
3. **O viés não é ruído: é geometria de vista.** Na pose "frente" o robô já
   nasce olhando a tag centrada e amostra sem girar: 148,0 / 148,4 / 148,0°,
   três vezes o mesmo número. É a ambiguidade de pose do alvo planar visto de
   frente (as duas soluções são simétricas em torno do eixo óptico e a
   mediana cai sempre na mesma). Nas poses em que o robô GIRA até a tag
   entrar a raio ≤ 0,5 do quadro, a vista é oblíqua e o erro cai (dir e diag:
   1–4°). A pose esq deu −27° numa de três com 12/25 amostras descartadas —
   caso intermediário.
4. **run8 (longe): o gatilho não soltou.** Destrava aceito por estagnação
   (ponta desceu 12,6 mm ≥ 10), lingueta em 8,1 mm; a missão avisou
   "o gatilho pode não ter soltado"; libera e arcos fecharam na régua e a
   lâmina parou em 8,9°. Mas a run23 abriu com lingueta 8,4 mm — a soltura
   acontece no libera, e o que diferencia run8 não está nos logs de fase
   (candidatos: ponta escapou do olhal no arco; captura 2,9 mm alta). Fica
   como 1 em 20 na manipulação, causa não fechada.

## O que isso muda

- A decisão "aceita e documenta o yaw do SEARCH, não vale a segunda vista"
  foi tomada com viés de −9° na pose padrão. De outras partidas o viés chega
  a −32° e derruba a missão (4/24). Duas correções baratas que NÃO são uma
  segunda vista:
  a) **amostrar sempre em vista oblíqua**: se a tag já está centrada, girar
     até ela ficar a raio 0,3–0,5 do quadro antes de coletar (é o que as
     outras poses fazem por acaso);
  b) **recuperar a tag na etapa intermediária** em vez de abortar: girar no
     lugar até reencontrá-la e remedir (a remedida já existe, só não tem a
     quem perguntar quando o quadro está vazio).
- O critério observável do destrava (estagnação ≥ 10 mm) deixou passar a
  run8; a run23 mostra que a lingueta no fim do destrava não prevê a
  abertura. O indicador que falta é da fase libera (a soltura), não do
  destrava.
- Para o artigo (V-A): partida não afeta a manipulação (20/20); afeta a
  percepção de longe (viés de yaw por geometria de vista, 4/24) — entra
  como limitação com número, e como correção se o Marco aprovar (a) e (b).

## Correções (2026-09-04, Marco: "implementa a vista oblíqua e a recuperação da tag")

Commit 9776759 em `chave_mission.py`:
- `~search_raio_min` (0,35): a coleta do SEARCH só acontece com a tag na
  janela [0,35, 0,5] do quadro (~30–43° fora do eixo); se ela já está mais
  para dentro, a base gira com omega fina até empurrá-la para a janela.
- `_recupera_tag` + `~recupera_timeout` (30 s): quando a remedida da etapa
  intermediária fica sem amostras, gira no lugar para o lado da parede
  estimada até a tag voltar ao quadro, leva-a à janela e remede; só então
  aborta se continuar sem amostras.

### Revalidação (`bateria_fix_busca.sh`, `tabela_fix_busca.txt`): 8/8

| pose | antes | depois | erro do yaw do SEARCH depois |
|---|---|---|---|
| frente (0, 1,0, 90°) | 0/3 | **3/3** | 0,1 / 0,8 / 3,4° (antes −32 ×3) |
| esq (−0,5, 1,0, 0°) | 2/3 | **3/3** | −24 / +6 / +5° |
| ref (0, 1,0, 0°) | 3/3 | 2/2 | −7 / −14° |

Na pose frente o log mostra "tag FRONTAL a raio 0,18 — girando até >= 0,35"
e a estimativa sai certa. A recuperação não precisou disparar em nenhuma
das 8 (nenhuma remedida ficou sem amostras).

### Testes forçados da recuperação — o que ficou e o que não ficou provado

1. `teste_recuperacao.sh` (janela 0–0,15, pose frente): 2/2 OK com yaw
   −178,9° e 176,6°. **A vista frontal NÃO reproduziu o viés de −32°.** O
   que está estabelecido é empírico: sem girar, a raio ~0,18, o SEARCH deu
   148° três vezes; com a janela oblíqua deu certo 3/3. A explicação
   "frontal = ambíguo" que dei no relato de 10:31 não é suficiente — pode
   ser a posição exata da tag no quadro naquele raio, o lado do dither,
   ou outra coisa; não investigado além disso.
2. `teste_recuperacao2.sh` (lateral 1,8 m na intermediária, tag a ~2,3 m e
   oblíqua): a recuperação EXECUTOU como desenhada na run2 — "sem amostras
   — recuperando: girando à esquerda", "tag FRONTAL a raio 0,08 — girando
   até >= 0,35", "reencontrada em 9,6 s — remedindo" — mas a remedida
   recuperada ficou com 0 amostras (tag de 132 mm a 2,3 m, além do alcance
   confiável) e abortou. A run1 mediu de 54° uma parede errada e o
   standoff final caiu NA parede (REFINE 0 amostras).
3. `teste_recuperacao3.sh` (coarse 0,95 + lateral 1,2): a remedida a ~52°
   NÃO falhou (10 amostras, yaw −174°), e `standoff_lateral` forçado
   quebrou o standoff da manipulação (IK fora de alcance) — teste inválido
   para o fim, mas mostra que a rejeição na periferia não é garantida a
   raio ~0,6.

**Conclusão honesta:** a busca oblíqua corrige as 4 falhas observadas
(3/3 + 3/3 nas poses que falhavam). A recuperação está implementada,
roda o caminho inteiro (gira, reencontra, centra, remede) e só age onde a
missão já ia abortar, mas um resgate completo de ponta a ponta não foi
demonstrado porque não consegui reproduzir artificialmente a condição
das 4 falhas sem quebrar outra coisa. Fica como rede de segurança com
custo zero, a ser validada quando a condição real reaparecer.
