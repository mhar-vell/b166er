# Baterias de simulação da chave seccionadora — relatórios e ferramentas

Relatórios, scripts de bateria, analisadores e dados pequenos (índices,
tabelas, JSONL, CSV < 300 kB) das baterias de 02–08 Set 2026. Viviam na
pasta da sessão do assistente; entraram no pacote em 08 Set 2026 por
decisão do Marco ("pode mandar o /docs"). Logs de rosout, CSVs grandes de
sonda e imagens de origem externa ficaram de fora.

Todos os números citados no artigo (Seções IV-D, IV-E e V-A) saem daqui.

| # | relatório | pasta | o que mede |
|---|---|---|---|
| 1 | `bateria_wb/RELATORIO.md` | bateria_wb | whole-body puro, critério esférico 20 mm: 5/5 (artefato do critério) |
| 2 | `bateria_wb2/RELATORIO2.md` | bateria_wb2 | whole-body puro, régua por eixo: 1/5; destrava fecha sem soltar |
| 3 | `bateria_hib/RELATORIO3.md` | bateria_hib | híbrido por fase (IK geometria, Fuzzy contato): 5/5 |
| 4 | `bateria_pond/RELATORIO4.md` | bateria_pond | ponderação ciente de batente (Chan–Dubey): 4/5, base nunca sai do piso |
| 5 | `bateria_obs18/RELATORIO5.md` | bateria_obs18 | critério observável do destrava (descida ≥ 18 mm): punho colapsa |
| 6 | `RELATORIO6_item2_profundidade.md` | bateria_estagna* | profundidade no whole-body/keepout; estagnação curta 5/5 punho íntegro |
| 7 | `bateria_fixos/RELATORIO7_linha_de_base.md` | bateria_fixos | ganhos fixos × Fuzzy nas fases de contato: 5/5 todos |
| 8 | `aproximacao/RELATORIO8_aproximacao_de_longe.md` | aproximacao | aproximação de longe (1,9 m): ganho importa; chatter da manobra; piso 0,90 |
| 9 | `poses/RELATORIO9_poses_de_partida.md` | poses | 8 poses × 3: 20/24; busca oblíqua + recuperação → 8/8 |
| 10 | `soltura/RELATORIO10_indicador_de_soltura.md` | soltura | run8 explicada (deriva de eixo); guarda 15 mm; indicador de soltura 5/5 |
| 11 | `punho/RELATORIO11_punho_limitado.md` | punho | J4 limitado a 4,2 N·m pela spec do RV-M2: 6/6; libera satura o punho (48 %), destrava não |
| 12 | `punho/RELATORIO12_punho_pior_caso.md` | punho/pior_caso | pior caso J4 2,9 N·m: 5/5; libera satura 57–65 % e cede 7–22° sem perder o anel |
| 13 | `teto_postura/RELATORIO13_teto_por_postura.md` | teto_postura | teto de velocidade pela margem de tombamento da postura medida: frenagem seca 0,05–0,07 → 0,02 rad; escala conjunta base+braço; queda pós-execução é do reset; adendo: teto também no drive da missão, angular pela margem lateral |
| 14 | `destrava_reassenta/RELATORIO14_destrava_reassenta.md` | destrava_reassenta | destrava que estagna curto (ponta sai do arame): captura descentrada não reproduz (8/8); estagnação curta em 4 s + reassentar (sobe, recaptura, desce) 2/2 |
| 15 | `abort_saida/RELATORIO15_abort_saida_em_degraus.md` | abort_saida | saída do anel no ABORT/RETRACT em degraus (sobe, eixo, fora): a saída ruim do E5 vinha da postura colapsada por 40 s de empurrão; degraus fecham em 1 iteração 2/2 |
| 16 | `run8_preso/RELATORIO16_run8_libera_presa.md` | run8_preso | por que a run8 escorregou: libera puxando com o gatilho travado (destrava fechou com lingueta 8 mm); reprodução 0/2 deslizes; reassentamento da libera exercitado (v4 1/2) e desligado por padrão |
| 17 | `nuc_smoke/RELATORIO17_nuc_smoke.md` | nuc_smoke | stack inteiro no NUC (py3.12 reprovisionado): 1 missão headless, chave aberta 31°; RETURN orbitava o alvo (estimador preso na IK abria buracos na pose, missão passou cega) → IK estagna cedo, retry só p/ resíduo grande, navegação para com pose velha e realinha: 89,5 → 10,7 s; adendo 2: IK do estimador limitada a 40 it + base sincronizada com o T265 pelo stamp → 8,6 s, buracos ≤ 0,07 s (= shiroi); CPU 94 %, RTF 0,40; câmera exige DISPLAY |

## Como reproduzir uma bateria

1. `scripts/sim_stack.sh preflight` (stack de pé, fixture presente, nada pendente).
2. `poses/run_once_pose.sh N OUT X Y YAW "<args do launch>"` roda UMA missão a
   partir da pose dada (reset com `--x/--y/--yaw`, confere a pose, lança
   `chave_mission.launch`, espera `resultado: MISSION_*`).
3. Os `bateria_*.sh` encadeiam execuções e gravam `indice.csv`; os
   `analisa_*.py` tabulam a partir dos logs.

Caminhos absolutos dentro dos scripts apontam para a pasta da sessão de
origem (`~/.claude/jobs/…` e `~/.claude/projects/…/sessoes/…`); ajuste
antes de reutilizar.
