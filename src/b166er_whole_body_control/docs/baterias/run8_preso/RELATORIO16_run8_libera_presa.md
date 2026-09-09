# Por que a run8 escorregou — libera puxando com o gatilho travado — 2026-09-09

Marco: "segue com o escorregão da run8". Pendência desde o RELATORIO10
(2026-09-04), que explicou o modo de falha (a ponta deslizou 20 mm ao
longo do eixo da chave e saiu do anel) e criou a guarda de 15 mm e o
indicador de soltura, mas deixou a causa em aberto ("captura 2,9 mm
alta; amostra de 1").

## 1. O que a trajetória da run8 diz (`poses/run8_longe.log`)

| t (s) | fase | eixo | prof (erro) | alt (erro) |
|---|---|---|---|---|
| 32297,9 | destrava, início | +7,2 | +2,0 | −19,1 |
| 32300,0 | destrava | +7,0 | −3,4 | −8,9 |
| 32300,9 | destrava fecha: **desceu 12,5 mm, lingueta 8,1 mm** | | | |
| 32302,1 | libera | +7,0 | +26,2 | +1,6 |
| 32304,2 | libera | +7,0 | +20,3 | +3,9 |
| 32306,3 | libera | +8,4 | **−5,6** | −0,5 |
| 32307,7 | libera fecha | **+27,2** | −3,8 | −1,7 |
| 32309,7 | arco1 fecha | +29,6 | −5,8 | −4,6 |

Três coisas, na ordem: o destrava fechou pela estagnação com a lingueta
em 8,1 mm — no modelo a aba só passa sob o laço a ~13 mm, então o
**gatilho continuou travado**; na libera a altura já estava no alvo
(erro ~0: a ponta estava a −25 mm do olhal ao fim do destrava), então a
libera **não empurrou mais**, só puxou; e o deslize de 19 mm no eixo
aconteceu nos últimos 1,4 s da libera, quando a profundidade **passou do
alvo** (erro −5,6: a ponta foi 35 mm para fora contra os ~31 mm que o
olhal recua com o gatilho preso, até a aba bater no laço).

A leitura mecânica: com o gatilho travado o olhal para no fim de curso;
a libera continua puxando (o ganho Fuzzy cresce com o erro
persistente); o dedo, com o degrau sob um arame parado, encontra o
caminho de menor resistência — escorrega ao longo do arame, que é o
eixo da chave — e sai do anel. A rampa do dedo não participa: fica 90 mm
acima do degrau (comentário no URDF).

## 2. Reproduzir de propósito: duas tentativas, nenhum deslize

`chave_mission.launch task_yaml:=…`, missão a partir de REFINE a 0,88 m:

| YAML | o que muda | resultado |
|---|---|---|
| `task_destrava_curto.yaml` (destrava −12 mm, curso 7/4) | destrava fecha com lingueta 1,4 mm | a libera desce até −25 e destrava sozinha: soltura 53 mm, chave aberta |
| `task_gatilho_travado.yaml` (destrava −20, libera alt −20: não desce mais) | lingueta 5,3 / 3,7 mm ao puxar | libera **luta 22,8 s e 16,2 s** (normal 3–10), mas solta: recuo 57,8 / 55,5 mm, deriva +0,1 / −0,6, chave aberta |

Puxar com o gatilho travado é visível (a libera demora o triplo) mas
não bastou para o dedo sair do anel: em ambas as execuções o puxão
acabou completando o destravamento. O deslize da run8 precisou de algo
a mais que não reproduzimos — a ultrapassagem do alvo em profundidade
sugere um puxão mais forte naquela execução (a bateria de poses rodou
com a régua e os ganhos de 04 Set). Segue como evento de 1 em ~60
missões.

## 3. O que mudou

A causa é reconhecível pela assinatura que a guarda já vê: a deriva no
eixo. Em vez de abortar, a libera passa a **reassentar**, com o mesmo
mecanismo do destrava (RELATORIO14):

- a guarda de deriva (15 mm, 5 amostras) devolve `falha_fase = 'preso'`;
- o MANIPULATE sobe 12 mm, recaptura por IK, **destrava de novo** (alvo
  relativo à captura nova) e **puxa de novo**, até `~reassenta_max` (1)
  vezes. É o que o operador faria: "não soltou — empurra de novo e puxa".
- `~ensaio_forca_preso` (só ensaio) força o primeiro 'preso' depois de
  10 mm de recuo, para exercitar o caminho.

## 4. Validação

Caminho forçado (`bateria_preso.sh`, `~ensaio_forca_preso`; missão a
partir de REFINE), quatro variantes da volta:

| variante | subida | volta | resultado |
|---|---|---|---|
| v1 | 12 mm mirando a altura da captura NA profundidade da captura | — | "não conseguiu subir": a IK subia e entrava ao mesmo tempo, contra o arame |
| v2 | 12 mm na vertical, a partir da ponta | captura em diagonal | "recaptura não fechou": o degrau esbarra no arame (6 mm) na diagonal |
| v3 | 25 mm na vertical | centro do olhal na altura elevada | "não conseguiu voltar por cima do arame": é a barra de cima do anel |
| v4 | 25 mm na vertical | ponto do 'atravessa', depois captura | run1 **OK** (preso a 10 mm de recuo → libera 8,1 s → soltura 56,8 → chave 30,5°); run2 **falhou** (preso a 24 mm de recuo: o anel já tinha recuado com o dedo e o ponto do atravessa, relativo ao olhal ORIGINAL, cai dentro do mecanismo) |

O que a v4 ensina: o reassentamento da libera funciona enquanto o anel
está perto de onde a captura o achou; quando o dedo já puxou o anel
(que é justamente o caso do puxão com o gatilho travado, em que o olhal
recua até ~31 mm), os pontos da volta precisam ser relativos à posição
**atual** do anel — a profundidade da ponta no instante do 'preso' é
uma estimativa dela. Isso não foi implementado nem validado hoje.

**Decisão:** o caminho fica no código, **desligado por padrão**
(`~reassenta_libera` false); a libera presa segue abortando pela guarda
de deriva, como antes. O que fica ligado é o diagnóstico (a flag
`'preso'` no status) e a explicação da run8.

Regressão com o YAML padrão (retry da libera desligado): 2/2 MISSION_OK
(destrava 14,1 e 17,7 mm, lingueta 7,9 e 12,5 mm).

## 5. O que fica

- A run8 tem causa: **puxar com o gatilho travado**. O destrava pode
  fechar pela estagnação com a lingueta aquém do curso que solta (8 mm
  na run8; ~13 no modelo, ~15 na bancada), e se a altura já estiver no
  alvo a libera não empurra mais — só puxa. O olhal para no fim de curso
  e o dedo patina pelo arame.
- Na bancada, o sinal é o mesmo: a libera demora o triplo e a ponta
  deriva no eixo. O operador vê antes do log.
- Próximo passo, se o evento reaparecer: volta relativa ao anel atual
  (usar a profundidade da ponta no 'preso' como profundidade do olhal
  para os pontos do atravessa/captura) e validar com o gancho de ensaio.
