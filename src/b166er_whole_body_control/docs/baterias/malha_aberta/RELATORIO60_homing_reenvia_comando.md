# RELATORIO60 — o homing reenvia o comando quando o braço não se mexe — 07 Nov 2026

Marco: "pode seguir com o reenvio do comando de homing". Branch `fix/homing-reenvia-comando` (commit 6b4e17c5), a partir
da main até a PR #125. Proposta do RELATORIO59.

## O problema (execução 3 da diagonal na bateria `poses_nov`)

O comando de homing (`/b166er/arm_home_cmd`) é publicado uma vez só, como mensagem "latched". Nessa execução, o registro
do servo do braço mostra uma falha de conexão TCP com o nó da missão logo depois de a conexão do comando ser aberta. O
servo nunca recebeu o comando, nenhuma junta se mexeu, e a missão esperou os 150 s do limite e abortou. A mesma coisa
pode acontecer no robô real, que usa os mesmos nós.

## A mudança

Durante a espera do homing, se nenhuma junta se mexer mais de 1° (`~homing_mexeu_deg`) em 10 s (`~homing_reenvio_s`), a
missão recria o publicador, o que força uma conexão nova com o servo, espera até 3 s pela conexão e reenvia o comando.
Faz isso no máximo 3 vezes (`~homing_reenvios`). No homing normal, a primeira junta, o J4, já se move em cerca de 5 s, e
o reenvio não dispara. Há também um parâmetro só para teste, `~ensaio_homing_perde_cmd`, que pula o primeiro envio para
exercitar o reenvio.

## Provas (pasta `homing_reenvio`)

| missão | comando inicial | reenvio | homing | resultado |
|---|---|---|---|---|
| ensaio (`ensaio_homing_perde_cmd`) | não enviado | 1 de 3, aos 10 s | concluído, 5 switches | MISSION_OK |
| normal | enviado | não disparou | concluído em 79 s | MISSION_OK |

No ensaio, a missão ficou 10 s com o braço parado, reenviou o comando, o homing correu normalmente e a missão terminou
com sucesso. Na missão normal, nada mudou.

## Conclusão

A falha de comunicação que derrubou a execução 3 da diagonal passa a custar 10 s em vez da missão. A mudança só age
quando o braço não se mexe depois do comando. **Recomendo mergear.** Não rodei uma bateria completa, porque o código
alterado só atua na espera do homing, e a missão normal mostrou que ele fica inerte quando o braço responde.
