# Guia de contribuição

Esse é o guia de contibuição do repositorio da Equipe Quimera #21222, feito para a otimização do nosso trabalho e documentação do desenvolvimento do software.

Para garantir que o codigo funcione dentro da arena, nao quebre entre as partidas e mantenha um historico limpo.

---

## Padrão das mensagens de commit

as mensagens de commit daqui pra frente devem seguir tal padrão:

text
tipo(escopo): descrição curta da alteração



### Tipos de commits (tipo)

O tipo define *O que* voce alterou no ecossistema do robo:

| Tipo | Descricao | Exemplo Pratico |

* | auto | Codigo exclusivo do periodo *Autonomo* (poses, paths) | auto(visao): adiciona tracking de apriltag |
* | teleop | Codigo do periodo *Controlado* (controles, funções dos botões) | teleop(chassi): adiciona deadzone no joystick |
* | hardware | Configuração ou inversão de motores/sensores/servos | hardware(intake): adiciona botão que inverte o intake |
* | tune | Ajuste de constantes e calibração fina | tune(pid): adiciona PIDF na flywheel |
* | fix | Correção de bugs de logica ou bugs que travavam o robo | fix(chassi): corrige direção das rodas |
* | docs | Alterações na documentação, comentarios ou arquivos md | docs(readme): atualiza o readme |


### Escopos Comuns (escopo)

O escopo (dentro dos parenteses) diz *onde* (em qual subsistema do robo ou classe) a mudança aconteceu:

* *chassi*: Rodas Mecanum, tração, motores de movimentação.
* *intake*: Sistema de coleta.
* *outtake*: Slides, Shooters, braço de deposito.
* *visao*: Cameras, AprilTags.
* *pid*: Controladores PIDF de qualquer mecanismo.
* *util*: Contas matematicas complexas, telemetria ou estados.


##  Exemplos do que FAZER e NÃO FAZER

*Errado:*
* git commit -m "mudei o codigo" (Não diz nada sobre a alteração)
* git commit -m "arrumei o motor do teleop" (Sem padrão, confuso)
* git commit -m "AUTONOMO FUNCIONANDO!!!!" (Polui o historico)

*Certo:*
* git commit -m "hardware(chassi): configura encoders nos motores traseiros"
* git commit -m "teleop(intake): mapeia botao A para ligar o intake"
* git commit -m "tune(outtake): ajusta os controladores PIDF para compensar o disparo"
* git commit -m "auto(chassi): ajusta path para nao passar os 30s"

---

## Fluxo de Trabalho

Para evitar que dois programadores sobrescrevam o codigo um do outro, siga este fluxo:

1. *Sempre de git pull* antes de começar a codar no note da equipe.
2. *Crie uma branch propria* se for testar uma logica muito complexa (git checkout -b <nome da branch nova(sas)>).
3. *Testou e funcionou no robo fisico?* Faça o commit conforme os topicos acima e de git push.