# 🤖 FRC Swerve Drive - SDS MK4i (L2) + SparkMax + CANcoder + Pigeon 2

**Desenvolvido pela equipe FRC #9219 Nine Tails 🦊 — Resende/RJ, Brasil**

Este repositório contém o código base em **Java (Command-Based)** para controle de um chassi Swerve utilizando módulos **SDS MK4i Nível L2**, motores Brushless **REV NEO** com controladores **SparkMax**, encoders absolutos **CTRE CANcoder** e giroscópio **CTRE Pigeon 2**.

O projeto está atualizado para as APIs mais recentes da **WPILib**, **REVLib (2025/2026)** (`SparkMaxConfig` / `setSetpoint`) e **CTRE Phoenix 6**.

## 🛠️ Especificações de Hardware e Bibliotecas

* **Módulos Swerve:** Swerve Drive Specialties (SDS) MK4i - Relação **L2**
  * Relação de Tração (*Drive*): `6.75:1`
  * Relação de Direção (*Steer/Angle*): `150/7:1` (~`21.43:1`) — *Invertido fisicamente no MK4i*
  * Diâmetro da Roda: `4 polegadas` (`0.1016 metros`)
* **Motores:** 8x REV Robotics NEO Brushless
* **Controladores de Motor:** 8x REV SparkMax
  * Motores de Tração (*Drive*): Configurados em modo **Coast** (`IdleMode.kCoast`)
  * Motores de Direção/Rotação (*Angle*): Configurados em modo **Brake** (`IdleMode.kBrake`)
* **Encoders Absolutos:** 4x CTRE CANcoder
* **Giroscópio (IMU):** 1x CTRE Pigeon 2.0
* **Controle:** Xbox Controller (Porta USB 0)

### Dependências (Vendor Libraries)

Certifique-se de ter instalado no seu projeto WPILib (via `Manage Vendor Libraries`):

1. **REVLib** (Versão 2025/2026+)
2. **CTRE Phoenix 6**

## 📍 Passo 1: Montagem do Pigeon 2 e Modos do SparkMax (Coast / Brake)

Antes de calibrar ou ligar o robô, verifique dois pontos fundamentais de montagem e configuração dos controladores:

### 1. Posicionamento e Orientação do Giroscópio (Pigeon 2)
* **Localização no Chassi:** O Pigeon 2 deve ser fixado o mais próximo possível do **centro geométrico de rotação do robô**, em uma superfície plana e rígida (evite áreas com alta vibração). Isso evita que acelerações centrífugas em curvas rápidas afetem as leituras inerciais.
* **Orientação (Frente do Robô):** O eixo **+X** desenhado na carcaça do Pigeon 2 deve apontar exatamente para a **frente do robô**, com a face superior voltada para cima (+Z para cima).
  * *Nota:* Caso o seu Pigeon 2 precise ser instalado rotacionado (ex: 90° ou 180°) ou de cabeça para baixo devido ao layout da elétrica, será necessário configurar o `MountPose` pelo **Phoenix Tuner X** para corrigir os eixos.

### 2. Modos de Operação no SparkMax (Coast vs. Brake)
No código (`SwerveModule.java`) e/ou via **REV Hardware Client**, os motores possuem comportamentos distintos quando não recebem comando:
* **Motores de Tração (*Drive*) em `Coast` (`IdleMode.kCoast`):** Permite que o robô desacelere suavemente quando o piloto solta o analógico, evitando tombamentos bruscos (*tipping*), reduzindo o aquecimento dos NEOs de tração e facilitando empurrar o robô em bancada/pit quando desabilitado.
* **Motores de Rotação/Direção (*Angle*) em `Brake` (`IdleMode.kBrake`):** Mantém o ângulo da roda rigidamente travado na direção comandada pelo PID, impedindo que o atrito do tapete ou impactos de outros robôs desviem o alinhamento das rodas durante a locomoção.

```java
// Exemplo em SwerveModule.java:
driveConfig.idleMode(IdleMode.kCoast); // Tração livre ao soltar o controle
angleConfig.idleMode(IdleMode.kBrake); // Direção firme e travada no ângulo
```

## 🧭 Passo 2: Calibração dos Offsets dos CANcoders (Phoenix Tuner X)

Antes de enviar o código para o robô, é obrigatório alinhar fisicamente as rodas e gravar o ponto zero (*Magnet Offset*) diretamente na memória de cada CANcoder usando o **Phoenix Tuner X**.

1. **Coloque o robô em cima de blocos (cavalete)** para que as rodas girem livremente.
2. **Alinhe as 4 rodas perfeitamente retas** apontando para a frente do chassi.
   * ⚠️ **MUITO IMPORTANTE (Lado do Bevel Gear):** Certifique-se de que a engrenagem cônica (*bevel gear*) de **todas as 4 rodas esteja apontando para o mesmo lado** (por exemplo, todas as engrenagens cônicas voltadas para o lado direito do robô ou todas para dentro). Use uma régua longa ou perfil de alumínio encostado nas laterais de duas rodas para garantir que estão 100% paralelas.
3. Conecte-se ao robô e abra o **Phoenix Tuner X**.
4. Vá na aba **Devices** e selecione o primeiro **CANcoder**.
5. Verifique se o sentido positivo de rotação do CANcoder é **Counter-Clockwise (CCW / Sentido Anti-Horário)** olhando o robô por cima (`SensorDirection = CounterClockwise_Positive`).
6. Com a roda perfeitamente alinhada, clique no botão **"Zero CANcoder"** (ou vá em *Config* -> *MagnetSensor* -> copie o valor inverso da posição absoluta atual para o `MagnetOffset` e clique em **Apply/Save**).
7. Confirme na aba de plotagem/status se a leitura de `Absolute Position` agora marca **`0.000 rotations`**.
8. Repita o processo para os outros 3 módulos.

## ⚙️ Passo 3: Configurando IDs e Dimensões no Código (`Constants.java`)

Abra o arquivo `src/main/java/frc/robot/Constants.java` e ajuste as constantes de acordo com a elétrica e a mecânica do seu robô:

### 1. IDs da Rede CAN

Substitua os números abaixo pelos IDs configurados nos seus SparkMax (via *REV Hardware Client*) e nos dispositivos CTRE (via *Phoenix Tuner X*):

```java
public static final int PIGEON_ID = 13;

public static final int FL_DRIVE_ID = 1, FL_ANGLE_ID = 2, FL_CANCODER_ID = 9;  // Frente Esquerda
public static final int FR_DRIVE_ID = 3, FR_ANGLE_ID = 4, FR_CANCODER_ID = 10; // Frente Direita
public static final int BL_DRIVE_ID = 5, BL_ANGLE_ID = 6, BL_CANCODER_ID = 11; // Trás Esquerda
public static final int BR_DRIVE_ID = 7, BR_ANGLE_ID = 8, BR_CANCODER_ID = 12; // Trás Direita
```

### 2. Dimensões do Chassi (`TRACK_WIDTH` e `WHEEL_BASE`)

A cinemática do Swerve precisa da distância **de centro a centro das rodas** (em metros) para calcular os ângulos de curva e rotação:

* **`TRACK_WIDTH` (Largura / Eixo Y):** Distância entre o centro da roda da esquerda e o centro da roda da direita.
* **`WHEEL_BASE` (Comprimento / Eixo X):** Distância entre o centro da roda da frente e o centro da roda de trás.

> 💡 **Dica para chassi MK4i:** Nos módulos MK4i montados nos cantos do frame, o centro da roda fica recuado `2.625 polegadas` (`6,67 cm`) de cada borda externa. Se você mediu o tamanho total externo da estrutura de alumínio (ex: `74,5 cm` de largura por `65,5 cm` de comprimento), subtraia `0.13335 metros` (`13,34 cm`) de cada medida, ou meça diretamente de um parafuso central do CANcoder até o outro.

```java
// Exemplo em metros (0.745m = 74,5 cm | 0.655m = 65,5 cm)
public static final double TRACK_WIDTH = 0.745; 
public static final double WHEEL_BASE = 0.655;
```

## 🔄 Passo 4: Verificando a Inversão dos Motores de Tração (`SwerveSubsystem.java`)

No arquivo `SwerveSubsystem.java`, cada módulo recebe um parâmetro booleano `driveInverted` no construtor:

```java
private final SwerveModule frontLeft = new SwerveModule(
    SwerveConstants.FL_DRIVE_ID, SwerveConstants.FL_ANGLE_ID, SwerveConstants.FL_CANCODER_ID, false);
private final SwerveModule frontRight = new SwerveModule(
    SwerveConstants.FR_DRIVE_ID, SwerveConstants.FR_ANGLE_ID, SwerveConstants.FR_CANCODER_ID, true);
```

* Se você alinhou as engrenagens cônicas (*Bevel Gears*) **para dentro** (esquerda apontando para a direita, e direita apontando para a esquerda), mantenha um lado `false` e o outro `true`.
* Se você alinhou todas as 4 engrenagens cônicas apontando para o **mesmo lado do chassi** (ex: todas para a direita), coloque o mesmo valor booleano (`false` ou `true`) nos 4 módulos.
* **Teste no cavalete:** Empurre o analógico esquerdo levemente para frente. Todas as 4 rodas devem apontar para frente e girar para frente. Se alguma roda girar para trás, basta inverter o booleano (`true` <-> `false`) daquele módulo específico.

## 🎮 Controles (Xbox Controller)

| Comando no Controle | Ação no Robô | 
| ----- | ----- | 
| **Analógico Esquerdo (Eixo Y)** | Translação Frente / Trás | 
| **Analógico Esquerdo (Eixo X)** | Translação Esquerda / Direita (*Strafe*) | 
| **Analógico Direito (Eixo X)** | Rotação no próprio eixo (Giro) | 
| **Botão Y** | Zera a referência de frente do giroscópio Pigeon 2 (*Zero Heading*) | 

## 🔧 Solução de Problemas Comuns (Troubleshooting)

### 1. O robô funciona perfeitamente suspenso no ar, mas "se perde" ou puxa quando colocado no chão

* **FeedForward de Velocidade em Volts (REVLib 2026):** Na API atualizada da REVLib, o ganho `feedForward.kV` opera em **Volts** (0 a 12V). Certifique-se de que em `SwerveModule.java` o cálculo esteja multiplicado por `12.0`:
  ```java
  driveConfig.closedLoop.feedForward.kV(12.0 / SwerveConstants.MAX_SPEED_METERS_PER_SECOND);
  ```
* **Sincronização do CANcoder no Boot:** Para evitar que o SparkMax leia `0.0` antes do barramento CAN inicializar, o método `resetToAbsolute()` aguarda um pacote válido do CANcoder usando `.waitForUpdate(0.1)`.
* **Torque de Direção no Tapete:** Se a roda demorar para virar com o peso do robô, aumente o ganho `P` do `angleConfig` (ex: de `0.5` para `1.5`) e o limite de corrente para `30A`.

### 2. Uma das rodas aponta para a direção certa, mas só gira se receber um "tranco" manual

* Isso indica **perda de uma das 3 fases** (fios Branco, Preto ou Vermelho) ou **falha no cabo/sensor Hall interno** do motor Brushless NEO daquele módulo.
* Verifique os conectores de potência, substitua o cabo flat preto do encoder do SparkMax ou troque o motor NEO defeituoso.

## 🦊 Créditos

Este projeto e documentação foram desenvolvidos e testados pela equipe de robótica **Nine Tails #9219** (*FIRST* Robotics Competition), sediada em **Resende/RJ - Brasil**.

Se este repositório ajudou a sua equipe a colocar o Swerve para rodar, deixe uma ⭐ no projeto!
