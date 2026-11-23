#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
// =====================================================
// Configuração
// =====================================================
#define ULTRASONIC_SIG 20
#define IV_ENTRADA 0
#define IV_SAIDA 1
#define SERVO_ENTRADA 18
#define SERVO_SAIDA 19
#define LED_VERDE 22
#define LED_VERMELHO 23
#define BUZZER 14
#define LCD_SDA 6
#define LCD_SCL 7

// KS0061: LCD de 16 colunas e 2 linhas, interface I2C.
const int ENDERECO_LCD = 0x27;
LiquidCrystal_I2C lcd(ENDERECO_LCD, 16, 2);
bool lcdOK = false;

const int CAPACIDADE = 5;
const int OCUPACAO_INICIAL = 0; // arrancar com o parque vazio; ajustar se houver carros
int ocupados = OCUPACAO_INICIAL;

const int NIVEL_IV_ATIVO = LOW;

const float ALTURA_SENSOR_CM = 10.0; 
const float ALTURA_MAX_CM = 5.0;
const float ALTURA_MIN_DETECAO_CM = 1.5;  

const unsigned long INTERVALO_SONAR_MS = 80;
const unsigned long TIMEOUT_SONAR_US = 6000; 
const unsigned long DEBOUNCE_IV_MS = 40;
const unsigned long TEMPO_VALIDACAO_MS = 400;
const unsigned long TEMPO_SERVO_MS = 100;   
const unsigned long TEMPO_SAIDA_ABERTA_MS = 2000; // espera depois do movimento de abertura
const unsigned long DURACAO_APITO_MS = 250;
const unsigned long PAUSA_APITOS_MS = 250;
const unsigned long TEMPO_LIVRE_MS = 700;
const unsigned long TEMPO_DESISTENCIA_MS = 8000;
const unsigned long INTERVALO_DEBUG_MS = 500;

const int PULSO_ENTRADA_FECHADA = 1300;
const int PULSO_ENTRADA_ABERTA = 1700;
const int PULSO_SAIDA_FECHADA = 1300;
const int PULSO_SAIDA_ABERTA = 1700;
const int PULSO_ENTRADA_PARADO = 1500;
const int PULSO_SAIDA_PARADO = 1500;

// PWM: servos a 50 Hz / 14 bits; buzzer a 880 Hz / 10 bits.
const int RESOLUCAO_SERVO = 14;
const int RESOLUCAO_BUZZER = 10;
const unsigned long MAX_DUTY_SERVO = 16383;

// =====================================================
// Estados
// =====================================================
enum Estado { REARMAR, FECHADA, VALIDAR, ABERTA, FECHAR, RECUSADA };

Estado estadoEntrada = REARMAR;
Estado estadoSaida = REARMAR;

unsigned long tempoInicioEntrada = 0;
unsigned long tempoInicioSaida = 0;

bool viuPassagemEntrada = false;
bool contouEntrada = false;
bool contouSaida = false;

bool sonarValido = false;
bool presente = false;
bool alturaOK = false;
float distancia = -1.0;
float altura = -1.0;
unsigned long ultimoSonar = 0;
int amostrasPresenca = 0;
int amostrasAltura = 0;
int amostrasLivres = 0;

bool ivEntradaAtivo = false;
bool ivSaidaAtivo = false;
bool ivEntradaBruto = false;
bool ivSaidaBruto = false;
unsigned long ivEntradaMudou = 0;
unsigned long ivSaidaMudou = 0;

bool servoEntradaOK = false;
bool servoSaidaOK = false;
bool buzzerOK = false;

unsigned long ultimoDebug = 0;

// =====================================================
// Funções
// =====================================================
bool tempoDecorreu(unsigned long agora, unsigned long antes, unsigned long intervalo) {
  return (agora - antes) >= intervalo;
}

void atualizarSensoresIV(unsigned long agora) {
  bool leituraEntrada = (digitalRead(IV_ENTRADA) == NIVEL_IV_ATIVO);
  if (leituraEntrada != ivEntradaBruto) {
    ivEntradaBruto = leituraEntrada;
    ivEntradaMudou = agora;
  }
  if (tempoDecorreu(agora, ivEntradaMudou, DEBOUNCE_IV_MS)) {
    ivEntradaAtivo = ivEntradaBruto;
  }

  bool leituraSaida = (digitalRead(IV_SAIDA) == NIVEL_IV_ATIVO);
  if (leituraSaida != ivSaidaBruto) {
    ivSaidaBruto = leituraSaida;
    ivSaidaMudou = agora;
  }
  if (tempoDecorreu(agora, ivSaidaMudou, DEBOUNCE_IV_MS)) {
    ivSaidaAtivo = ivSaidaBruto;
  }
}

void medirAltura(unsigned long agora) {
  if (!tempoDecorreu(agora, ultimoSonar, INTERVALO_SONAR_MS)) return;
  ultimoSonar = agora;

  pinMode(ULTRASONIC_SIG, OUTPUT);
  digitalWrite(ULTRASONIC_SIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_SIG, HIGH);
  delayMicroseconds(12); 
  digitalWrite(ULTRASONIC_SIG, LOW);

  pinMode(ULTRASONIC_SIG, INPUT);
  unsigned long duracao = pulseIn(ULTRASONIC_SIG, HIGH, TIMEOUT_SONAR_US);

  if (duracao == 0) {
    distancia = -1.0;
    sonarValido = false;
    altura = -1.0;
    alturaOK = false;
    amostrasPresenca = 0;
    amostrasAltura = 0;
    amostrasLivres = 0;
    return;
  }

  distancia = duracao / 58.3;
  sonarValido = (distancia >= 2.0 && distancia <= ALTURA_SENSOR_CM + 2.0);

  if (!sonarValido) {
    altura = -1.0;
    alturaOK = false;
    amostrasPresenca = 0;
    amostrasAltura = 0;
    amostrasLivres = 0;
    return;
  }

  altura = ALTURA_SENSOR_CM - distancia;
  if (altura < 0.0) altura = 0.0;

  bool detetado = (altura >= ALTURA_MIN_DETECAO_CM);

  if (detetado) {
    if (amostrasPresenca < 3) amostrasPresenca++;
    amostrasLivres = 0;
  } else {
    if (amostrasLivres < 3) amostrasLivres++;
    amostrasPresenca = 0;
  }

  if (detetado && altura <= ALTURA_MAX_CM) {
    if (amostrasAltura < 3) amostrasAltura++;
  } else {
    amostrasAltura = 0;
  }

  if (amostrasPresenca >= 3) presente = true;
  if (amostrasLivres >= 3) presente = false;
  alturaOK = (amostrasAltura >= 3);
}

void escreverServo(int pino, int pulsoUs) {
  unsigned long duty = (pulsoUs * MAX_DUTY_SERVO) / 20000;
  ledcWrite(pino, duty);
}

bool entradaAutorizada() {
  return ocupados < CAPACIDADE;
}

// =====================================================
// Contagem e LCD
// =====================================================
void atualizarContagem(bool entrou, bool saiu) {
  ocupados = ocupados + (entrou ? 1 : 0) - (saiu ? 1 : 0);
  if (ocupados < 0) ocupados = 0;
  if (ocupados > CAPACIDADE) ocupados = CAPACIDADE;
}

void iniciarLCD() {
  Wire.setPins(LCD_SDA, LCD_SCL);
  if (!Wire.begin()) {
    Serial.println("[LCD] Falha ao iniciar I2C. As cancelas continuam a funcionar.");
    return;
  }
  Wire.setClock(100000);
  Wire.setTimeOut(10);
  Wire.beginTransmission(ENDERECO_LCD);
  if (Wire.endTransmission() != 0) {
    Serial.println("[LCD] Sem resposta em 0x27. Verificar ligacoes/endereco com scanner_i2c.");
    return;
  }
  lcd.init(); // substitui lcd.begin(16, 2)
  lcd.backlight();
  // caracteres: ordinal masculino e a com til
  byte ordinal[8] = {6, 9, 9, 6, 0, 15, 0, 0};
  byte aComTil[8] = {13, 18, 0, 14, 1, 15, 17, 15};
  lcd.createChar(1, ordinal);
  lcd.createChar(2, aComTil);
  lcdOK = true;
  Serial.println("[LCD] KS0061 encontrado. Contagem inicia com OCUPACAO_INICIAL.");
}

void atualizarLCD(unsigned long agora, bool antesDeAbrir = false) {
  if (!lcdOK) return;
  bool entradaEmMovimento = (estadoEntrada == ABERTA || estadoEntrada == FECHAR) && !tempoDecorreu(agora, tempoInicioEntrada, TEMPO_SERVO_MS);
  bool saidaEmMovimento = (estadoSaida == ABERTA || estadoSaida == FECHAR) && !tempoDecorreu(agora, tempoInicioSaida, TEMPO_SERVO_MS);
  if ((!antesDeAbrir && entradaEmMovimento) || saidaEmMovimento) return;

  Wire.beginTransmission(ENDERECO_LCD);
  if (Wire.endTransmission() != 0) {
    lcdOK = false;
    Serial.println("[LCD] Ligacao perdida; verificar a montagem e reiniciar.");
    return;
  }

  String linha1 = "N\1 Lug. Ocupados";
  String linha2 = String(ocupados) + "/" + String(CAPACIDADE);
  if (ocupados >= CAPACIDADE) {
    linha1 = "Parque cheio!";
  } else if (sonarValido && presente && altura > ALTURA_MAX_CM) {
    linha1 = "Altura n\2o";
    linha2 = "permitida!";
  } else if (!servoEntradaOK || !servoSaidaOK) {
    linha1 = "Verificar servos";
  } else if (estadoEntrada == RECUSADA) {
    linha1 = "Entrada recusada";
  } else if (sonarValido && estadoEntrada == ABERTA && !contouEntrada) {
    linha1 = "Bem-vindo!";
  }

  while (linha1.length() < 16) linha1 += " ";
  while (linha2.length() < 16) linha2 += " ";
  linha1 = linha1.substring(0, 16);
  linha2 = linha2.substring(0, 16);
  lcd.setCursor(0, 0);
  lcd.print(linha1);
  lcd.setCursor(0, 1);
  lcd.print(linha2);
}

bool viaLivreDurante(bool condicaoLivre, unsigned long agora, unsigned long tempoAlvo, bool &aContar, unsigned long &desde) {
  if (!condicaoLivre) {
    aContar = false;
    return false;
  }
  if (!aContar) {
    aContar = true;
    desde = agora;
  }
  return tempoDecorreu(agora, desde, tempoAlvo);
}

// =====================================================
// Cancela Entrada
// =====================================================
bool processarEntrada(unsigned long agora) {
  bool eventoConfirmado = false;
  bool viaLivre = sonarValido && !presente && !ivEntradaAtivo;

  static bool aContarLivre = false;
  static unsigned long livreDesde = 0;

  switch (estadoEntrada) {
    case REARMAR:
      if (viaLivreDurante(viaLivre, agora, TEMPO_LIVRE_MS, aContarLivre, livreDesde)) {
        estadoEntrada = FECHADA;
        tempoInicioEntrada = agora;
        aContarLivre = false;
      }
      break;

    case FECHADA:
      if (sonarValido && presente && !ivEntradaAtivo) {
        estadoEntrada = VALIDAR;
        tempoInicioEntrada = agora;
        aContarLivre = false;
      }
      break;

    case VALIDAR:
      if (sonarValido && !presente) {
        estadoEntrada = REARMAR;
        aContarLivre = false;
      } else if (tempoDecorreu(agora, tempoInicioEntrada, TEMPO_VALIDACAO_MS)) {
        if (!sonarValido || !alturaOK || !entradaAutorizada() || ivEntradaAtivo) {
          estadoEntrada = RECUSADA;
          tempoInicioEntrada = agora;
          aContarLivre = false;
        } else {
          viuPassagemEntrada = false;
          contouEntrada = false;
          estadoEntrada = ABERTA;
          tempoInicioEntrada = agora;
          aContarLivre = false;
        }
      }
      break;

    case RECUSADA:
      if (viaLivreDurante(viaLivre, agora, TEMPO_LIVRE_MS, aContarLivre, livreDesde)) {
        estadoEntrada = FECHADA;
        aContarLivre = false;
      }
      break;

    case ABERTA:
      if (ivEntradaAtivo) viuPassagemEntrada = true;

      if (viuPassagemEntrada && !ivEntradaAtivo && !contouEntrada &&
          tempoDecorreu(agora, tempoInicioEntrada, TEMPO_SERVO_MS)) {
        contouEntrada = true;
        eventoConfirmado = true;
      }

      // Depois da passagem confirmada, o IV livre permite fechar.
      // O sonar continua a validar aproximacao/altura e a desistência sem passagem.
      if (viaLivreDurante(contouEntrada ? !ivEntradaAtivo : viaLivre,
                          agora, TEMPO_LIVRE_MS, aContarLivre, livreDesde) &&
          tempoDecorreu(agora, tempoInicioEntrada, TEMPO_SERVO_MS) &&
          (contouEntrada || tempoDecorreu(agora, tempoInicioEntrada, TEMPO_DESISTENCIA_MS))) {
        estadoEntrada = FECHAR;
        tempoInicioEntrada = agora;
        aContarLivre = false;
      }
      break;

    case FECHAR:
      // se voltar a haver algo na zona, reabrir em vez de continuar a fechar
      if (ivEntradaAtivo || (!contouEntrada && (!sonarValido || presente))) {
        estadoEntrada = ABERTA;
        tempoInicioEntrada = agora;
        aContarLivre = false;
      } else if (tempoDecorreu(agora, tempoInicioEntrada, TEMPO_SERVO_MS)) {
        estadoEntrada = REARMAR;
        aContarLivre = false;
      }
      break;
  }

  return eventoConfirmado;
}

// =====================================================
// Cancela Saída
// =====================================================
bool processarSaida(unsigned long agora) {
  bool eventoConfirmado = false;
  bool viaLivre = !ivSaidaAtivo;

  static bool aContarLivre = false;
  static unsigned long livreDesde = 0;

  switch (estadoSaida) {
    case REARMAR:
      if (viaLivreDurante(viaLivre, agora, TEMPO_LIVRE_MS, aContarLivre, livreDesde)) {
        estadoSaida = FECHADA;
        tempoInicioSaida = agora;
        aContarLivre = false;
      }
      break;

    case FECHADA:
      if (ivSaidaAtivo) {
        contouSaida = false;
        estadoSaida = ABERTA;
        tempoInicioSaida = agora;
        aContarLivre = false;
      }
      break;

    case ABERTA:
      if (viaLivreDurante(viaLivre, agora, TEMPO_LIVRE_MS, aContarLivre, livreDesde) &&
          tempoDecorreu(agora, tempoInicioSaida, TEMPO_SERVO_MS + TEMPO_SAIDA_ABERTA_MS)) {
        if (!contouSaida) {
          contouSaida = true;
          eventoConfirmado = true;
        }
        estadoSaida = FECHAR;
        tempoInicioSaida = agora;
        aContarLivre = false;
      }
      break;

    case FECHAR:
      if (ivSaidaAtivo) {
        estadoSaida = ABERTA;
        tempoInicioSaida = agora;
        aContarLivre = false;
      } else if (tempoDecorreu(agora, tempoInicioSaida, TEMPO_SERVO_MS)) {
        estadoSaida = REARMAR;
        aContarLivre = false;
      }
      break;

    default:
      estadoSaida = REARMAR;
      break;
  }

  return eventoConfirmado;
}

// =====================================================
// Leds, buzzers servomotores
// =====================================================
void atualizarSaidasHardware(unsigned long agora) {
  if (servoEntradaOK) {
    int pulso =   PULSO_ENTRADA_PARADO;
    if(!tempoDecorreu(agora, tempoInicioEntrada, TEMPO_SERVO_MS)){
      if(estadoEntrada == ABERTA) pulso = PULSO_ENTRADA_ABERTA;
      if(estadoEntrada == FECHAR) pulso = PULSO_ENTRADA_FECHADA;
    }
    escreverServo(SERVO_ENTRADA, pulso);
  }

  if (servoSaidaOK) {
    int pulso =   PULSO_SAIDA_PARADO;
    if(!tempoDecorreu(agora, tempoInicioSaida, TEMPO_SERVO_MS)){
      if(estadoSaida == ABERTA) pulso = PULSO_SAIDA_ABERTA;
      if(estadoSaida == FECHAR) pulso = PULSO_SAIDA_FECHADA;
    }
    escreverServo(SERVO_SAIDA, pulso);
  }

  // Vermelho - falha de leitura/recusa/falha do servo da entrada
  bool alturaInvalida = sonarValido && presente && (altura > ALTURA_MAX_CM);
  bool aviso = !sonarValido || alturaInvalida || (estadoEntrada == RECUSADA) || !entradaAutorizada() || !servoEntradaOK;
  digitalWrite(LED_VERDE, aviso ? LOW : HIGH);
  digitalWrite(LED_VERMELHO, aviso ? HIGH : LOW);
  
  if (buzzerOK) {
    static bool alturaJaAvisada = false;
    static bool sequenciaAtiva = false;
    static unsigned long inicioApitos = 0;

    if (sonarValido && !presente) alturaJaAvisada = false;
    bool avisoAltura = alturaInvalida && entradaAutorizada();
    if (!avisoAltura) sequenciaAtiva = false;
    if (avisoAltura && !alturaJaAvisada) {
      inicioApitos = agora;
      sequenciaAtiva = true;
      alturaJaAvisada = true;
    }

    unsigned long decorrido = agora - inicioApitos;
    bool som = false;
    if (sequenciaAtiva) {
      som = (decorrido < DURACAO_APITO_MS) ||
            (decorrido >= DURACAO_APITO_MS + PAUSA_APITOS_MS &&
             decorrido < 2 * DURACAO_APITO_MS + PAUSA_APITOS_MS);
      if (decorrido >= 2 * DURACAO_APITO_MS + PAUSA_APITOS_MS) {
        sequenciaAtiva = false;
      }
    }
    ledcWrite(BUZZER, som ? 512 : 0);
  }
}

// =====================================================
// Serial Monitor
// =====================================================
void imprimirDiagnostico(unsigned long agora) {
  if (!tempoDecorreu(agora, ultimoDebug, INTERVALO_DEBUG_MS)) return;
  ultimoDebug = agora;

  Serial.print("Distancia: ");
  if (distancia >=0 ) Serial.print(distancia, 1);
  else Serial.print("sem eco");
  Serial.print(" cm | sonar valido: ");
  Serial.print(sonarValido ? "sim" : "nao");
  Serial.print(" | Altura: ");
  if (sonarValido) Serial.print(altura, 1);
  else Serial.print("--");
  Serial.print(" cm | Presenca: ");
  if(sonarValido) Serial.print(presente ? "sim" : "nao");
  else Serial.print("indefinida");
  Serial.print(" | Altura OK: ");
  Serial.print(alturaOK ? "sim" : "nao");
  Serial.print(" | IV entrada: ");
  Serial.print(ivEntradaAtivo ? "ativo" : "livre");
  Serial.print(" (nivel "); Serial.print(digitalRead(IV_ENTRADA)); Serial.print(")");
  Serial.print(" | IV saida: ");
  Serial.print(ivSaidaAtivo ? "ativo" : "livre");
  Serial.print(" (nivel "); Serial.print(digitalRead(IV_SAIDA)); Serial.print(")");
  Serial.print(" | Entrada: ");
  Serial.print((int)estadoEntrada);
  Serial.print(" | Saida: ");
  Serial.print((int)estadoSaida);
  Serial.print(" | Ocupados: "); Serial.print(ocupados);
  Serial.print(" | Livres: "); Serial.println(CAPACIDADE - ocupados);
}

// =====================================================
// 9. Setup e loop
// =====================================================
void setup() {
  Serial.begin(115200);

  pinMode(ULTRASONIC_SIG, INPUT);
  pinMode(IV_ENTRADA, INPUT_PULLUP);
  pinMode(IV_SAIDA, INPUT_PULLUP);
  pinMode(LED_VERDE, OUTPUT);
  pinMode(LED_VERMELHO, OUTPUT);

  digitalWrite(LED_VERDE, LOW);
  digitalWrite(LED_VERMELHO, HIGH);

  // Os canais são independentes: uma falha no buzzer não bloqueia os servos
  // ledcAttachChannel(pino, frequência, resolução, canal)- configurar um pino da ESP32 para gerar um sinal PWM
  servoEntradaOK = ledcAttachChannel(SERVO_ENTRADA, 50, RESOLUCAO_SERVO, 0);
  servoSaidaOK = ledcAttachChannel(SERVO_SAIDA, 50, RESOLUCAO_SERVO, 1);
  buzzerOK = ledcAttachChannel(BUZZER, 880, RESOLUCAO_BUZZER, 2);

  if (servoEntradaOK) escreverServo(SERVO_ENTRADA, PULSO_ENTRADA_PARADO);
  if (servoSaidaOK) escreverServo(SERVO_SAIDA, PULSO_SAIDA_PARADO);
  if (buzzerOK) ledcWrite(BUZZER, 0);

  if (!servoEntradaOK) Serial.println("[ERRO] PWM da cancela de entrada indisponivel.");
  if (!servoSaidaOK) Serial.println("[ERRO] PWM da cancela de saida indisponivel.");
  if (!buzzerOK) Serial.println("[AVISO] PWM do buzzer indisponivel; as cancelas continuam independentes.");

  Serial.println();
  Serial.println("Estados: 0=REARMAR, 1=FECHADA, 2=VALIDAR, 3=ABERTA, 4=FECHAR, 5=RECUSADA");
  Serial.print("Altura valida: "); Serial.print(ALTURA_MIN_DETECAO_CM);
  Serial.print(" a "); Serial.print(ALTURA_MAX_CM); Serial.println(" cm (inclusive)");
  if (ocupados < 0) ocupados = 0;
  if (ocupados > CAPACIDADE) ocupados = CAPACIDADE;
  iniciarLCD();
}

void loop() {
  unsigned long agora = millis();

  atualizarSensoresIV(agora);
  medirAltura(agora);

  agora = millis();

  bool entrou = false;
  bool saiu = false;
  Estado entradaAntes = estadoEntrada;

  if (servoEntradaOK) entrou = processarEntrada(agora);
  if (servoSaidaOK) saiu = processarSaida(agora);

  if (entrou) Serial.println("[EVENTO] Passagem de entrada confirmada pelo sensor IV.");
  if (saiu) Serial.println("[EVENTO] Passagem de saida confirmada pelo sensor IV.");
  atualizarContagem(entrou, saiu);

  // Mostrar a saudacao antes de iniciar o movimento, sem prolongar o pulso.
  if (entradaAntes != ABERTA && estadoEntrada == ABERTA) {
    atualizarLCD(millis(), true);
    tempoInicioEntrada = millis();
  }
  agora = millis();
  atualizarSaidasHardware(agora);
  imprimirDiagnostico(agora);
  atualizarLCD(millis());
}
