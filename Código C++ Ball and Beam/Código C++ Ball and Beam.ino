#include <Servo.h> // Biblioteca para controlar o servomotor
#include "Adafruit_VL53L0X.h" // Biblioteca para usar o sensor VL53L0X

Adafruit_VL53L0X lox = Adafruit_VL53L0X(); // Objeto do sensor VL53L0X
Servo s; // Objeto do tipo Servo para controlar o motor

const int pinoServo = 9; // Pino de controle do servomotor
int horizontal = 90; // Posição inicial do servo para a barra equilibrada

// Filtro Butterworth para suavizar os valores da leitura do sensor
float alpha = 0.2; // Constante para suavização (quanto menor, mais suaviza)
float filteredValue = 0; // Armazena o valor filtrado

// Variáveis PID e controle
float Acao = 0;
float Kd = 1;
float Kp = 1;
float Ki = 1;
float posicao = 0; // Posição lida do sensor
float posicaoreal = 0; // Posição após ajuste do filtro
float Erro = 0; // Diferença entre a posição de referência e a real
float Erro_ant = 0; // Armazena o erro anterior
float Errotheta = 0; // Ângulo de correção do servo
float Acaotheta = 0; // Ação a ser aplicada no servo
float Ref = 0; // Referência de posição (ajustável pela interface Python)
int count = 0; // Contador para temporização
long tempo = 0; // Temporizador global
float TempoCiclo = 0.01; // Tempo de ciclo (em segundos)
float T = TempoCiclo; // Usado no cálculo de Ki e Kd

// Variáveis para autotune
float Kp_auto = 0.1;
float Ku = 0;
float Tu = 0;
bool oscilando = false;
bool medindoPeriodo = false;
unsigned long inicioOscilacao = 0;
unsigned long fimOscilacao = 0;
int numOscilacoes = 0;

float Kp_limite = 1.6;
float Ki_limite = 0.2;
float Kd_limite = 0.2;

// Variável para indicar se o autotune foi finalizado
bool autotune_finalizado = false;

void setup() {
  Serial.begin(115200); // Inicia a comunicação serial
  if (!lox.begin()) { // Verifica se o sensor foi iniciado corretamente
    Serial.println(F("Falha ao inicializar o VL53L0X"));
    while (1); // Para o programa se o sensor não inicializar
  }

  // Configurações do Timer2 para gerar interrupções a cada 10 ms
  TCCR2A = 0x00; // Modo normal
  TCCR2B = 0x07; // Prescaler 1:1024
  TCNT2 = 100; // Configura o tempo de overflow para 10 ms
  TIMSK2 = 0x01; // Habilita a interrupção do Timer2

  s.attach(pinoServo); // Associa o pino do servo ao objeto Servo
  s.write(horizontal); // Inicializa o servo na posição horizontal
}

// Rotina de interrupção do Timer2 (executada a cada 10 ms)
ISR(TIMER2_OVF_vect) {
  count++; // Incrementa o contador
  tempo++; // Incrementa o temporizador global

  if (count == 6) { // Executa a cada 60 ms
    count = 0; // Reseta o contador

    // Filtro Butterworth para suavizar o valor do sensor
    filteredValue = alpha * posicao + (1 - alpha) * filteredValue;

    if (filteredValue != 0 && filteredValue <= 700) { // Verifica leituras válidas
      posicaoreal = filteredValue - 225; // Ajuste da posição real
    }

    Erro = Ref - posicaoreal; // Calcula o erro entre referência e posição atual

    // Calcula a ação de controle PID
    Acao = Kp * Erro + Ki * (Erro + Erro_ant) * T / 2 + Kd * (Erro - Erro_ant) / T;
    Erro_ant = Erro; // Atualiza o erro anterior para o próximo ciclo

    // Converte a ação em um ângulo para o servo
    Errotheta = Acao * 22 / 200;
    Acaotheta = Errotheta;

    // Limita o ângulo do servo para evitar ultrapassagem
    if (Acaotheta > 22) {
      Acaotheta = 22;
    } else if (Acaotheta < -22) {
      Acaotheta = -22;
    }

    int anguloServo = horizontal + Acaotheta;
    s.write(anguloServo); // Envia o comando para o servo
    autotunePID();
  }

  TCNT2 = 100; // Reinicia o timer para o próximo ciclo
}

void autotunePID() {
  if (!oscilando) {
    Kp_auto += 0.05;

    if (Kp_auto > Kp_limite) {
      Kp_auto = Kp_limite;
    }
    oscilando = verificarOscilacao();
  }

  if (oscilando && !medindoPeriodo) {
    inicioOscilacao = millis();
    medindoPeriodo = true;
  }

  if (medindoPeriodo && verificarOscilacao()) {
    fimOscilacao = millis();
    Tu = (fimOscilacao - inicioOscilacao) / 1000.0;
    Ku = Kp_auto;
    calcularPID();
    numOscilacoes++;

    // Se completou 1 oscilações, finaliza o autotune
    if (numOscilacoes >= 1) {
      Serial.println("Autotune finalizado");
      oscilando = false;
      medindoPeriodo = false;
      return;
    }

    inicioOscilacao = fimOscilacao; // Reinicia a contagem para uma nova oscilação
  }
}

bool verificarOscilacao() {
  static float posicao_ant = posicaoreal;
  if ((posicaoreal > 0 && posicao_ant < 0) || (posicaoreal < 0 && posicao_ant > 0)) {
    posicao_ant = posicaoreal;
    return true;
  }
  posicao_ant = posicaoreal;
  return false;
}

void calcularPID() {
  if (!autotune_finalizado) {
    Kp = 0.6 * Ku;
    Ki = 2 * Kp / Tu;
    Kd = Kp * Tu / 8;

    // Limita os valores de Ki e Kd
    if (Ki > Ki_limite) Ki = Ki_limite;
    if (Kd > Kd_limite) Kd = Kd_limite;

    // Envia os valores ajustados de PID para o Python via Serial
    Serial.println("Autotune finalizado");
    Serial.print("Kp: ");
    Serial.println(Kp);
    Serial.print("Ki: ");
    Serial.println(Ki);
    Serial.print("Kd: ");
    Serial.println(Kd);

    autotune_finalizado = true; // Marca o autotune como finalizado
  }
}

void reiniciarAutotune() {
  autotune_finalizado = false; // Permitir novo autotune
  Kp = 1;
  Ki = 1;
  Kd = 1;
  Kp_auto = 0.1;
  numOscilacoes = 0;
  oscilando = false;
  medindoPeriodo = false;
  Serial.println("Autotune reiniciado");
}

void loop() {
  // Verifica se há comandos do Python na serial
  if (Serial.available()) {
    String comando = Serial.readStringUntil('\n');
    comando.trim();
    if (comando.startsWith("SET_REF:")) {
      String valorRef = comando.substring(8);
      Ref = valorRef.toFloat(); // Atualiza a referência
      Serial.print("Nova referencia: ");
      Serial.println(Ref);
    } else if (comando == "REINICIAR_AUTOTUNE") {
      reiniciarAutotune();
    }
  }

  // Adquire os dados do sensor VL53L0X
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  posicao = measure.RangeMilliMeter;

  // Envia a posição atual para o Python
  Serial.print("Posicao: ");
  Serial.println(posicao);

  delay(60); 
}
