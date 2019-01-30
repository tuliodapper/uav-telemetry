#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <BMP180.h>
#include <Adafruit_INA219.h>
#include <SdFat.h> // Cartao SD

#define DELAY_MS 200

// ### CARTAO SD ####
SdFat sdCard;
SdFile meuArquivo;
const int chipSelect = 5; // Pino ligado ao CS do modulo

// ### ACELEROMETRO ####
#define PINO_INTERRUPCAO 2  // use pin 2 on Arduino Uno & most boards
MPU6050 mpu;
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

// ### SENSOR PRESSAO DIFERENCIAL ####
#define A 0.0378
#define B 0.0378
#define C -0.9244
#define NUM_AMOSTRAS_PRESSAO_DIFERENCIAL 125
#define PINO_PRESSAO_DIFERENCIAL A0
double ValorPrDif = 0;

// ### MEDIDOR DE CORRENTE ####
Adafruit_INA219 ina219;
int ValorMedCorr = 0;  

// ### BAROMETRO ###
BMP180 barometer;
long ValorPressao = 0;
double ValorTemperatura = 0;

// ### BOTAO ####
#define PIN0_BOTAO_INICIAR_PAUSAR 13
#define debounceDelay 50
int btn_begin;             
int btn_begin_last = LOW;
unsigned long lastDebounceTime_btn_begin = 0;

// ### LED ####
#define PINO_LED 8
int sgn_led = LOW;

// ### VARIÁVEIS GERAIS ####
int b_transmitindo = 0;
int b_gravando     = 0;

int b_ler_sensor_pressao_diferencial = 0;
int b_ler_orientacao         = 0;
int b_ler_pressao            = 0;
int b_ler_corrente_eletrica  = 0;
int b_ler_velocidade_roda    = 0;
int b_ler_aceleracao         = 0;

int b_habilitado_cartao_sd         = 0;
int b_habilitado_orientacao_aceleracao = 0;
int b_habilitado_pressao           = 0;
int b_habilitado_velocidade_roda   = 0;

unsigned long tempo_inicio = 0;
unsigned long tempo_fim = 0;

// ### FUNÇÕES ####
void gravar_dados();
void enviar_dados();
void ler_sensor_pressao_diferencial();
void ler_orientacao_aceleracao();
void ler_pressao();
void ler_corrente_eletrica();
void ler_velocidade_roda();

void dmpDataReady();
int iniciar_arquivo();
void sinalizar(int tempo);
void aguarda(unsigned long tempo_max);

void setup() {
  
  // Define as entradas
  pinMode(PINO_PRESSAO_DIFERENCIAL, INPUT);
  pinMode(PIN0_BOTAO_INICIAR_PAUSAR, INPUT);
  pinMode(PINO_INTERRUPCAO, INPUT);
    
  // Define as saidas
  pinMode(PINO_LED, OUTPUT);
   
  // ## SERIAL ## Obs: Tranca se não tiver comunicação serial funcionando, pois telemetria é fundamental para o funcionamento do sistema.
  Serial.begin(57600);   
  if (!Serial) {
    while(1) { sinalizar(100); }
  }
  
  // ## CARTÃO SD ##
  if(sdCard.begin(chipSelect,SPI_HALF_SPEED)){
    b_habilitado_cartao_sd = 1;
  }
  
  // ### ACELEROMETRO ####  
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  mpu.setDLPFMode(MPU6050_DLPF_BW_5);
  mpu.setXGyroOffset(68);
  mpu.setYGyroOffset(-8);
  mpu.setZGyroOffset(-20);
  mpu.setXAccelOffset(963);
  mpu.setYAccelOffset(-3524);
  mpu.setZAccelOffset(720);
  if (devStatus == 0) {
    b_habilitado_orientacao_aceleracao = 1;
  }

  // ### BAROMETRO ####  
  barometer = BMP180();
  if(barometer.EnsureConnected())
  {
    barometer.SoftReset();
    barometer.Initialize();
    b_habilitado_pressao  = 1;
  }
  
  // ### MEDIDOR DE CORRENTE ### Obs: o sensor de corrente não tem verificação se está habilitado ou não.
  ina219.begin();
  
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

  char data[30];
  char nome_arquivo[15];
  char numero;
  char i;
  char caracter;

  // Se não está gravando em cartão SD e não está transmitindo por telemetria, então aguarda comando de gravação e/ou transmissão.
  while ((b_gravando == 0) && (b_transmitindo == 0)){
    if(Serial.available()){
      numero = Serial.readBytesUntil('|',data,30);
      data[numero] = 0;
      // Protocolo de inicio de coleta de dados : "b_transmitindo;b_gravando;b_ler_sensor_pressao_diferencial;b_ler_orientacao;b_ler_pressao;b_ler_corrente_eletrica;b_ler_velocidade_roda;b_ler_aceleracao;nome_arquivo"
      i = sscanf((char*)data,"%d;%d;%d;%d;%d;%d;%d;%d;%s", &b_transmitindo, &b_gravando, &b_ler_sensor_pressao_diferencial, &b_ler_orientacao, &b_ler_pressao, &b_ler_corrente_eletrica, &b_ler_velocidade_roda, &b_ler_aceleracao, nome_arquivo);
      // Confere integridade da mensagem de inicio de coleta de dados
      if (i == 9){
        // Cria arquivo com o nome se b_gravando = 1
        if (b_gravando == 1) {
          if (!meuArquivo.open(nome_arquivo, O_WRITE | O_CREAT | O_AT_END)) {
            b_gravando = 0;
          }
        }
        // Inicializa a interrupção do MPU-6050 se (b_ler_orientacao == 1) || (b_ler_aceleracao == 1)
        if ((b_ler_orientacao == 1) || (b_ler_aceleracao == 1)) {
          mpu.setDMPEnabled(true);
          attachInterrupt(digitalPinToInterrupt(PINO_INTERRUPCAO), dmpDataReady, RISING);
          mpuIntStatus = mpu.getIntStatus();
          packetSize = mpu.dmpGetFIFOPacketSize();
          mpu.resetFIFO();
        }
        tempo_inicio = millis();
        
      // Se a mensagem não se encaixou em uma mensagem de inicio de coleta de dados, então pode ser que...
      } else {
        b_transmitindo  = 0;
        b_gravando      = 0;
        
        i = sscanf((char*)data, "%s", nome_arquivo);
        if (i == 1){
          // Confere se é um comando de Pausar; se sim, manda ok, pq está parado mesmo...
          if (nome_arquivo[0] == 'p'){
            Serial.println("o");
          // Confere se é um comando de Enviar Habilitados; se sim, envia...
          } else if (nome_arquivo[0] == 'i'){
            enviar_habilitados();
          }
        }
      }
    }
    
    if (b_gravando == 1) {
      // Inserir Colunas
      meuArquivo.println("Tempo;PrDif;Q.w;Q.x;Q.y;Q.z;Pressao;Corrente;VelRoda;Acel.x;Acel.y;Acel.z");
      // Inserir Medidas Habilitadas
      meuArquivo.print("1;");
      meuArquivo.print(b_ler_sensor_pressao_diferencial);
      meuArquivo.print(';');
      meuArquivo.print(b_ler_orientacao);
      meuArquivo.print(';');
      meuArquivo.print(b_ler_orientacao);
      meuArquivo.print(';');
      meuArquivo.print(b_ler_orientacao);
      meuArquivo.print(';');
      meuArquivo.print(b_ler_orientacao);
      meuArquivo.print(';');
      meuArquivo.print(b_ler_pressao);
      meuArquivo.print(';');
      meuArquivo.print(b_ler_corrente_eletrica);
      meuArquivo.print(';');
      meuArquivo.print(b_ler_velocidade_roda);
      meuArquivo.print(';');
      meuArquivo.print(b_ler_aceleracao);
      meuArquivo.print(';');
      meuArquivo.print(b_ler_aceleracao);
      meuArquivo.print(';');
      meuArquivo.println(b_ler_aceleracao);
    }
    
    if ((b_gravando == 1) || (b_transmitindo == 1)){
      digitalWrite(PINO_LED, LOW);
      Serial.println("o");
    }
  }
  
  tempo_fim = millis();

  // Aguardar completar 50 ms
  if ((b_ler_orientacao == 1) || (b_ler_aceleracao == 1)) {
    while (!mpuInterrupt && fifoCount < packetSize){}
  } else {
    delay(DELAY_MS - (tempo_fim - tempo_inicio));
  }

  tempo_inicio = millis();

  if (b_ler_sensor_pressao_diferencial == 1) {
    ler_sensor_pressao_diferencial();
  }

  if ((b_ler_orientacao == 1) || (b_ler_aceleracao == 1)) {
    ler_orientacao_aceleracao();
  }

  if (b_ler_pressao == 1) {
    ler_pressao();
  }

  if (b_ler_corrente_eletrica == 1) {
    ler_corrente_eletrica();
  }

  if (b_ler_velocidade_roda == 1){
    ler_velocidade_roda();
  }

  if (b_gravando == 1) {
    gravar_dados();
  }

  if (b_transmitindo == 1){
    enviar_dados();
  }

  // Verifica se deve Pausar (comando "p")
  if(Serial.available()){
    numero = Serial.read();
    if (numero == 'p'){
        // Desabilita interrupção se estiver (b_ler_orientacao == 1) || (b_ler_aceleracao == 1)
        if ((b_ler_orientacao == 1) || (b_ler_aceleracao == 1)) {
          mpu.setDMPEnabled(false);
          detachInterrupt(digitalPinToInterrupt(PINO_INTERRUPCAO));
          mpuInterrupt = false;
        }
        // Fecha arquivo se estiver gravando
        if (b_gravando == 1) {
          meuArquivo.close();
          b_gravando = 0;
        }
        b_transmitindo = 0;
        // Envia Ok
        Serial.println("o");
    }
  }

}

void ler_sensor_pressao_diferencial(){
  uint16_t velocidade_amostrada_anterior = 0;
  char i;
  ValorPrDif = 0;
  for (i=0;i<NUM_AMOSTRAS_PRESSAO_DIFERENCIAL;i++){
    ValorPrDif = C*ValorPrDif + (double) (A*analogRead(PINO_PRESSAO_DIFERENCIAL)) + (double) (B*velocidade_amostrada_anterior);
    delayMicroseconds(1190);
  }
}

void ler_orientacao_aceleracao(){
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      mpu.resetFIFO();
      digitalWrite(PINO_LED, HIGH);
  } else if (mpuIntStatus & 0x02) {
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;
      if ((b_ler_orientacao == 1) || (b_ler_aceleracao == 1)) {
        mpu.dmpGetQuaternion(&q, fifoBuffer); // q.w, q.x, q.y, q.z
      }
      if (b_ler_aceleracao == 1) {
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      }
  }
}

void ler_pressao(){
  if(barometer.IsConnected){
    ValorPressao = barometer.GetPressure();
  }
}

void ler_corrente_eletrica(){
  ValorMedCorr = ina219.getCurrent_mA();
}

void ler_velocidade_roda(){
  
}

void aguarda(unsigned long tempo_max){
}

void dmpDataReady() {
  mpuInterrupt = true;
}

void sinalizar(int tempo){
  digitalWrite(PINO_LED, HIGH);
  delay(100);
  digitalWrite(PINO_LED, LOW);
  delay(tempo);
  digitalWrite(PINO_LED, HIGH);
  delay(100);
  digitalWrite(PINO_LED, LOW);
  delay(tempo);
  digitalWrite(PINO_LED, HIGH); 
  delay(100);
  digitalWrite(PINO_LED, LOW);
  delay(tempo);
}

void gravar_dados(){
  // Tempo
  meuArquivo.print(tempo_inicio);
  meuArquivo.print(';');
  // Pressao Diferencial
  meuArquivo.print(ValorPrDif);
  meuArquivo.print(';');
  // Orientação
  meuArquivo.print(q.w);
  meuArquivo.print(';');
  meuArquivo.print(q.x);
  meuArquivo.print(';');
  meuArquivo.print(q.y);
  meuArquivo.print(';');
  meuArquivo.print(q.z);
  meuArquivo.print(';');
  // Pressao
  meuArquivo.print(ValorPressao);
  meuArquivo.print(';');
  // Medição de Corrente
  meuArquivo.print(ValorMedCorr);
  meuArquivo.print(';');
  // Velocidade da Roda
  meuArquivo.print("0");
  meuArquivo.print(';');
  // Aceleração da aeronave
  meuArquivo.print(aaReal.x);
  meuArquivo.print(';');
  meuArquivo.print(aaReal.y);
  meuArquivo.print(';');
  meuArquivo.print(aaReal.z);
  meuArquivo.println();
}

void enviar_dados(){
  // Tempo
  Serial.print(tempo_inicio);
  Serial.print(';');
  // Pressao Diferencial
  Serial.print(long (ValorPrDif*100));
  Serial.print(';');
  // Orientação
  Serial.print(long (q.w*100));
  Serial.print(';');
  Serial.print(long (q.x*100));
  Serial.print(';');
  Serial.print(long (q.y*100));
  Serial.print(';');
  Serial.print(long (q.z*100));
  Serial.print(';');
  // Pressao
  Serial.print(ValorPressao);
  Serial.print(';');
  // Medição de Corrente
  Serial.print(ValorMedCorr);
  Serial.print(';');
  // Velocidade da Roda
  Serial.print(0);
  Serial.print(';');
  // Aceleração da aeronave
  Serial.print(aaReal.x);
  Serial.print(';');
  Serial.print(aaReal.y);
  Serial.print(';');
  Serial.print(aaReal.z);
  Serial.println(';');
}

void enviar_habilitados() {
  Serial.print(b_habilitado_cartao_sd);
  Serial.print(';');
  Serial.print(b_habilitado_orientacao_aceleracao);
  Serial.print(';');
  Serial.print(b_habilitado_pressao);
  Serial.print(';');
  Serial.print(b_habilitado_velocidade_roda);
  Serial.println(';');
}

