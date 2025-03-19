#include "tpl_os.h"
#include "Arduino.h"
#include "board_v2.h"

//Macro para definicoes
#define mEEC1_DLC       8
#define mEEC1_EXT_FRAME 1

#define mEEC1_ID_ngear  0x18F00503
#define mEEC1_ID_rpm    0x0CF00400
#define mEEC1_ID_speed  0x18FEF100

#define resolution_ngear    1 
#define resolution_rpm      0.125   // 0.125 bit / RPM --> 8 RPM / bit
#define resolution_speed    256     // faixa -> 0 até 250km/h

#define offset_ngear    -125
#define offset_rpm      0
#define offset_speed    0

#define max_speed   250
#define base_rpm    3200

#define R_w 0.326   // raio do pneu
#define n_d 3.55    // taxa de transmissão do diferencial
//#define pi  3.14159 
#define pi_over_30 0.10471975512 // para transformação de rps para radianos/s

// Variáveis numéricas para (des)encapsulamento
long unsigned int mID_ngear;
unsigned char mDLC_ngear = 0;
unsigned char mDATA_ngear[8];

//FRAME_DATA (SEND)
unsigned char mEEC1_data_rpm[8]     = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
unsigned char mEEC1_data_speed[8]   = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static unsigned int n_gear  = 1;
static unsigned int rpm     = 0; // unidade: rpm // !Utiliza-se rps para cálculo da velocidade!
static float speed          = 0; // unidade: km/h

//Constroi um objeto MCP_CAN e configura o chip selector para o pino definido no board.h para ECU2_CAN1_CS.
MCP_CAN CAN1(ECU3_CAN1_CS);

void setup(){
    Serial .begin(115200);
    // Inicializa o controlador can : baudrate = 250K, clock=8MHz
    while(CAN1.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK){
        delay(200);
    }
    Serial.println ("MCP2515 can_send inicializado com sucesso!");
    CAN1.setMode(MCP_NORMAL);
}

TASK(serialRPM){ //  “-w=XXXX”          // espero ao menos 7 dígitos seguindo o padrão, com a adição de \n
    if(Serial.available() > 0){         // Padrão básico -> Como estou lendo na Serial, simplifiquei o formato da mensagem
        char nextRPM[7]; 
        Serial.readBytes(nextRPM, 7);
        if(nextRPM[0] == '-' && nextRPM[1] == 'w' && nextRPM[2] == '='){
            unsigned int maxRPM = base_rpm + 800 * n_gear; // Limites máximos RPM
            unsigned int temp_rpm = (nextRPM[3] - '0')*1000 + (nextRPM[4] - '0')*100 + (nextRPM[5] - '0')*10 + (nextRPM[6] - '0');
            
            if(temp_rpm >= 0 && temp_rpm <= maxRPM){
                GetResource(infoSerial);
                rpm = temp_rpm;
                ReleaseResource(infoSerial);
            }
        }
        String flushing = Serial.readString(); // "Limpa" a Serial
    }
    TerminateTask();
} 

TASK(interfaceProcessing){ 
    float taxa_transm[5] = {3.83, 2.36, 1.69, 1.31, 1.0};

    GetResource(infoCAN);
    float n_g = taxa_transm[(n_gear - 1)];
    ReleaseResource(infoCAN);
    
    GetResource(infoSerial); // Get/Release Relacionado ao rpm
    // rpm * pi_over_30 => radianos/s; m/s * 3.6 => km/h
    float temp_speed = (R_w * rpm * pi_over_30 * 3.6)/(n_g * n_d); 
    // Encapsulamento da rotação: 
    unsigned int data_rpm = rpm * resolution_rpm;
    ReleaseResource(infoSerial);

    unsigned char ms_rpm, ls_rpm;
    ls_rpm = data_rpm;
    ms_rpm = data_rpm >> 8;
    GetResource(infoCAN);
    mEEC1_data_rpm[3] = ls_rpm; 
    mEEC1_data_rpm[4] = ms_rpm;
    ReleaseResource(infoCAN);
    

    if(temp_speed >= 0 && temp_speed <= 250){
        speed = temp_speed;
    }

    // Encapsulamento da velocidade: Data_speed = speed/res - offset => speed * 256
    unsigned int data_speed = speed * resolution_speed;
    unsigned char ms_speed, ls_speed;
    ls_speed = data_speed;
    ms_speed = data_speed >> 8;
    GetResource(infoCAN);
    mEEC1_data_speed[1] = ls_speed; 
    mEEC1_data_speed[2] = ms_speed;
    ReleaseResource(infoCAN);

    TerminateTask();
} 

void readCAN(void){
    if (!digitalRead(ECU3_CAN1_INT)){
        //Lê os dados: mID = identificador, mDLC = comprimento, mDATA = dados do frame
        CAN1.readMsgBuf(&mID_ngear, &mDLC_ngear, mDATA_ngear);

        if((mID_ngear & CAN_EXTENDED_ID) == mEEC1_ID_ngear){
            unsigned int data_ngear = 0;
            data_ngear = mDATA_ngear[3];

            unsigned int temp_ngear = data_ngear * resolution_ngear + offset_ngear;
            if (n_gear < 1 || n_gear > 5) n_gear = 1;

            if(temp_ngear >= 1 && temp_ngear <= 5){
                n_gear = temp_ngear;
            }
        }
        //Serial.print("Sua velocidade eh "); Serial.println(speed);
    }
}

void sendRpmCan(void){
    static byte ret = 0;
    ret=CAN1.sendMsgBuf(mEEC1_ID_rpm,CAN_EXTID,mEEC1_DLC,mEEC1_data_rpm);
    if (ret==CAN_OK) {
        //
    } else if (ret == CAN_SENDMSGTIMEOUT) {
        Serial.println ("ECM: Message Timeout (rpm)");
    } else {
        Serial.println("ECM: Error to send (rpm)");
    }
}

void sendSpeedCan(void){
    static byte ret = 0;
    
    ret=CAN1.sendMsgBuf(mEEC1_ID_speed,CAN_EXTID,mEEC1_DLC,mEEC1_data_speed);
    
    if (ret==CAN_OK) {
        //
    } else if (ret == CAN_SENDMSGTIMEOUT) {
        Serial.println ("ECM: Message Timeout (Speed)");
    } else {
        Serial.println("ECM: Error to send (Speed)");
    }
}

int elapsedTime = 0;

TASK(interfaceCAN){
    GetResource(infoCAN);
    readCAN();
    
    // Envia a rotação a cada 100ms e a velocidade a cada 250ms
    elapsedTime += 50;
    if(elapsedTime % 100 == 0){
        sendRpmCan();
    }
    if(elapsedTime % 250 == 0){
        sendSpeedCan();
    }
    if(elapsedTime >= 500){
        elapsedTime = 0;
    }
    ReleaseResource(infoCAN);

    TerminateTask();
}