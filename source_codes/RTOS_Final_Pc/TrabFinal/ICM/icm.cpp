// Universidade de Brasilia (UnB)
// Curso: Engenharia Automotiva
// Disciplina : Engenharia de Software Automotivo
//////////////////////////////////////////////////////////////////////////////////

#include "tpl_os.h"
#include "Arduino.h"
#include "board_v2.h"
#define mEEC1_ID_ngear 0x18F00503
#define mEEC1_ID_rpm   0x0CF00400
#define mEEC1_ID_speed 0x18FEF100  

#define resolution_ngear    1 
#define resolution_rpm      8 // 
#define resolution_speed    1.0/256.0  

#define offset_ngear    -125 
#define offset_rpm      0 
#define offset_speed    0 

//#define ms_ngear    0
#define ls_ngear    3
#define ms_rpm      4
#define ls_rpm      3
#define ms_speed    2
#define ls_speed    1


// Informações pra Serial
unsigned int n_gear = 1; // Marcha mínima proposta = 1
unsigned int rpm    = 0;
static float speed  = 0;

//Variaveis para armazenar informacoes do frame recebido
long unsigned int mID;
unsigned char mDLC = 0;
unsigned char mDATA[8];

unsigned int data_ngear = 0;
unsigned int data_rpm = 0;
unsigned int data_speed = 0;

char msgString[128];

//Constroi um objeto MCP_CAN e configura o chip selector para o pino 10.
MCP_CAN CAN1(ECU4_CAN1_CS);

void setup()
{
    // Inicializa a interface serial : baudrate = 115200
    Serial .begin(115200);

    // Inicializa o controlador can : baudrate = 500K, clock=08MHz
    while(CAN1.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK){
        delay(200);
    }
    Serial.println ("Modulo CAN inicializado!");

    CAN1.setMode(MCP_NORMAL);

    //Configura o pino de interrupção para recepção
    pinMode(ECU4_CAN2_INT, INPUT);
    Serial.println ("MCP2515 exemplo can_receive...");
    Serial.print ("ID\t\tType\tDLC\tByte0\tByte1\tByte2");
    Serial.println ("\tByte3\tByte4\tByte5\tByte6\tByte7");
}


TASK(interfaceCAN){
    if (!digitalRead(ECU4_CAN2_INT)){
        //Lê os dados: mID = identificador, mDLC = comprimento, mDATA = dados do frame
        CAN1.readMsgBuf(&mID, &mDLC, mDATA);
        
        // Captura mensagem de n_gear
        if((mID & CAN_EXTENDED_ID) == mEEC1_ID_ngear){
            GetResource(infoCAN);
            data_ngear = mDATA[ls_ngear];
            ReleaseResource(infoCAN);

            //n_gear = data_ngear * resolution_ngear + offset_ngear;
            //if (n_gear < 1 || n_gear > 5) n_gear = 1; // 
        }

        // Captura mensagem de rpm
        if((mID & CAN_EXTENDED_ID) == mEEC1_ID_rpm){
            GetResource(infoCAN);
            data_rpm = mDATA[ls_rpm] + (mDATA[ms_rpm] << 8);
            ReleaseResource(infoCAN);

            //rpm = data_rpm * resolution_rpm + offset_rpm; // 
        }

        // Captura mensagem de speed
        if((mID & CAN_EXTENDED_ID) == mEEC1_ID_speed){
            GetResource(infoCAN);
            data_speed = mDATA[ls_speed] + (mDATA[ms_speed] << 8);
            ReleaseResource(infoCAN);

            //speed = data_speed * resolution_speed + offset_speed; // 
        }

    }

    TerminateTask();
}

TASK(interfaceProcess){
    GetResource(infoCAN);
    unsigned int temp_ngear = data_ngear * resolution_ngear + offset_ngear;
    unsigned int temp_rpm = data_rpm * resolution_rpm + offset_rpm; 
    float temp_speed = data_speed * resolution_speed + offset_speed; //
    ReleaseResource(infoCAN);

    GetResource(infoSerial);
    if(temp_ngear >= 1 && temp_ngear <= 5){
        n_gear = temp_ngear;
    }

    if(temp_rpm >= 0 && temp_rpm <= (3200+800*n_gear)){
        rpm = temp_rpm;
    }

    if(temp_speed >= 0 && temp_speed <= 250){
        speed = temp_speed;
    }
    ReleaseResource(infoSerial);

    TerminateTask();
}

TASK(interfaceSerial){
    GetResource(infoSerial);

    Serial.print("Rotacao = "); Serial.print(rpm); Serial.print(" RPM");
    Serial.print(", Velocidade = "); Serial.print(speed); Serial.print(" km/h");
    Serial.print(", Marcha atual = "); Serial.print(n_gear); Serial.println();

    ReleaseResource(infoSerial);

    TerminateTask();
}

