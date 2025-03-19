#include "tpl_os.h"
#include "Arduino.h"
#include "board_v2.h"

//Macro para definicoes
#define mEEC1_ID_ngear 0x18F00503
#define mEEC1_DLC 8
#define mEEC1_EXT_FRAME 1
#define resolution_ngear 1 // 1 bit vira um mesmo, né?
#define offset_ngear -125
#define pos 4 -1
#define marcha_menor 1
#define marcha_maior 5


//FRAME_DATA
unsigned char mEEC1_data_ngear[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Resource currGearData

static unsigned int n_gear = 1; // Resource currNGear

//Constroi um objeto MCP_CAN e configura o chip selector para o pino definido no board.h para ECU2_CAN1_CS.
MCP_CAN CAN1(ECU2_CAN1_CS);

void setup(){
    Serial .begin(115200);
    // Inicializa o controlador can : baudrate = 250K, clock=8MHz
    while(CAN1.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK){
        delay(200);
    }
    Serial.println ("MCP2515 can_send inicializado com sucesso!");
    CAN1.setMode(MCP_NORMAL);
}

TASK(serialGear){
    if(Serial.available() > 0){ // Padrão básico -> Como estou lendo na Serial, simplifiquei o formato da mensagem
        char nextGear[4]; 
        Serial.readBytes(nextGear, 4);

        GetResource(currNGear);
        if(nextGear[0] == '-' && nextGear[1] == 'n' && nextGear[2] == 'g'){
            if((nextGear[3] - '0') >= marcha_menor && (nextGear[3] - '0') <= marcha_maior){
                n_gear = (nextGear[3] - '0');
            }
        }
        ReleaseResource(currNGear);
        
        String flushing = Serial.readString(); // "Limpa" a Serial
    }

    TerminateTask();
}

TASK(interfaceProcessing){
    // Formação de Frame: Pos 4; Início 1; 8 bits; Res: 1bit/marcha; faixa -125 a +125; offset -125
    // data = Valor/Res - (offset) = (Valor/1) - (-125)
    GetResource(currNGear);
    int data_ngear = (n_gear - offset_ngear);
    ReleaseResource(currNGear);

    GetResource(currGearData);
    mEEC1_data_ngear[pos] = data_ngear;
    ReleaseResource(currGearData);

    TerminateTask();
}

// Envia o pacote pela CAN 1
TASK(sendGearCAN){
    static byte ret = 0;

    GetResource(currGearData);
    ret=CAN1.sendMsgBuf(mEEC1_ID_ngear,CAN_EXTID,mEEC1_DLC,mEEC1_data_ngear);
    ReleaseResource(currGearData);
    
    if (ret==CAN_OK) {
        //Serial.print("n"); Serial.println(n_gear);
    } else if (ret == CAN_SENDMSGTIMEOUT) {
        Serial.println ("TCM: Message Timeout");
    } else {
        Serial.println("TCM: Error to send");
    }
    
    TerminateTask();
}

