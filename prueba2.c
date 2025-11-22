//*****************************************************************************
//
// PROYECTO MONITORIZACION AMBIENTAL EN ENTORNO CERRADOS
//

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib2.h"
#include "utils/locator.h"
#include "utils/lwiplib.h"
#include "utils/ustdlib.h"
#include "utils/uartstdio.h"
#include "httpserver_raw/httpd.h"
#include "drivers/pinout.h"
#include "lwip/tcp.h"
#include "lwip/ip_addr.h"
#include "lwip/err.h"
#include <math.h>

#include "HAL_I2C.h"
#include "sensorlib2.h"
#include "FT800_TIVA.h"

#define B1_OFF GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_0)
#define B1_ON !(GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_0))
#define min_espera 1

//Sensor ENS160
#define ENS160_ADDR  0x52   // ADD=GND
uint16_t ens_TVOC = 0;
uint16_t ens_ECO2 = 0;
uint16_t ens_AQI = 0;
bool Ens_OK = false;
//SENSOR BOOSTERPACK
uint8_t Opt_OK, Tmp_OK, Bme_OK, Bmi_OK;
uint8_t Sensor_OK=0;
float T_act,P_act,H_act;
float lux;
char string[80];
int DevID=0;
int16_t T_amb, T_obj;
float Tf_obj, Tf_amb;
int lux_i, T_amb_i, T_obj_i;

// BME280
int returnRslt;
int g_s32ActualTemp   = 0;
unsigned int g_u32ActualPress  = 0;
unsigned int g_u32ActualHumity = 0;
// Defines SLEEP
#define SLEEP SysCtlSleepFake()
//#define SLEEP SysCtlSleep()
volatile int Flag_ints = 0;
volatile bool g_bPendingSend = false;

// === INSTRUMENTOS ===
#define PIANO       70
#define SIRENA      1
#define CAMPANA     73
#define ALARMA      6
#define XILOFONO    65
#define TROMPETA    69
#define SILENCIO    0

// === NOTAS IMPORTANTES ===
#define DO_CENTRAL  60
#define RE          62
#define MI          64
#define FA          65
#define SOL         67
#define LA          69
#define SI          71
#define DO_AGUDO    72

// === NOTAS
#define LA_AGUDA    81  //  BIP agudo
#define LA_BAJA     57  // BIIIIP grave

// Defines lwIP/ESP32
#define SYSTICKHZ               100
#define SYSTICKMS               (1000 / SYSTICKHZ)

#define SYSTICK_INT_PRIORITY    0x80
#define ETHERNET_INT_PRIORITY   0xC0

unsigned long REG_TT[6];
const int32_t REG_CAL[6]= {CAL_DEFAULTS};

uint32_t g_ui32IPAddress;
uint32_t g_ui32SysClock;
volatile bool g_bLED;
int i=0;


struct tcp_pcb *g_pcb = NULL;
uint8_t g_bConnected = 0;  // Opcional ahora (no esperamos)

char g_pcPostData[512];
char g_pcMessage[128] = "Hola DESDE TIVA";

#define ESP_IP_A 192
#define ESP_IP_B 168
#define ESP_IP_C 100
#define ESP_IP_D 50
#define ESP_PORT 80

int pasos_restantes=0;
int nota_actual=0;
int indice_nota = 0;

int temporizador_notas[] = {
    LA_AGUDA,    // BIP
    LA_AGUDA,    // BIP
    LA_AGUDA,    // BIP
    LA_BAJA,     // BIIIIP (grave largo)
    0            // Silencio (fin)
};
int temporizador_duraciones[] = {
    1,  // BIP 0.5s
    1,  // BIP 0.5s
    1,  // BIP 0.5s
    4,  // BIIIIP 2s (largo para aviso)
    2   // Silencio 1s
};
int temporizador_length = 5;  // NUmero de elementos

//Mensajes de alarma
char t_tempa[20] = "HIGH TEMPERATURE";
char t_tempb[20] = "LOW TEMPERATURE";
char t_presa[20] = "HIGH PRESSURE";
char t_presb[20] = "LOW PRESSURE";
char t_humeda[20] = "HIGH HUMIDITY";
char t_humedb[20] = "LOW HUMIDITY";
char t_co2a[40] = "CO2 ABOVE LIMITS";
char calidad[30]="";
char t_timer[30]="";


char settings;
char ok;
char set[5];
char back;

#define H1  20
#define anchoset 200
#define Hgrad 180
#define XT  HSIZE*1/5-50
#define XP  HSIZE*2/5-50
#define XH  HSIZE*3/5-30
#define XC  HSIZE*4/5
int numero;

int alarma=0;
int timer=0;
//Chars para mostrar en pantalla
char luz[30]="";
char temp[30]="";
char hum[30]="";
char pres[30]="";
char bares_char[30]="";
char numero_char[30]="";
char t_tvoc[30]="";
char t_co2[30]="";
char t_aqui[30]="";

//ESTADOS
typedef enum {
    p_ppal, //PRINCIPAL
    p_set,  //SETTINGS
    p_pad1, //PARA DEFINIR VALOR MINIMO DE MEDIDAS
    p_pad2, //PARA DEFINIR VALOR MAXIMO DE MEDIDAS
    p_pad_prev,
    p_descont, //PARA QUE NO SE ENVIE MENSAJE POR TELEGRAM CONSTATEMENTE
    p_alarma, //AVISO DE ALARMA PORQUE SE HA SALIDO DEL RANGO ESTABLECIDO DE ALGUNA VARIABLE
    p_sendpost, //ENVIA POST REQUEST A ESP32
    p_temporizador  //SUENA AVISO DE FIN DE TEMPORIZADOR
}Estado;
Estado estado;
Estado estado_ant;

int Ttim, Tmax, Pmax, Hmax, Tmin, Pmin, Hmin;

int T_uncomp,T_comp;
char mode;
long int inicio, tiempo;
int t1=0;
int time_wait=0;
int ts=0;

int i_var;//indice para recorrer los vectores de variables
int aqi=1;
int Ttim, Tmax, Pmax, Hmax, Tmin, Pmin, Hmin,Co2min,Co2max;
int* valores_min[]={&Tmin,&Pmin,&Hmin,&Co2min,&Ttim};
int* valores_max[]={&Tmax,&Pmax,&Hmax,&Co2max};
char* textos_min[]={"floor limit:\n %d C \n","floor limit:\n %d mbar \n","floor limit:\n %d %% \n",
                    "floor limit:\n %d ppm \n","set timer:\n %d min \n"};
char* textos_max[]={"ceiling limit:\n %d C \n","ceiling limit:\n %d mbar \n","ceiling limit:\n %d %% \n",
                    "ceiling limit:\n %d ppm \n",};

bool b_alarma;
bool b_wifi;

void ShowStateIP(void)
{
    uint32_t link_up = EMACPHYRead(EMAC0_BASE, 0, EPHY_BMSR) & EPHY_BMSR_LINKSTAT;

    if (link_up)
    {
        // HAY CABLE ENCHUFADO → CONECTADO (aunque la IP sea estática)
        ComColor(0,255,0);
        ComTXT(HSIZE-anchoset/3, VSIZE*3/5, 26, OPT_CENTERX, "CONNECTED");

    }
    else
    {
        // NO CONECTADO
        ComColor(255,0,0);
        ComTXT(HSIZE-anchoset/3, VSIZE*3/5, 26, OPT_CENTERX, "NOT CONNECTED");
    }
}

bool ENS160_Init(void)
{
    uint8_t part_id[2];

    // 1. Leer PART_ID
    if (!readI2C(ENS160_ADDR, 0x00, part_id, 2))
    {
        UARTprintf("Error de comunicacion I2C al leer ID\n");
        return false;
    }


    UARTprintf("\nDEBUG ENS160 ID: [0]=0x%x, [1]=0x%x \n", part_id[0], part_id[1]);

    // Debe ser: part_id[0] == 0x60 y part_id[1] == 0x01
    if (part_id[0] != 0x60 || part_id[1] != 0x01)
    {
        UARTprintf("ID Incorrecto. Esperado: 60 01. Leido: %x %x\n", part_id[0], part_id[1]);
        return false;
    }

    // Espera de arranque del sensor
    SysCtlDelay(g_ui32SysClock / 3000 * 1000);

    // 2. Poner en STANDARD mode (OPMODE_STD = 0x02) en registro OPMODE (0x10)
    uint8_t mode = 0x02;
    writeI2C(ENS160_ADDR, 0x10, &mode, 1);

    SysCtlDelay(g_ui32SysClock / 3000 * 100);

    Ens_OK = true;
    return true;
}


void ENS160_Read(void)
{
    uint8_t status;
    uint8_t data[8];
    uint8_t aqi;
    readI2C(ENS160_ADDR, 0x20, &status, 1);
    readI2C(ENS160_ADDR, 0x22, data, 8);
    uint8_t validity = (status >> 2) & 0x03;
    readI2C(ENS160_ADDR, 0x21, &aqi, 1);

        ens_TVOC = data[0] | (data[1] << 8);   // <--- Â¡SWAP AQUÃ�! MSB primero
        ens_ECO2 =  data[2] | (data[3] << 8);   // <--- Â¡SWAP AQUÃ�! MSB primero
        ens_AQI = aqi;


        sprintf(t_tvoc, "%d",ens_TVOC);
        sprintf(t_co2, "%d ppm",ens_ECO2);
        sprintf(t_aqui, "%d",ens_AQI);
        UARTprintf("Validity: %d | AQI: %d (1=Excellent ... 5=Unhealthy) | TVOC: %d ppb | eCO2: %d ppm\n",
                      validity, ens_AQI, ens_TVOC, ens_ECO2);
}
void PlaySirenStep(void)
{
    VolNota(127);
    if (pasos_restantes == 0)
    {
        if (nota_actual == 0) {
            TocaNota(1, 69); pasos_restantes = 1;
        }
        else if (nota_actual == 1) {
            TocaNota(1, 57); pasos_restantes = 4;
        }
        else {
            // === FIN DE CICLO: RESET PARA REPETIR ===
            TocaNota(0, 0);  // Silencio breve
            nota_actual = 0;
            pasos_restantes = 0;

            return;
        }
        nota_actual++;
    }
    pasos_restantes--;
}
void TocaTemporizadorStep(void)
{

    if (pasos_restantes == 0)
    {
        // === Â¿FIN DE LA MELODÃ�A? ===
        if (temporizador_notas[indice_nota] == 0)
        {
            TocaNota(SILENCIO, 0);  // Silencio final
            indice_nota = 0;
            pasos_restantes = 0;

        }

        // === TOCAR NUEVA NOTA ===
        TocaNota(XILOFONO, temporizador_notas[indice_nota]);

        // === DURACIÃ“N (en pasos de 500 ms) ===
        pasos_restantes = temporizador_duraciones[indice_nota];
        indice_nota++;
    }

    // === RESTAR UN PASO ===
    pasos_restantes--;
}

// Funciones SLEEP/Timer0
void SysCtlSleepFake(void)
{
    while(!Flag_ints);
    Flag_ints = 0;
}

void Timer0IntHandler(void)
{
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    Flag_ints = 1;
    if(estado == p_descont || timer==1 ){
        t1++;
        if(t1>=4){
            ts++; // segundos
            t1=0;
        }
        if(ts>=60){
            ts=0;
            time_wait++; //minutos
        }
    }else {
        t1=0;
        ts=0;
        time_wait=0;
    }

}
void lwIPHostTimerHandler(void)
{

    uint32_t ui32NewIPAddress = lwIPLocalIPAddrGet();
    if(ui32NewIPAddress != g_ui32IPAddress) {
        g_ui32IPAddress = ui32NewIPAddress;  // Actualiza global (sin prints)

    }
}
// Sensores
void ReadSensors(void)
{

    if(Opt_OK)
    {
        lux=OPT3001_getLux();
        lux_i=(int)round(lux);
    }

    if(Bme_OK)
    {
        returnRslt = bme280_read_pressure_temperature_humidity(
                &g_u32ActualPress, &g_s32ActualTemp, &g_u32ActualHumity);
        T_act=(float)g_s32ActualTemp/100.0;
        P_act=(float)g_u32ActualPress/100.0;
        H_act=(float)g_u32ActualHumity/1000.0;
    }
    if(Ens_OK)
    {
        ENS160_Read();
    }


    if(Opt_OK)
    {
        UARTprintf("\033[10;1H----------------------------\n");
        sprintf(string,"  OPT3001: %5.3f Lux   \n",lux);
        UARTprintf(string);
    }

    if(Bme_OK)
    {
        UARTprintf("-----------------------------------------\n");
        sprintf(temp, "%.2f C \n",T_act);
        sprintf(pres, "%.2f mbar \n",P_act);
        sprintf(hum, "%.2f %% \n",H_act);
        sprintf(string, "  BME: T:%.2f C  P:%.2fmbar  H:%.3f  \n",T_act,P_act,H_act);
        UARTprintf(string);
    }
    if(Ens_OK)
    {
        sprintf(string, " ENS160: TVOC %5u ppb   eCO2 %5u ppm\n", ens_TVOC, ens_ECO2);
        UARTprintf(string);
    }


    UARTprintf("------------------------------------------------\n");
}

// lwIP/ESP32 simplificadas
#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line) {}
#endif

void DisplayIPAddress(uint32_t ui32Addr)
{
    char pcBuf[16];
    usprintf(pcBuf, "%d.%d.%d.%d", ui32Addr & 0xff, (ui32Addr >> 8) & 0xff,
             (ui32Addr >> 16) & 0xff, (ui32Addr >> 24) & 0xff);
    UARTprintf(pcBuf);
}

void SysTickIntHandler(void)
{
    lwIPTimer(SYSTICKMS);
}

err_t ESPConnected(void *arg, struct tcp_pcb *tpcb, err_t err)
{
    if (err == ERR_OK) {
        g_bConnected = 1;
        UARTprintf("Conectado a ESP32.\n");


        if (g_bPendingSend) {
            g_bPendingSend = false;  // limpiar bandera
            g_pcb = tpcb;            // asegurar PCB activo
            ESPSendPost();           // lanzar envIO
        }
    } else {
        UARTprintf("Connect failed: %d\n", err);
    }
    return ERR_OK;
}



err_t ESPRecv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    if (p != NULL) {
        pbuf_free(p);
    }
    return ERR_OK;
}

err_t ESPSent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
    UARTprintf("Enviado OK (%u bytes).\n", len);
    return ERR_OK;
}

void ESPError(void *arg, err_t err)
{
    // Silencio: No log, no close. Deja que lwIP maneje.
}
void ESPSendPost(void)
{
    if (!g_pcb) {
        UARTprintf("No PCB, conectando y dejando envio pendiente...\n");
        g_bPendingSend = true;  // <-- marcar que hay envio pendiente
        ESPInit();              // inicia conexion
        return;
    }

    // Si ya hay conexion activa, enviar directo
    int len = usprintf(g_pcPostData,
        "POST /send HTTP/1.1\r\n"
        "Host: %d.%d.%d.%d\r\n"
        "Content-Type: application/x-www-form-urlencoded\r\n"
        "Content-Length: %d\r\n"
        "\r\n"
        "text=%s",
        ESP_IP_A, ESP_IP_B, ESP_IP_C, ESP_IP_D,
        (int)strlen(g_pcMessage), //se envia mensaje segun alerta
        g_pcMessage);

    UARTprintf("Enviando POST (%d bytes): %s\n", len, g_pcMessage);

    err_t e = tcp_write(g_pcb, g_pcPostData, len, TCP_WRITE_FLAG_COPY);
    if (e == ERR_OK) {
        e = tcp_output(g_pcb);
        if (e == ERR_OK) {
            UARTprintf("POST enviado! Cerrando...\n");
            tcp_close(g_pcb);
            g_pcb = NULL;
            g_bConnected = 0;
        }
    }
}

void ESPInit(void)
{
    if(g_pcb) return;

    g_pcb = tcp_new();
    if(g_pcb) {
        ip_addr_t dest;
        IP4_ADDR(&dest, ESP_IP_A, ESP_IP_B, ESP_IP_C, ESP_IP_D);
        tcp_bind(g_pcb, IP_ADDR_ANY, 0);
        tcp_connect(g_pcb, &dest, ESP_PORT, ESPConnected);
        tcp_recv(g_pcb, ESPRecv);
        tcp_sent(g_pcb, ESPSent);
        tcp_err(g_pcb, ESPError);
        UARTprintf("Conectando a ESP32...\n");
    } else {
        UARTprintf("tcp_new failed.\n");
    }
}




int main(void)
{
    uint32_t ui32User0, ui32User1;
        uint8_t pui8MACArray[6];

        // 1. Reloj del sistema
        SysCtlMOSCConfigSet(SYSCTL_MOSC_HIGHFREQ);
        g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                SYSCTL_USE_PLL | SYSCTL_CFG_VCO_240), 120000000);



        // 2. Habilitar perifericos de la placa base (LEDs, UART, Ethernet)
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);




        // 3. Configurar Pines de la placa base (Ethernet y UART)
        GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0 |GPIO_PIN_4);
        GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0 |GPIO_PIN_1);

        // Configuracion UART
        UARTStdioConfig(0, 115200, g_ui32SysClock);
        UARTprintf("\033[2J\033[H");
        UARTprintf("Tiva to ESP32... \n");

        // 4. Configurar Botones
        GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0|GPIO_PIN_1);
        GPIOPadConfigSet(GPIO_PORTJ_BASE,GPIO_PIN_0|GPIO_PIN_1,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);

        // 5. Inicializar LwIP (Ethernet)
        // Obtener MAC
        FlashUserGet(&ui32User0, &ui32User1);
        if((ui32User0 == 0xffffffff) || (ui32User1 == 0xffffffff)) {
            UARTprintf("No MAC!\n");
            while(1) {}
        }
        pui8MACArray[0] = ((ui32User0 >>  0) & 0xff);
        pui8MACArray[1] = ((ui32User0 >>  8) & 0xff);
        pui8MACArray[2] = ((ui32User0 >> 16) & 0xff);
        pui8MACArray[3] = ((ui32User1 >>  0) & 0xff);
        pui8MACArray[4] = ((ui32User1 >>  8) & 0xff);
        pui8MACArray[5] = ((ui32User1 >> 16) & 0xff);

        // IP estÃ¡tica
        uint32_t ui32IP = (192u<<24)|(168u<<16)|(100u<<8)|110u;
        uint32_t ui32NetMask = (255u<<24)|(255u<<16)|(255u<<8)|0u;
        uint32_t ui32Gateway = (192u<<24)|(168u<<16)|(100u<<8)|1u;
        lwIPInit(g_ui32SysClock, pui8MACArray, ui32IP, ui32NetMask, ui32Gateway, IPADDR_USE_STATIC);

        g_ui32IPAddress = lwIPLocalIPAddrGet();
        UARTprintf("IP Asignada. Iniciando Timers...\n");

        // 6. Timers y SysTick
        SysTickPeriodSet(g_ui32SysClock / SYSTICKHZ);
        SysTickEnable();
        SysTickIntEnable();

        // Timer0 SLEEP (500ms)
          SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
          TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
          TimerLoadSet(TIMER0_BASE, TIMER_A, g_ui32SysClock / 4 - 1);
          TimerIntRegister(TIMER0_BASE, TIMER_A, Timer0IntHandler);
          IntEnable(INT_TIMER0A);
          TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
          TimerEnable(TIMER0_BASE, TIMER_A);


            GPIOPinConfigure(GPIO_PA0_U0RX);
            GPIOPinConfigure(GPIO_PA1_U0TX);
            GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

            UARTStdioConfig(0, 115200, g_ui32SysClock);

          LocatorInit();
          LocatorMACAddrSet(pui8MACArray);
          LocatorAppTitleSet("Tiva ESP32 Send");
          IntPrioritySet(INT_EMAC0, ETHERNET_INT_PRIORITY);
          IntPrioritySet(FAULT_SYSTICK, SYSTICK_INT_PRIORITY);
          IntEnable(INT_EMAC0);
          IntMasterEnable();


          SysCtlDelay(g_ui32SysClock / 10);

          UARTprintf("Configurando BoosterPack...\n");
         Conf_Boosterpack(1, g_ui32SysClock);    // Configura I2C0 en PB2/PB3

          SysCtlDelay(g_ui32SysClock / 3000 * 1500);
          HAL_Init_SPI(2,g_ui32SysClock);         // Configura SPI en Slot 2

         Inicia_pantalla();
          SysCtlDelay(g_ui32SysClock/3);

          Nueva_pantalla(169,125,70);
          Dibuja();
          for ( i = 0; i < 6; i++) Esc_Reg(REG_TOUCH_TRANSFORM_A + 4 * i, REG_CAL[i]);



    //Inicializamos sensores
    UARTprintf("\033[2J \033[1;1H Inicializando OPT3001... ");
    Sensor_OK=Test_I2C_Dir(OPT3001_SLAVE_ADDRESS);
    if(!Sensor_OK)
    {
        UARTprintf("Error en OPT3001\n");
        Opt_OK=0;

    }
    else
    {
        OPT3001_init();
        UARTprintf("Hecho!\n");
        UARTprintf("Leyendo DevID... ");
        DevID=OPT3001_readDeviceId();
        UARTprintf("DevID= 0X%x \n", DevID);
        Opt_OK=1;
    }

    UARTprintf("Inicializando BME280... ");
    Sensor_OK=Test_I2C_Dir(BME280_I2C_ADDRESS2);
    if(!Sensor_OK)
    {
        UARTprintf("Error en BME280\n");
        Bme_OK=0;
    }
    else
    {
        bme280_data_readout_template();
        bme280_set_power_mode(BME280_NORMAL_MODE);
        UARTprintf("Hecho! \nLeyendo DevID... ");
        readI2C(BME280_I2C_ADDRESS2,BME280_CHIP_ID_REG, &DevID, 1);
        UARTprintf("DevID= 0X%x \n", DevID);
        Bme_OK=1;
    }
    UARTprintf("Inicializando ENS160... ");
    Sensor_OK = Test_I2C_Dir(ENS160_ADDR);
    if(!Sensor_OK)
    {
        UARTprintf("Error en ENS160\n");
        Ens_OK = false;
    }
    else
    {
        if(ENS160_Init())
            UARTprintf("Hecho! PART_ID OK\n");
        else
        {
            UARTprintf("FallÃ³ PART_ID\n");
            Ens_OK = false;
        }
    }

    //Limites
    Tmax=35;
    Tmin=20;
    Pmax=1030;
    Pmin=1000;
    Co2min=600;
    Co2max=1500;
    Hmax=60;
    Hmin=40;
    Ttim=10;


    while(1) {
        SLEEP;
        Nueva_pantalla(16,16,16);
        ComColor(160, 160, 160);
        ComRect(12, 12, HSIZE-12, VSIZE-12, true);
        ComColor(0,0,0);
        ReadSensors();
        switch(estado){

        case p_ppal:

            //TEMPERATURA
            if(T_act<Tmin){
                ComColor(0,0,255);
                ComBgcolor(0,0,255);
                alarma = 1;
                strcpy(g_pcMessage,t_tempb);

            }else if(T_act>Tmax){
                ComColor(255,0,0);
                ComBgcolor(255,0,0);
                alarma = 1;
                strcpy(g_pcMessage,t_tempa);
            }else{
                ComColor(0,0,0);
                ComBgcolor(0,0,0);
            };

            ComCirculo(XT+4,H1+110,10);
            ComColor(240,240,240);
            ComTXT(XT+10,H1+140, 22, OPT_CENTERX,temp);
            ComColor(255,255,255);

            ComProgbar(XT, H1, 8, 100, 0, 50-(int)T_act, 40,50);

            ComColor(255,255,255);
            ComTXT(XT,H1+120, 22, OPT_CENTERX,"T:");

            //PRESION
            if(P_act<Pmin) {
                ComBgcolor(0,0,255);
                strcpy(g_pcMessage,t_presb);
                alarma = 1;
            }
            else if(P_act>Pmax) {
                ComBgcolor(255,0,0);
                strcpy(g_pcMessage,t_presa);
                alarma = 1;
            }
            else ComBgcolor(240,240,240);
            ComTXT(XP+10,H1+130, 22, OPT_CENTERX,pres);
            ComColor(0,0,0);
            int P_act_desp=P_act-950;
            ComGauge(XP, H1+50, 50, OPT_FLAT, 5, 4, (int)P_act_desp, 100);
            ComColor(255,255,255);
            ComTXT(XP,H1+110, 22, OPT_CENTERX,"P:");

            //HUMEDAD
            if(H_act<Hmin){
                ComBgcolor(0,0,255);
                 strcpy(g_pcMessage,t_humedb);
                alarma = 1;
            }
            else if(H_act>Hmax){
                ComBgcolor(255,0,0);
                strcpy(g_pcMessage,t_humeda);
                alarma=1;
            }
            else ComBgcolor(240,240,240);
            ComTXT(XH+10,H1+130, 22, OPT_CENTERX,hum);
            ComColor(0,0,0);
            ComGauge(XH, H1+50, 50, OPT_FLAT, 5, 4, (int)H_act, 100);
            ComColor(255,255,255);
            ComTXT(XH,H1+110, 22, OPT_CENTERX,"H:");

          //  c02
            if(ens_ECO2<Co2min) {
                ComBgcolor(0,0,255);

            }
            else if(ens_ECO2>Co2max){
                ComBgcolor(255,0,0);
                strcpy(g_pcMessage,t_co2a);
                alarma=1;
            }
            else ComBgcolor(255,255,0);
            ComTXT(XC+10,H1+130, 22, OPT_CENTERX,t_co2);
            ComColor(0,0,0);
            int Co2_desp=ens_ECO2-400;
            ComGauge(XC, H1+50, 50, OPT_FLAT, 5, 4, (int)Co2_desp, 1600);
            ComColor(255,255,255);
            ComTXT(XC,H1+110, 22, OPT_CENTERX,"CO2:");

            //CALIDAD DEL AIRE
            ComTXT(XP,VSIZE-30, 22, OPT_CENTERX,"AIR QUALITY:");
            switch (ens_AQI){
            case 1:
                ComColor(134, 216, 218);
                strcpy(calidad, "EXCELLENT");
                ComTXT(XH,VSIZE-30, 22, OPT_CENTERX,calidad);
                break;
            case 2:
                ComColor(189, 220, 81);
                strcpy(calidad, "GOOD");
                ComTXT(XH,VSIZE-30, 22, OPT_CENTERX,calidad);
                break;
            case 3:
                ComColor(242, 192, 5);
                strcpy(calidad, "MODERATE");
                ComTXT(XH,VSIZE-30, 22, OPT_CENTERX,calidad);
                break;
            case 4:
                ComColor(249, 159,8);
                strcpy(calidad, "POOR");
                ComTXT(XH,VSIZE-30, 22, OPT_CENTERX,calidad);
                break;
            case 5:
                ComColor(241, 77, 77);
                strcpy(calidad, "UNHEALTHY");
                ComTXT(XH,VSIZE-30, 22, OPT_CENTERX,calidad);
                break;
            }


            ComFgcolor(200,50,50);
            ComColor(250,250,250);
            settings=Boton(HSIZE-100,  VSIZE-60,  80,  40,  26, "SETTINGS");
            //timer
           if(timer){

                ComColor(0,0,127);
                ComTXT(HSIZE-90,VSIZE-90, 22, OPT_CENTERX,"TIMER: ");
                ComColor(0,0,0);
                sprintf(t_timer,"%d",time_wait);
                ComTXT(HSIZE-40,VSIZE-90, 22, OPT_CENTERX,t_timer);

           }


            if(alarma && b_alarma) estado=p_alarma;
            else if(alarma && b_wifi)estado=p_sendpost;
            else if(timer && time_wait>=Ttim)estado=p_temporizador;
            else if(settings) estado=p_set;
            else estado=p_ppal;
            break;

        case p_set:
            ShowStateIP(); //Muestra si el ethernet esta enchufado o no
            set[0] = 0;
            set[1] = 0;
            set[2] = 0;
            set[3] = 0;
            set[4] = 0;
            ComFgcolor(255,0,0);
            ComColor(0,0,0);
            set[0]=Boton(HSIZE/2- anchoset/2,VSIZE*2/7,  anchoset,  30,  26, "SET TEMPERATURE");
            set[1]=Boton(HSIZE/2- anchoset/2,VSIZE*3/7, anchoset,  30,  26, "SET PRESSURE");
            set[2]=Boton(HSIZE/2- anchoset/2,VSIZE*4/7,  anchoset,  30,  26, "SET HUMIDITY");
            set[3]=Boton(HSIZE/2- anchoset/2,VSIZE*5/7,  anchoset,  30,  26, "SET CO2");
            set[4]=Boton(HSIZE/2- anchoset/2,VSIZE/7,  anchoset,  30,  26, "SET TIMER");
            back=Boton(HSIZE/2- anchoset/4,  VSIZE*6/7,  anchoset/2,  30,  26, "BACK");

            if (b_alarma)  ComFgcolor(0,255,0);      // verde si está ON
            else           ComFgcolor(250,0,0);  // gris si está OFF

            if (Boton(25, VSIZE*3/7, anchoset/2, 30, 26, "ALARM"))
            {
                if (b_alarma) {
                    // Si ya estaba ON → lo apagamos (permitimos desactivar)
                    b_alarma = false;
                }
                else {
                    // Solo se permite activar si WIFI está apagado
                    if (!b_wifi) {
                        b_alarma = true;
                    }
                }
            }

            // ====== BOTÓN WIFI ======
            if (b_wifi)  ComFgcolor(0,255,0);
            else         ComFgcolor(250,0,0);

            if (Boton(HSIZE-25-anchoset/2, VSIZE*3/7, anchoset/2, 30, 26, "WIFI"))
            {
                if (b_wifi) {
                    // Si ya estaba ON → lo apagamos
                    b_wifi = false;
                }
                else {
                    // Solo se permite activar si ALARMA está apagado
                    if (!b_alarma) {
                        b_wifi = true;
                        timer = 0; //no hay timer si alerta por wifi se activa
                    }
                }
            }

            if(alarma && b_alarma) estado=p_alarma;
            else if(alarma && b_wifi)estado=p_sendpost;
            else if(timer && time_wait>=Ttim)estado=p_temporizador;
            else if(back) estado=p_ppal;
            else if(set[0]||set[1]||set[2]||set[3]||set[4]) estado=p_pad1;
            else estado=p_set;
            break;

        case p_pad1:
            ComFgcolor(250,0,0);
            ComColor(255,255,255);
            int** valores=valores_min;
            char** textos=textos_min;
            for(i=0;i<5;i++) if(set[i]){i_var=i;break;}
            if(i_var<0)i_var=0;
            numero=*valores[i_var];
            sprintf(numero_char,textos[i_var],numero);
            ComTXT(HSIZE/2,VSIZE/2,22,OPT_CENTERX,numero_char);
         if(i_var<3 ){
            if(Boton(HSIZE*3/4-50,VSIZE/2-35,70,70,31,"+")) numero++,*valores[i_var]=numero;
            if(Boton(HSIZE*1/4-50,VSIZE/2-35,70,71,31,"-")) numero--,*valores[i_var]=numero;
         }else if(i_var==3){
             if(Boton(HSIZE*3/4-50,VSIZE/2-35,70,70,31,"+")) numero+=50,*valores[i_var]=numero;
             if(Boton(HSIZE*1/4-50,VSIZE/2-35,70,71,31,"-")) numero-=50,*valores[i_var]=numero;
         }else{
             if(Boton(HSIZE*3/4-50,VSIZE/2-35,70,70,31,"+")) numero+=5,*valores[i_var]=numero;
             if(Boton(HSIZE*1/4-50,VSIZE/2-35,70,71,31,"-")) numero-=5,*valores[i_var]=numero;
             timer=1;
             time_wait=0;
             ts = 0;
             t1 = 0;
         }

            if(Boton(HSIZE/2-20,VSIZE-60,50,50,26,"OK")){
                *valores[i_var]=numero;
                if(i_var==4) estado=p_set;
                else estado=p_pad2;
            }
            break;

        case p_pad2:
            ComFgcolor(250,0,0);
            ComColor(255,255,255);

            valores=valores_max;
            textos=textos_max;
            for(i=0;i<4;i++) if(set[i]){i_var=i;break;}
            if(i_var<0)i_var=0;
            numero=*valores[i_var];
            sprintf(numero_char,textos[i_var],numero);
            ComTXT(HSIZE/2-10,VSIZE/2,22,OPT_CENTERX,numero_char);
            if(i_var<3){
                if(Boton(HSIZE*3/4-50,VSIZE/2-35,70,70,31,"+")) numero++,*valores[i_var]=numero;
                if(Boton(HSIZE*1/4-50,VSIZE/2-35,70,71,31,"-")) numero--,*valores[i_var]=numero;
            }else{
                if(Boton(HSIZE*3/4-50,VSIZE/2-35,70,70,31,"+")) numero+=50,*valores[i_var]=numero;
                if(Boton(HSIZE*1/4-50,VSIZE/2-35,70,71,31,"-")) numero-=50,*valores[i_var]=numero;
            }

            if(Boton(HSIZE/2-20,VSIZE-60,50,50,26,"OK")) *valores[i_var]=numero,estado=p_set;
            if(alarma && b_alarma) estado=p_alarma;
            break;


        case p_alarma:
            PlaySirenStep();
            ok=Boton(HSIZE-100,  VSIZE-60,  80,  40,  26, "DONE");
            ComColor(0,0,0);
            ComTXT(HSIZE/2-10,VSIZE/2,31,OPT_CENTERX,"ALARM ACTIVE");
            if(ok) {
                estado=p_ppal;
                ok=0;
                alarma=0;
                b_alarma=false;
                TocaNota(SILENCIO, 0);


            }
            else estado=p_alarma;
            break;
        case p_sendpost:
            ESPSendPost();
            estado = p_descont;
            break;

        case p_descont:
            UARTprintf("\nEsperando tiempo para volver a enviar mensaje\n");
            ComColor(0,0,0);
            ComTXT(HSIZE/2-10,VSIZE/2,31,OPT_CENTERX,"MESSAGE SEND");
            ok=Boton(HSIZE-100,  VSIZE-60,  80,  40,  26, "DONE");
            if(time_wait>=min_espera){
                estado = p_ppal;
                alarma = 0;
            }
            if(ok){
                estado = p_ppal;
                b_wifi = false;
                alarma = 0;
            }
            break;
        case p_temporizador:
            TocaTemporizadorStep();
            ComColor(0,0,0);
            ComTXT(HSIZE/2-10,VSIZE/2,31,OPT_CENTERX,"TIMER FINISHED");
            ok=Boton(HSIZE-100,  VSIZE-60,  80,  40,  26, "DONE");
            if(ok) {
                estado=p_ppal;
                ok=0;
                timer=0;
                TocaNota(SILENCIO, 0);
            }
            break;

        }
        Dibuja();


    }

    return 0;
}
