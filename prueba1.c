//*****************************************************************************
//
// enet_lwip_esp32_fire_forget.c - Raw lwIP to ESP32: ENVIA Y YA (sin espera).
//
// Fixes:
// - ENVIA sin esperar g_bConnected (connect async).
// - QUITADO "Connection: close" + cierre manual solo si OK.
// - IGNORA ESPError/Recv (no cierra PCB en abort).
// - SIN lwIPHostTimerHandler() (IP estática).
// - Mensaje fijo, sin sensores.
// - Fix encoding: Strings sin acentos.
//
// Copyright (c) 2013-2020 Texas Instruments Incorporated.  All rights reserved.
//
//*****************************************************************************

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

#include "HAL_I2C.h"
#include "sensorlib2.h"
#include "FT800_TIVA.h"

#define B1_OFF GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_0)
#define B1_ON !(GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_0))

//Sensor ENS160
#define ENS160_ADDR  0x52   // ADD=GND
uint16_t ens_TVOC = 0;
uint16_t ens_ECO2 = 0;
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

// === NOTAS (agudas para bip) ===
#define LA_AGUDA    81  // A5 (880 Hz) → BIP agudo
#define LA_BAJA     57  // A3 (220 Hz) → BIIIIP grave


typedef enum{
 reposo,
 alarma,
 temporizador,

}estados;
estados estado = reposo;

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
uint8_t estadoboton = 0;
int i=0;
int t1=0;

struct tcp_pcb *g_pcb = NULL;
uint8_t g_bConnected = 0;  // Opcional ahora (no esperamos)

char g_pcPostData[512];
char g_pcMessage[128] = "Hola DESDE TIVA";

#define ESP_IP_A 192
#define ESP_IP_B 168
#define ESP_IP_C 100
#define ESP_IP_D 50
#define ESP_PORT 80

// === SIRENA DE BOMBEROS ESPAÑOLA ===
int siren_notes[] = {
    DO_CENTRAL,DO_CENTRAL,SOL,SOL,LA,LA,SOL,SOL,FA,FA,MI,MI,RE,RE,DO_CENTRAL
};

int pasos_restantes=0;
int nota_actual=0;
int indice_nota = 0;

int temporizador_notas[] = {
    LA_AGUDA,    // BIP (agudo rápido)
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
int temporizador_length = 5;  // Número de elementos

bool ENS160_Init(void)
{
    uint8_t part_id[2];

    // 1. Leer PART_ID
    // CORRECCIÓN 1: Usamos '!' porque readI2C devuelve true si va bien.
    // Si devuelve false (o 0), entonces entramos al error.
    if (!readI2C(ENS160_ADDR, 0x00, part_id, 2))
    {
        UARTprintf("Error de comunicacion I2C al leer ID\n");
        return false;
    }

    // DEBUG: Ver qué estamos leyendo realmente
    UARTprintf("\nDEBUG ENS160 ID: [0]=0x%x, [1]=0x%x \n", part_id[0], part_id[1]);

    // CORRECCIÓN 2: El ENS160 es Little Endian (LSB primero).
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
    readI2C(ENS160_ADDR, 0x20, &status, 1);
    readI2C(ENS160_ADDR, 0x22, data, 8);
    uint8_t validity = (status >> 2) & 0x03;
//    if (validity == 0 || validity == 1)
//    {
        ens_TVOC = data[0] | (data[1] << 8);   // <--- ¡SWAP AQUÍ! MSB primero
        ens_ECO2 =  data[2] | (data[3] << 8);   // <--- ¡SWAP AQUÍ! MSB primero
//    }
//    else
//    {
//        ens_TVOC = 0;
//        ens_ECO2 = 0;
//    }
    UARTprintf("Status Flag: %d (0=OK, 1=Warm, 2=Init)\n", validity);
}
void PlaySirenStep(void)
{
    VolNota(50);
    if (pasos_restantes == 0)
    {
        if (nota_actual == 0) {
            TocaNota(1, 69); pasos_restantes = 1;  // UIIII 0.5s
        }
        else if (nota_actual == 1) {
            TocaNota(1, 57); pasos_restantes = 4;  // UAAAA 2s
        }
        else {
            // === FIN DE CICLO: RESET PARA REPETIR ===
            TocaNota(0, 0);  // Silencio breve
            nota_actual = 0; // ¡RESET!
            pasos_restantes = 0;
            // estado = reposo;  // Comenta si quieres repetir
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
        // === ¿FIN DE LA MELODÍA? ===
        if (temporizador_notas[indice_nota] == 0)
        {
            TocaNota(SILENCIO, 0);  // Silencio final
            indice_nota = 0;
            pasos_restantes = 0;

        }

        // === TOCAR NUEVA NOTA ===
        TocaNota(XILOFONO, temporizador_notas[indice_nota]);

        // === DURACIÓN (en pasos de 500 ms) ===
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
    t1++;

}
void lwIPHostTimerHandler(void)
{
    // Versión minimalista: Solo chequea IP si cambió (raro con estática).
    // Si quieres silencio total, deja vacío {}.
    uint32_t ui32NewIPAddress = lwIPLocalIPAddrGet();
    if(ui32NewIPAddress != g_ui32IPAddress) {
        g_ui32IPAddress = ui32NewIPAddress;  // Actualiza global (sin prints)
        // Opcional: UARTprintf("IP changed to: "); DisplayIPAddress(ui32NewIPAddress); UARTprintf("\n");
    }
}
// Sensores (vacío)
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

        // Si el botón había pedido envío, hazlo ahora
        if (g_bPendingSend) {
            g_bPendingSend = false;  // limpiar bandera
            g_pcb = tpcb;            // asegurar PCB activo
            ESPSendPost();           // lanzar envío real
        }
    } else {
        UARTprintf("Connect failed: %d\n", err);
    }
    return ERR_OK;
}


// Ignorar recv completamente (no esperamos response)
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

// IGNORAR errores: No cierra PCB (fire-and-forget)
void ESPError(void *arg, err_t err)
{
    // Silencio: No log, no close. Deja que lwIP maneje.
}
void ESPSendPost(void)
{
    if (!g_pcb) {
        UARTprintf("No PCB, conectando y dejando envio pendiente...\n");
        g_bPendingSend = true;  // <-- marcar que hay envío pendiente
        ESPInit();              // inicia conexión
        return;
    }

    // Si ya hay conexión activa, enviar directamente
    int len = usprintf(g_pcPostData,
        "POST /send HTTP/1.1\r\n"
        "Host: %d.%d.%d.%d\r\n"
        "Content-Type: application/x-www-form-urlencoded\r\n"
        "Content-Length: %d\r\n"
        "\r\n"
        "text=%s",
        ESP_IP_A, ESP_IP_B, ESP_IP_C, ESP_IP_D,
        (int)strlen(g_pcMessage),
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

void HandleButton(void)
{


    switch(estadoboton) {
        case 0:
            if(B1_ON) {
                UARTprintf("Button pressed.\n");
                estadoboton = 1;
                SysCtlDelay(2400000);  // Debounce
            }
            break;

        case 1:
            if(B1_OFF) {
                UARTprintf("Button released - enviando!\n");
                if(g_ui32IPAddress != 0) {
                    ESPSendPost();  // Envía directo (sin check connected)
                } else {
                    UARTprintf("No IP.\n");
                }
                estadoboton = 0;
            }
            break;


    }
}

// Main simplificado
int main(void)
{
    uint32_t ui32User0, ui32User1;
        uint8_t pui8MACArray[6];

        // 1. Reloj del sistema
        SysCtlMOSCConfigSet(SYSCTL_MOSC_HIGHFREQ);
        g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                SYSCTL_USE_PLL | SYSCTL_CFG_VCO_240), 120000000);



        // 2. Habilitar periféricos de la placa base (LEDs, UART, Ethernet)
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

        // IMPORTANTE: Asegúrate de habilitar el puerto del I2C (Port B para BP1) aquí también por seguridad


        // 3. Configurar Pines de la placa base (Ethernet y UART)
        // MOVIDO: PinoutSet debe ir ANTES de configurar tus sensores
        //PinoutSet(true, false);
       // SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
        GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0 |GPIO_PIN_4);
        GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0 |GPIO_PIN_1);

        // Configuración UART
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

        // IP estática
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
          TimerLoadSet(TIMER0_BASE, TIMER_A, g_ui32SysClock / 2 - 1);
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


          SysCtlDelay(g_ui32SysClock / 10); // Pequeña pausa para estabilizar voltajes

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
            UARTprintf("Falló PART_ID\n");
            Ens_OK = false;
        }
    }




    while(1) {
        SLEEP;

        ReadSensors();  // Vacío
        HandleButton();  // Envío directo
        switch(estado){
        case reposo:
            break;
        case alarma:
            PlaySirenStep();
            break;
        case temporizador:
            TocaTemporizadorStep();
            break;

        }
        // LED blink
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1,
        (GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_1) ^ GPIO_PIN_1));
    }

    return 0;
}
