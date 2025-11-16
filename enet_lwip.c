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
#include "FT800_TIVA.h"

#define B1_OFF GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_0)
#define B1_ON !(GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_0))
// Defines SLEEP
#define SLEEP SysCtlSleepFake()
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
#define DO_AGUDO    72  // C5 (523 Hz) → BIP medio

typedef enum{
 reposo,
 alarma,
 temporizador,

}estados;
estados estado = temporizador;

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
    // Vacío: Mensaje fijo. Añade ADC si quieres.
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

    SysCtlMOSCConfigSet(SYSCTL_MOSC_HIGHFREQ);
    g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                                         SYSCTL_USE_PLL | SYSCTL_CFG_VCO_240), 120000000);
    HAL_Init_SPI(1,g_ui32SysClock);  //Boosterpack a usar, Velocidad del MC
    Inicia_pantalla();       //Arranque de la pantalla
    SysCtlDelay(g_ui32SysClock/3);

    //Habilitar los periféricos implicados: GPIOF, J, N
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    PinoutSet(true, false);
    UARTStdioConfig(0, 115200, g_ui32SysClock);
    UARTprintf("\033[2J\033[H");
    UARTprintf("Tiva to ESP32: \n\n");

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0 |GPIO_PIN_4); //F0 y F4: salidas
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0 |GPIO_PIN_1); //N0 y N1: salidas

    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0|GPIO_PIN_1);   //J0 y J1: entradas
    GPIOPadConfigSet(GPIO_PORTJ_BASE,GPIO_PIN_0|GPIO_PIN_1,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU); //Pullup en J0 y J1

    // Timer0 SLEEP (500ms)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, g_ui32SysClock / 2 - 1);
    TimerIntRegister(TIMER0_BASE, TIMER_A, Timer0IntHandler);
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER0_BASE, TIMER_A);

    // SysTick lwIP
    SysTickPeriodSet(g_ui32SysClock / SYSTICKHZ);
    SysTickEnable();
    SysTickIntEnable();

    // MAC
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

    UARTprintf("Waiting IP. Press button to send.\n");

    // IP estática
    uint32_t ui32IP = (192u<<24)|(168u<<16)|(100u<<8)|110u;
    uint32_t ui32NetMask = (255u<<24)|(255u<<16)|(255u<<8)|0u;
    uint32_t ui32Gateway = (192u<<24)|(168u<<16)|(100u<<8)|1u;
    lwIPInit(g_ui32SysClock, pui8MACArray, ui32IP, ui32NetMask, ui32Gateway, IPADDR_USE_STATIC);

    g_ui32IPAddress = lwIPLocalIPAddrGet();
    UARTprintf("IP: ");
    DisplayIPAddress(g_ui32IPAddress);
    UARTprintf("\nESP32 ready (192.168.100.50).\n");

    LocatorInit();
    LocatorMACAddrSet(pui8MACArray);
    LocatorAppTitleSet("Tiva ESP32 Send");

    IntPrioritySet(INT_EMAC0, ETHERNET_INT_PRIORITY);
    IntPrioritySet(FAULT_SYSTICK, SYSTICK_INT_PRIORITY);
    IntEnable(INT_EMAC0);
    IntMasterEnable();

    Nueva_pantalla(169,125,70);
    Dibuja();
    for ( i = 0; i < 6; i++) Esc_Reg(REG_TOUCH_TRANSFORM_A + 4 * i, REG_CAL[i]);

    //prueba n otas
//    VolNota(100);           // Volumen: 0-127 (100 = fuerte)
//    TocaNota(0, 60);        // Instrumento 0 (Piano), nota 60 (C4 = Do central)
//    SysCtlDelay(g_ui32SysClock / 3); // Espera 1 segundo
//
//    TocaNota(25, 64);       // Guitarra eléctrica, Mi4
//    SysCtlDelay(g_ui32SysClock/ 3);
//
//    TocaNota(73, 67);       // Flauta, Sol4
//    SysCtlDelay(g_ui32SysClock / 3);
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
