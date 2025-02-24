/*! ============================================================================
 *
 *  Copyright(c) 2025 JV Gomes. All rights reserved.
 *
 *  Fila Dry box - Firmware
 *  @brief  O programa a seguir faz o controle do hardware da drybox - estufa 
 *          de secagem de filamento de impressão 3D
 * 
 *  @note   O programa realiza medições continuas de temperatura e humidade
 *          por meio do modulo HDC 1080 comandado via barramento i2c. Com base
 *          na medição de temperatura e humidade o programa controla o acionmanto
 *          de um aquecedor e ventuinha para deslocamento de ar no interior da
 *          estufa
 *
 *  @file	    Drybox_Firmware.c
 *  @author     Joao Vitor G. de Oliveira
 *  @date	    18 Fev 2020
 *  @version    1.0
 * 
============================================================================ */

/* ============================   LIBRARIES  =============================== */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "ssd1306.h"

/* =============================   MACROS   ================================ */
// Defin. Interface I2C para comunicação do modulo HDC_1080:
#define HDC1080_I2C_BAUDRATE 100    //DEfine a freq do canal como 100 KHz
#define HDC1080_I2C_PORT i2c0       //Define a porta de comunicação I2C
#define HDC1080_I2C_SDA 8           //Define o pino 8 como SDA
#define HDC1080_I2C_SCL 9           //Define o pino 9 como SCL

// Defin. Interface I2C para comunicação do modulo SSD1306:
#define SSD1306_I2C_BAUDRATE 400    //DEfine a freq do canal como 400 KHz
#define SSD1306_I2C_PORT i2c1      //Define a porta de comunicação I2C
#define SSD1306_I2C_SDA 14          //Define o pino 14 como SDA
#define SSD1306_I2C_SCL 15          //Define o pino 15 como SCL

// Defin. do sesor de temperatura e umidade (HDC1080):
#define HDC1080_ADDR 0x40           //Define o endereco do modulo como 0x40
#define TEMP_REG 0x00               //Define o endereco do registro de temperatura
#define HUMI_REG 0x01               //Define o endereco do registro de umidade

// Defin. do display Oled (SSD1306):
#define OLED_ADDR 0x3C              //Define o endereco do display ole
#define OLED_WIDTH 128              //Define a largura do display
#define OLED_HEIGHT 64              //Define a altura do display

// Defin. dos pinos dos leds de visibilidade:
#define LED_RED_PIN 13 
#define LED_BLUE_PIN 12     
#define LED_GREEN_PIN 11    

// Defin. dos pinos de acionamento dos botoes
#define BTN_A_PIN 5
#define BTN_B_PIN 6 
#define DEBOUNCE_TIME_MS 50

// Defin. do pino de controle do aquecedor
#define AQUECEDOR_PIN 4

// Defin. dos valores de aquecimento para cada filamento
#define PLA_TEMP_MAX 50
#define PLA_TEMP_MIN 45
#define ABS_TEMP_MAX 70
#define ABS_TEMP_MIN 65

/* =========================   GLOBAL VARIABLES   ========================== */
// Variaveis de controle de tempo da interrupcao dos botoes
volatile uint32_t last_press_A_time = 0;
volatile uint32_t last_press_B_time = 0;

// variavel de controle do estado do aquecedor
volatile bool aquec_state = 1;

//Registro de mensagem do display
char mensagem [10] = {};

// Variavel de controle de uso da drybox
volatile bool Drybox_Run = 0;

// variavel de controle para processamento de telas
volatile uint8_t ctr = 0;

// Flags de leituras e controle dos registros
bool PLA_flag = false;
bool ABS_flag = false;

bool Forced_init = 0;

// Variaveis de controle da temperatura e umidade
float temperature = 0.00 , humidity = 0.00;

/* ========================   FUNCTION PROTOTYPE   ========================= */
// Funcoes I2c:
void I2c_init(void);    //Funcao de inicializacao do barramento I2c

// mapeamento de hardware
void Gpios_init(void);   //Inicializa todos os periféricos

// Callback da interrupcao dos botoes
void gpio_callback(uint gpio, uint32_t events);

// HDC1080:
uint16_t HDC1080_read_register(uint8_t reg);    //funcao de leitura dos registradores
float HDC1080_read_temperature(void);           //funcao de leitura da temperatura
float HDC1080_read_humidity(void);              //funcao de leitura da umidade

/* ===========================   MAIN FUNCTION   =========================== */
int main(void)
{
    stdio_init_all();

    I2c_init();
    Gpios_init();

    SSD1306_init();

    // Initialize render area for entire frame (SSD1306_WIDTH pixels by SSD1306_NUM_PAGES pages)
    struct render_area frame_area = {
        .start_col = 0,
        .end_col = SSD1306_WIDTH - 1,
        .start_page = 0,
        .end_page = SSD1306_NUM_PAGES - 1
    };

    calc_render_area_buflen(&frame_area);

    uint8_t buf[SSD1306_BUF_LEN];
    memset(buf, 0, SSD1306_BUF_LEN);
    render(buf, &frame_area);
    
    SSD1306_scroll(true);
    sleep_ms(1000);
    SSD1306_scroll(false);

    char *tela1[] = {
    "               ",
    "  EMBARCATECH  ",
    "     DRYBOX    ",
    "               "};
    //"123456789012345"};

    char *tela2[] = {
    "     CAIXA     ",
    "    SECADORA   ",
    "       DE      ",
    "   FILAMENTOS  "};

    char *tela3[] = {
    "      PLA      ",
    "               ",
    " TEMP:       C ",
    " UMID:       % "};

    char *tela4[] = {
    "      ABS      ",
    "               ",
    " TEMP:      *C ",
    " UMID:       % "};

    char *tela5[] = {
    "     DRYBOX    ",
    "   DESLIGADA   ",
    "  PRESSIONE A  ",
    "   PARA LIGAR  "};
    
    // Printa a primeira tela com led azul acesso por 3s
    for (int i = 0, y = 0; i < count_of(tela1); i++)
    {
        WriteString(buf, 5, y, tela1[i]);
        y += 8;
    }              
    render(buf, &frame_area);
    gpio_put(LED_BLUE_PIN,true);

    sleep_ms(3000);

    // Printa a segunda tela com led azul ainda acesso por 3s
    for (int i = 0, y = 0; i < count_of(tela2); i++)
    {
        WriteString(buf, 5, y, tela2[i]);
        y += 8;
    }              
    render(buf, &frame_area);

    sleep_ms(3000);
    
    // Printa a 5 tela desliga led azul e inicia programa
    for (int i = 0, y = 0; i < count_of(tela5); i++)
    {
        WriteString(buf, 5, y, tela5[i]);
        y += 8;
    }              
    render(buf, &frame_area);
    
    gpio_put(LED_BLUE_PIN,false);
    gpio_put(LED_GREEN_PIN,false);
    gpio_put(LED_RED_PIN,true);

    PLA_flag = 1;
    Drybox_Run = 0;

    while (true) // Laco de repeticao principal
    {
        if ( ctr == 1)
        {
             // Indicacao do estado de aciomaneto
            if (Drybox_Run == 0)
            {
                gpio_put(LED_RED_PIN,true);
                gpio_put(LED_GREEN_PIN,false);

                for (int i = 0, y = 0; i < count_of(tela5); i++)
                {
                    WriteString(buf, 5, y, tela5[i]);
                    y += 8;
                }
                gpio_put(AQUECEDOR_PIN,true); //Desliga o aquecedor o aquecedor
            }
            else if (Drybox_Run == 1)
            {
                gpio_put(LED_RED_PIN,false);
                gpio_put(LED_GREEN_PIN,true);

                if (PLA_flag == 1)
                {
                    for (int i = 0, y = 0; i < count_of(tela3); i++)
                    {
                        WriteString(buf, 5, y, tela3[i]);
                        y += 8;
                    }
                    // Controle do aquecimento da estufa
                    if ((temperature <= PLA_TEMP_MAX) && (temperature <= PLA_TEMP_MIN))
                    {
                        gpio_put(AQUECEDOR_PIN,false); //Liga o aquecedor
                    }
                    else 
                    {
                        gpio_put(AQUECEDOR_PIN,true); //Desliga o aquecedor o aquecedor
                    }
                }
                if (ABS_flag == 1)
                {
                    for (int i = 0, y = 0; i < count_of(tela4); i++)
                    {
                        WriteString(buf, 5, y, tela4[i]);
                        y += 8;
                    }

                    // Controle do aquecimento da estufa
                    if ((temperature <= ABS_TEMP_MAX) && (temperature <= ABS_TEMP_MIN))
                    {
                        gpio_put(AQUECEDOR_PIN,false); //Liga o aquecedor
                    }
                    else 
                    {
                        gpio_put(AQUECEDOR_PIN,true); //Desliga o aquecedor o aquecedor
                    }
                }
            }

            sleep_ms(20);
            ctr = 0;
        }

        if(Drybox_Run == 1)
        {
            temperature = HDC1080_read_temperature();
            humidity = HDC1080_read_humidity();

            sprintf(mensagem, "%.2f", temperature);
            WriteString(buf, 60, 20, mensagem); //Buffer , desl. horiz, desl. vert, string
            render(buf, &frame_area);

            sprintf(mensagem, "%.2f", humidity);
            WriteString(buf, 60, 24, mensagem); //Buffer , desl. horiz, desl. vert, string
            render(buf, &frame_area);
            
            // Controle de cionamento do aquecedor:
            if (PLA_flag == 1)
            {     
                if (temperature >= PLA_TEMP_MAX)
                {
                    gpio_put(AQUECEDOR_PIN,true); //Desliga o aquecedor
                }
                else if (temperature <= PLA_TEMP_MIN)
                {
                    gpio_put(AQUECEDOR_PIN,false); //Liga o aquecedor
                }
            }
            
            if (ABS_flag == 1)
            {
                if (temperature >= ABS_TEMP_MAX)
                {
                    gpio_put(AQUECEDOR_PIN,true); //Desliga o aquecedor
                }
                else if (temperature <= ABS_TEMP_MIN)
                {
                    gpio_put(AQUECEDOR_PIN,false); //Liga o aquecedor
                }
            }

            if(Forced_init == 1)
            {
                gpio_put(AQUECEDOR_PIN,false); //Liga o aquecedor
                Forced_init = 0;
            }

            sleep_ms(100);
        }
        else
        {
            gpio_put(AQUECEDOR_PIN,true); //Desliga o aquecedor
        }
        
    }

} /* end main */
/* =======================  DEVELOPMENT OF FUNCTIONS ======================= */
/*! ---------------------------------------------------------------------------
 *  @brief      Funcao de inicializacao das GPIOs
 *  @param      none
 *  @return     Void
 * 
 ----------------------------------------------------------------------------*/
void Gpios_init(void)
{
    gpio_init(LED_RED_PIN);
    gpio_set_dir(LED_RED_PIN, GPIO_OUT);
    
    gpio_init(LED_GREEN_PIN);
    gpio_set_dir(LED_GREEN_PIN, GPIO_OUT);
    
    gpio_init(LED_BLUE_PIN);
    gpio_set_dir(LED_BLUE_PIN, GPIO_OUT);

    // Desligando os LEDs no início
    gpio_put(LED_RED_PIN, false);
    gpio_put(LED_GREEN_PIN, false);
    gpio_put(LED_BLUE_PIN, false);
 
    gpio_init(BTN_A_PIN);
    gpio_set_dir(BTN_A_PIN, GPIO_IN);
    gpio_pull_up(BTN_A_PIN);
     
    gpio_init(BTN_B_PIN);
    gpio_set_dir(BTN_B_PIN, GPIO_IN);
    gpio_pull_up(BTN_B_PIN);

    gpio_init(AQUECEDOR_PIN);
    gpio_set_dir(AQUECEDOR_PIN, GPIO_OUT);
    gpio_put(AQUECEDOR_PIN, true);   //Garante que o aquecedor inicie desligado

    // Configurando interrupções para os dois botões corretamente
    gpio_set_irq_enabled_with_callback(BTN_A_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled_with_callback(BTN_B_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
}

/*! ---------------------------------------------------------------------------
 *  @brief      Funcao de chamada de interrupcao do acionamento dos botoes
 *  @param      gpio    Numero do pino GPIO
 *  @param      evento  Evento causador da interrupcao
 *  @return     Void
 * 
 ----------------------------------------------------------------------------*/
void gpio_callback(uint gpio, uint32_t events) 
{
    uint32_t now = time_us_32();

    if (gpio == BTN_A_PIN && (now - last_press_A_time > DEBOUNCE_TIME_MS * 1000))
    {   
        // Altera o estado de funcionamento do sistema
        Drybox_Run = !Drybox_Run;
        Forced_init = !Forced_init;

        ctr = 1;
        last_press_A_time = now;
    }
    else if (gpio == BTN_B_PIN && (now - last_press_B_time > DEBOUNCE_TIME_MS * 1000))
    {
        ABS_flag = !ABS_flag;
        PLA_flag = !PLA_flag;

        ctr = 1;
        last_press_B_time = now;
    }
}
/* ===========================  I2C FUNCTIONS  ============================= */

/*! ---------------------------------------------------------------------------
 *  @brief      Funcao de inicializacao do barramento i2c
 *  @param      none
 *  @return     Void
 * 
 ----------------------------------------------------------------------------*/
 void I2c_init(void)
 {
    // Inicializacao do I2C na frequencia pre configurada para o sensor de temperatura
    i2c_init(HDC1080_I2C_PORT, HDC1080_I2C_BAUDRATE*1000);
    
    // Definicao dos pinos 
    gpio_set_function(HDC1080_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(HDC1080_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(HDC1080_I2C_SDA);
    gpio_pull_up(HDC1080_I2C_SCL);

    // Inicializacao do I2C na frequencia pre configurada para o display OLED
    i2c_init(SSD1306_I2C_PORT, SSD1306_I2C_BAUDRATE*1000);
    
    // Definicao dos pinos 
    gpio_set_function(SSD1306_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(SSD1306_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(SSD1306_I2C_SDA);
    gpio_pull_up(SSD1306_I2C_SCL);

 }

 /* =========================  HDC1080 FUNCTIONS  =========================== */

/*! ---------------------------------------------------------------------------
 *  @brief      A funcao faz a leitura dos dados do registro de entrada
 *  @param[in] reg  endereco do registrador
 *  @return     valor lido no registrador 
 * 
 ----------------------------------------------------------------------------*/
 uint16_t HDC1080_read_register(uint8_t reg) 
 {
    uint8_t buf[2];

    // Envia o comando de leitura para o registrador
    i2c_write_blocking(HDC1080_I2C_PORT, HDC1080_ADDR, &reg, 1, true);
    sleep_ms(20); 

    // Leitura dos dois bytes de dados
    i2c_read_blocking(HDC1080_I2C_PORT, HDC1080_ADDR, buf, 2, false);

    //retorno dos valores concatenados
    return (buf[0] << 8) | buf[1];
}

/*! ---------------------------------------------------------------------------
 *  @brief      A funcao faz a leitura do valor de temperatura
 *  @param      none
 *  @return     Valor de temperatura
 * 
 ----------------------------------------------------------------------------*/
float HDC1080_read_temperature(void) 
{
    uint16_t raw_temp = HDC1080_read_register(TEMP_REG);
    //Conversao do valor de temperatura conforme datasheet
    return (raw_temp / 65536.0) * 165.0 - 40.0; 
}

/*! ---------------------------------------------------------------------------
 *  @brief      A funcao faz a leitura do valor de humidade
 *  @param      none
 *  @return     Valor de umidade
 * 
 ----------------------------------------------------------------------------*/
float HDC1080_read_humidity(void) 
{
    uint16_t raw_humi = HDC1080_read_register(HUMI_REG);
    //Conversao do valor de umidade conforme datasheet
    return (raw_humi / 65536.0) * 100.0; 
}   

/* end program */
 