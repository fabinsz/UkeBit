#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "inc/ssd1306.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "kiss_fftr.h"

#define BUTTON_A 5  // GPIO conectado ao Botão A
#define BUTTON_B 6  // GPIO conectado ao Botão B 

#define RED_LED 13  // GPIO conectado ao led vermelho
#define BLUE_LED 12   // GPIO conectado ao led azul
#define GREEN_LED 11    // GPIO conectado ao led verde

// Pino e canal do microfone no ADC.
#define MIC_CHANNEL 2
#define MIC_PIN (26 + MIC_CHANNEL)

// Parâmetros e macros do ADC.
#define ADC_CLOCK_DIV 192.f
#define SAMPLES 4096 // O tamanho da FFT deve ser potência de 2
#define ADC_ADJUST(x) (x * 3.3f / (1 << 12u) - 1.65f) // Ajuste do valor do ADC para Volts.
#define SAMPLE_RATE 10000  // Taxa de amostragem (10 kHz)

//pino oled
const uint I2C_SDA = 14;
const uint I2C_SCL = 15;

// Buffer de amostras do ADC.
uint16_t adc_buffer[SAMPLES];

// FFT input/output buffers
kiss_fft_scalar in[SAMPLES];
kiss_fft_cpx out[SAMPLES / 2];

// Canal e configurações do DMA
uint dma_channel;
dma_channel_config dma_cfg;

// inicializacao das funcoes
float calculate_frequancy();
void sample_mic();
void centraliza();
void led_blue_on();
void led_red_on();
void led_green_on();
void led_off();
void ADC();
void botoes();
void I2C();
void leds();
void dma();

void draw_float_on_oled(uint8_t *ssd, int x, int y, float value, int precision) {
    char buffer[20];  
    sprintf(buffer, "%.*f", precision, value); // Usa sprintf() para converter float para string
    ssd1306_draw_string(ssd, x, y, buffer);
}

/**
 * Realiza as leituras do ADC e armazena os valores no buffer.
 */
void sample_mic() {
    adc_fifo_drain(); // Limpa o FIFO do ADC.
    adc_run(false); // Desliga o ADC (se estiver ligado) para configurar o DMA.
  
    dma_channel_configure(dma_channel, &dma_cfg,
      adc_buffer, // Escreve no buffer.
      &(adc_hw->fifo), // Lê do ADC.
      SAMPLES, // Faz "SAMPLES" amostras.
      true // Liga o DMA.
    );
  
    // Liga o ADC e espera acabar a leitura.
    adc_run(true);
    dma_channel_wait_for_finish_blocking(dma_channel);
    
    // Acabou a leitura, desliga o ADC de novo.
    adc_run(false);
  }
  
/**
  * Calcula a frequncia dominante das leituras do ADC.
  */
 float calculate_frequency() {
    // Converter ADC para valores float normalizados (-1.65V a 1.65V)
    for (int i = 0; i < SAMPLES; i++) {
        in[i] = adc_buffer[i] * 3.3f / 4096.0f - 1.65f;
    }

    // Criar configuração da FFT
    kiss_fftr_cfg cfg = kiss_fftr_alloc(SAMPLES, 0, NULL, NULL);

    // Executar FFT
    kiss_fftr(cfg, in, out);
    
    // Encontrar pico na FFT (frequência dominante)
    int peak_index = 0;
    float max_magnitude = 0.0;

    for (int i = 1; i < SAMPLES / 2 - 1; i++) {  // Evita limites
        float magnitude = sqrt(out[i].r * out[i].r + out[i].i * out[i].i);
        if (magnitude > max_magnitude) {
            max_magnitude = magnitude;
            peak_index = i;
        }
    }

    // Interpolação parabólica para melhorar precisão da frequência detectada
    float alpha = sqrt(out[peak_index - 1].r * out[peak_index - 1].r + out[peak_index - 1].i * out[peak_index - 1].i);
    float beta = max_magnitude;
    float gamma = sqrt(out[peak_index + 1].r * out[peak_index + 1].r + out[peak_index + 1].i * out[peak_index + 1].i);

    float delta = (alpha - gamma) / (2 * (2 * beta - alpha - gamma));
    float refined_index = peak_index + delta;

    // Calcular a frequência correspondente ao pico interpolado
    float dominant_frequency = (refined_index * SAMPLE_RATE) / SAMPLES;

    // Liberar memória da FFT
    free(cfg);

    return dominant_frequency;
}

// exibicao do menu
void menu(uint8_t *ssd, int num) {
    // Definindo um array de strings
    char corda_1[20], corda_2[20], corda_3[20], corda_4[20];
    char* cordas[] = {corda_1, corda_2, corda_3, corda_4};  // Array de ponteiros para as variáveis

    // Usando sprintf para formatar as strings corretamente
    for (int i = 0; i < 4; i++) {
        sprintf(cordas[i], "Corda %d", i + 1);  // Usando sprintf para formatar a string
    }
    
    memset(ssd, 0, ssd1306_buffer_length);  // Limpa o buffer antes de desenhar o menu    
    
    // Centralizando o título "UkeBit" no meio da tela (128x64)
    centraliza(ssd, "UkueBit");

    // Atualiza a string da corda selecionada
    switch (num)
    {
    case 0:
        strcpy(cordas[0], "> Corda 1");  // Atualiza a string
        break;
    case 1:
        strcpy(cordas[1], "> Corda 2");
        break;
    case 2:
        strcpy(cordas[2], "> Corda 3");
        break;
    case 3:
        strcpy(cordas[3], "> Corda 4");
        break;
    } 
    
    // Desenhando as cordas
    for (int i = 0; i < 4; i++) {
        ssd1306_draw_string(ssd, 10, 20 + (i * 10), cordas[i]);  // Posicionando as cordas
    }
}
// centraliza um titulo no display
void centraliza(uint8_t *ssd, char title[]){
    int title_len = strlen(title);
    int title_x = (128 - title_len * 6) / 2;  // 6 é a largura de cada caractere em pixels (supondo uma fonte de 6x8)
    ssd1306_draw_string(ssd, title_x, 0, title); 
}

// desenha frequenciua registrada no display
void draw_frequency(uint8_t *ssd, int num, float frequency) {
    char ideal[20], cord[20];
    
    memset(ssd, 0, ssd1306_buffer_length);  // Limpa o buffer antes de desenhar o menu    
    
    if (frequency <= 20 && frequency >= 4){
        draw_float_on_oled(ssd, 40, 25, frequency, 2); // Exibe frequencia com 2 casas decimais
    }

    switch (num)
    {
    case 0:
        sprintf(ideal, "ideal 17");  // Atualiza a string
        strcpy(cord, "Corda 1");
        break;
    case 1:
        sprintf(ideal, "ideal 11.90");
        strcpy(cord, "Corda 2");
        break;
    case 2:
        sprintf(ideal, "ideal 9.50");
        strcpy(cord, "Corda 3");
        break;
    case 3:
        sprintf(ideal, "ideal 14");
        strcpy(cord, "Corda 4");
        break;
    } 
    
    centraliza(ssd, cord);
    ssd1306_draw_string(ssd, 10, 50, ideal);
}

// visualiza a cor do led
void visualizer(int num, float frequency){

    switch (num)
    {
    case 0:
        if(frequency <= 16.75 && frequency >= 4){
            led_blue_on();

        }
        else if(frequency >= 17.25 && frequency <= 20){
            led_red_on();

        }
        else if(frequency < 17.25 && frequency > 16.75){
            led_green_on();

        }
        else{
            led_off();
        }

        break;
    case 1:
        if(frequency <= 11.65 && frequency >= 4){
            led_blue_on();

        }
        else if(frequency >= 12.15 && frequency <= 20){
            led_red_on();

        }
        else if(frequency < 12.15 && frequency > 11.65){
            led_green_on();

        }
        else{
            led_off();
        }


        break;
    case 2:
        if(frequency <= 9.25 && frequency >= 4){
            led_blue_on();

        }
        else if(frequency >= 9.80 && frequency <= 20){
            led_red_on();

        }
        else if(frequency < 9.75 && frequency > 9.25){
            led_green_on();

        }
        else{
            led_off();;
        }

        break;
    case 3:
        if(frequency <= 13.75 && frequency >= 4){
            led_blue_on();

        }
        else if(frequency >= 14.25 && frequency <= 20){
            led_red_on();

        }
        else if(frequency < 14.25 && frequency > 13.75){
            led_green_on();

        }
        else{
            led_off();
        }

        break;
    } 

}

// funcoes para os leds
void led_green_on(){
    gpio_put(BLUE_LED, 0);
    gpio_put(GREEN_LED, 1);
    gpio_put(RED_LED, 0);
}

void led_red_on(){
    gpio_put(BLUE_LED, 0);
    gpio_put(GREEN_LED, 0);
    gpio_put(RED_LED, 1);
}

void led_blue_on(){
    gpio_put(BLUE_LED, 1);
    gpio_put(GREEN_LED, 0);
    gpio_put(RED_LED, 0);
}

void led_off(){
    gpio_put(BLUE_LED, 0);
    gpio_put(GREEN_LED, 0);
    gpio_put(RED_LED, 0);
}
// inicializacao ADC
void ADC(){
    adc_gpio_init(MIC_PIN);
    adc_init();
    adc_select_input(MIC_CHANNEL);
}
// inicializacao Botoes
void botoes(){

    gpio_init(BUTTON_A);
    gpio_set_dir(BUTTON_A, GPIO_IN);
    gpio_pull_up(BUTTON_A);

    
    gpio_init(BUTTON_B);
    gpio_set_dir(BUTTON_B, GPIO_IN);
    gpio_pull_up(BUTTON_B);
}
// Inicializacao do I2C
void I2C(){
    
    i2c_init(i2c1, ssd1306_i2c_clock * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
}
// Inicializacao dos leds
void leds(){
    
    gpio_init(RED_LED);
    gpio_set_dir(RED_LED, GPIO_OUT);

    gpio_init(BLUE_LED);
    gpio_set_dir(BLUE_LED, GPIO_OUT);

    gpio_init(GREEN_LED);
    gpio_set_dir(GREEN_LED, GPIO_OUT);
}
// Tomando posse de canal do DMA.
void dma(){

    dma_channel = dma_claim_unused_channel(true);
    dma_cfg = dma_channel_get_default_config(dma_channel);
    channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&dma_cfg, false);
    channel_config_set_write_increment(&dma_cfg, true);
    channel_config_set_dreq(&dma_cfg, DREQ_ADC);
}

int main() {
    stdio_init_all();   // Inicializa os tipos stdio padrão presentes ligados ao binário
    sleep_ms(1000);
    bool main_menu = true;
    int num_b = 0;

    // Preparação das funcoes.
    ADC();

    botoes();   

    I2C();

    leds();

    // Processo de inicialização completo do OLED SSD1306
    ssd1306_init();

    // Preparar área de renderização para o display
    struct render_area frame_area = {
        start_column : 0,
        end_column : ssd1306_width - 1,
        start_page : 0,
        end_page : ssd1306_n_pages - 1
    };

    adc_fifo_setup(
        true, // Habilitar FIFO
        true, // Habilitar request de dados do DMA
        1, // Threshold para ativar request DMA é 1 leitura do ADC
        false, // Não usar bit de erro
        false // Não fazer downscale das amostras para 8-bits, manter 12-bits.
    );

    adc_set_clkdiv(ADC_CLOCK_DIV);

    dma();

    calculate_render_area_buffer_length(&frame_area);

    // Zera o display inteiro
    uint8_t ssd[ssd1306_buffer_length];

    
    render_on_display(ssd, &frame_area);  // Atualiza o display

    while (true) {
        sample_mic();
        float frequency = calculate_frequency();
        
        memset(ssd, 0, ssd1306_buffer_length);


        while(main_menu){
            led_off();

            if (gpio_get(BUTTON_B) == 0) {  // Verifica se o botão foi pressionado
                sleep_ms(50);  // Debounce: espera 50ms para evitar leituras falsas
                if (gpio_get(BUTTON_B) == 0) { // Confirma que ainda está pressionado
                    num_b++;
                    while (gpio_get(BUTTON_B) == 0); // Aguarda o botão ser solto
                }
            }
            else if(num_b == 4){
                num_b = 0;
            }
            else if (gpio_get(BUTTON_A) == 0){
                sleep_ms(50);  
                if (gpio_get(BUTTON_A) == 0) { 
                    main_menu = false; // desativa o menu temporariamente
                    while (gpio_get(BUTTON_A) == 0); 
                }
            }
            switch (num_b) {
                case 0: menu(ssd, num_b); break;
                case 1: menu(ssd, num_b); break;
                case 2: menu(ssd, num_b); break;
                case 3: menu(ssd, num_b); break;
            }
            render_on_display(ssd, &frame_area);

        }

        if (gpio_get(BUTTON_B) == 0 || gpio_get(BUTTON_A) == 0) {  // Verifica se o botão foi pressionado
            sleep_ms(50);  
            if (gpio_get(BUTTON_B) == 0 || gpio_get(BUTTON_A) == 0) { // Confirma que ainda está pressionado
                main_menu = true; // ativa o menu novamente
                while (gpio_get(BUTTON_B) == 0 || gpio_get(BUTTON_A) == 0); // Aguarda o botão ser solto
            }
        }
        switch (num_b) { 
            case 0: 
                draw_frequency(ssd, num_b, frequency); 
                visualizer(num_b, frequency);
                break;
            case 1: 
                draw_frequency(ssd, num_b, frequency); 
                visualizer(num_b, frequency);
                break;
            case 2: 
                draw_frequency(ssd, num_b, frequency); 
                visualizer(num_b, frequency);
                break;
            case 3: 
                draw_frequency(ssd, num_b, frequency); 
                visualizer(num_b, frequency);
                break;
        }

        render_on_display(ssd, &frame_area);
        sleep_ms(100);

    }

    return 0;
}
