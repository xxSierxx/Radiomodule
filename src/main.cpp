#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/rcc.h>



void u_sart();

int main () {


    u_sart ();

    while (true) {
        // M0 - PB6, M1 - PB5
        gpio_clear(GPIOB,  GPIO5|GPIO6);
        gpio_toggle (GPIOD, GPIO14);
        volatile uint8_t symb = usart_recv_blocking(USART2);
        //usart_send_blocking(USART2,0x36);

        for(volatile int i = 0; i<1'000'0; i++){}

        //
        //symb = usart_recv_blocking(USART2);


    }
}
void u_sart(){

    //Светодиод
    rcc_periph_clock_enable(RCC_GPIOD);
    gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO14);

    //пины для M0 и M1
    rcc_periph_clock_enable(RCC_GPIOB);
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,GPIO5|GPIO6);


    //УАПП
    rcc_periph_clock_enable(RCC_USART2);

    usart_set_baudrate(USART2, 9600); //скорость передачи
    usart_set_databits (USART2,8); //  длина в битах
    usart_set_parity(USART2,USART_PARITY_NONE);
    usart_set_stopbits(USART2,USART_STOPBITS_1); //Стоп-БИТ

    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
    usart_set_mode(USART2,USART_MODE_TX_RX);

    //Контакты для УАПП
    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_mode_setup(GPIOA,GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2|GPIO3);
    gpio_set_af(GPIOA, GPIO_AF7, GPIO2|GPIO3); // PA2 - TX, PA3 - RX

    usart_enable(USART2);


}






