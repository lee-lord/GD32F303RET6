systick_config();
    gd_eval_led_init(LED2);
    gd_eval_led_init(LED3);
    gd_eval_led_init(LED4);
    gd_eval_led_init(LED5);

    while(1){
        /* turn on led2, turn off led5 */
        gd_eval_led_on(LED2);
        gd_eval_led_off(LED5);
        delay_1ms(1000);
        /* turn on led3, turn off led2 */
        gd_eval_led_on(LED3);
        gd_eval_led_off(LED2);
        delay_1ms(1000);
        /* turn on led4, turn off led3 */
        gd_eval_led_on(LED4);
        gd_eval_led_off(LED3);
        delay_1ms(1000);
        /* turn on led5, turn off led4 */
        gd_eval_led_on(LED5);
        gd_eval_led_off(LED4);
        delay_1ms(1000);
    }