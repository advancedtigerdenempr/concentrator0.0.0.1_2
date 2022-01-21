#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include <stdio.h>
#include "esp_spi_flash.h"
#include <driver/adc.h>
#include "esp_adc_cal.h"
#include "freertos/queue.h"
#include "driver/touch_pad.h"
#include "driver/uart.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/rtc_io.h"
#include "esp_freertos_hooks.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif

#include "lvgl_helpers.h"
//#include "display_port.h"
//#include "lvgl_tft/esp_lcd_backlight.h"

#define LV_TICK_PERIOD_MS 1


//Константы настройки АЦП
#define NO_OF_SAMPLES 64 //Множественная выборка
#define NUMBER_OF_VALUES 40 //Количество выборок АЦП
//Константы настройки сенсорных кнопок
#define TOUCH_BUTTON_NUM 4
#define TOUCH_CHANGE_CONFIG 0
#define TOUCH_QUEUE_LENGTH 10
#define SIZE_DATA_OF_THE_TOUCH_QUEUE 16
//Псевдонимы GPIO
#define LED1 GPIO_NUM_38
#define LED2 GPIO_NUM_37
#define BEEP GPIO_NUM_0
#define PWR_PUMP GPIO_NUM_33
#define CLAP1 GPIO_NUM_45
#define CLAP2 GPIO_NUM_11
#define PWR GPIO_NUM_3
#define PWR_DISP GPIO_NUM_35
#define PWR_LED_DISP GPIO_NUM_36
#define SERVICE_BUTTON GPIO_NUM_34
//Логические уровни
#define HL 1
#define LL 0
//Константы калибровки инициализации FreeRTOS
#define TOUCH_QUEUE_LENGTH 4
#define SIZE_DATA_OF_THE_TOUCH_QUEUE 4
#define CS_QUEUE_LENGTH 10
#define SIZE_DATA_OF_THE_CS_QUEUE 6
#define PRESSURE_QUEUE_LENGTH 10 //Количество элементов очереди
#define SIZE_DATA_OF_THE_PRESSURE_QUEUE 4 //Размер элемента очереди в байтах
#define SIGNALS_LVGL_QUEUE_LENGTH 1
#define SIZE_DATA_OF_THE_SIGNALS_LVGL_QUEUE 1
#define DATA_SERVICE_LVGL_QUEUE_LENGTH 5
#define SIZE_DATA_SERVICE_OF_THE_DATA_LVGL_QUEUE 1
//Истина и ложь
#define TRUE 1
#define FALSE 0
//Псевдонимы сенсорных кнопок
#define RESUME_AFTER_POWER_IS_RESTORED 0
#define RESETTING_THE_MACHINE 1
#define PARAMETER_SELECTION 2
#define INCREASE 0
#define DECREASE 1
//Константы калибровки
#define CLICK_THRESHOLD 8000
#define DEFAULT_VALUE_T1 1
#define DEFAULT_VALUE_T2 1
#define DEFAULT_VALUE_T3 1
#define DEFAULT_VALUE_T4 1
#define DEFAULT_VALUE_OFFSET 1
#define	RATTLE_RELAXATION_TIME_MS 50  //Время релаксации дребезга контактов
#define	LIMITING_RATE_OF_PARAMETER_CHANGE_MS 500  //Время между инкрементированием значения параметра, нужно из-за высокой скорости работы МК чтобы инженер успел убрать палец с сенсорной кнопки перед следующей операцией инкрементирования

static const touch_pad_t button[TOUCH_BUTTON_NUM] = {
    TOUCH_PAD_NUM5,
    TOUCH_PAD_NUM6,
    TOUCH_PAD_NUM7,
    TOUCH_PAD_NUM8,
};
//Прототипы функций
void check_res_creating_task(BaseType_t);
void check_res_creating_queue(QueueHandle_t);
void check_res_creating_semph(SemaphoreHandle_t);
uint8_t getCheckSum(uint8_t*);
static void ADC_Task(void *pvParameter);
static void Buttons_Task(void *pvParameter);
static void Clapan1_Task(void *pvParameter);
static void Clapan2_Task(void *pvParameter);
static void CS_Task(void *pvParameter);
static void Service_Task(void *pvParameter);
static void Time_Task(void *pvParameter);
static void Checking_Power_Loss_Task(void *pvParameter);
static void lv_tick_task(void *arg);
static void guiTask(void *pvParameter);
static void T1_handler(TimerHandle_t);
void T2_handler(TimerHandle_t);
void T3_handler(TimerHandle_t);
void T4_handler(TimerHandle_t);
void Timer_of_offset_handler(TimerHandle_t);
static void create_application_gui(void);
void READ_TOUCH(uint32_t * tmp);
lv_obj_t * text_start1;
lv_obj_t * text_start2;

//Глобальные переменные
_Bool t1_end;
_Bool t2_end;
uint64_t time_s;
uint32_t time_min_filter_coarse_clear;
uint16_t time_h_filter_coarse_clear;
uint32_t time_min_filter_fine_clear;
uint16_t time_h_filter_fine_clear;
uint64_t T1, T2, T3, T4, OFFSET;
uint32_t val;
_Bool coarse_filter_cleaning_is_required;
_Bool fine_filter_cleaning_is_required;
uint8_t pt_m;
uint8_t pt_h;

//Глобальные переменные FreeRTOS и ESP-IDF
BaseType_t res_Creating;        //Возвращенное значение результата вызова функции создания задачи
QueueHandle_t xQueue_press;
QueueHandle_t xQueue_touch;
QueueHandle_t xQueue_CS;
QueueHandle_t xQueue_signals_LVGL;
SemaphoreHandle_t clap2_semph;
SemaphoreHandle_t xGuiSemaphore;
nvs_handle_t flash_handle;
esp_err_t err;
TaskHandle_t adc_desc;
TaskHandle_t button_desc;
TaskHandle_t clapan1_desc;
TaskHandle_t clapan2_desc;
TaskHandle_t cs_desc;
TaskHandle_t service_desc;
TaskHandle_t time_desc;
TaskHandle_t power_loss_desc;
TaskHandle_t gui_desc;
TaskHandle_t lv_tick_desc;

TimerHandle_t Timer3;
TimerHandle_t Timer4;
TimerHandle_t Timer1;
TimerHandle_t Timer2;
TimerHandle_t Timer_of_offset;

//Глобальные переменные LVGL
lv_style_t style_main_text;
lv_style_t style_index_text;
lv_style_t style_input_field_text;
lv_style_t style_selectable_text;
lv_style_t style_button_text;
lv_style_t style_sub_index_text;
lv_obj_t * screen_start;
lv_obj_t * start_lab1;
lv_obj_t * start_lab2;
lv_obj_t * start_lab3;
lv_obj_t * start_lab4;
lv_obj_t * start_lab5;
lv_obj_t * start_lab6;
lv_obj_t * start_lab7;
lv_obj_t * start_lab8;
lv_obj_t * start_lab9;
lv_obj_t * start_lab10;
lv_obj_t * start_lab11;
lv_obj_t * start_lab12;
lv_obj_t * start_lab13;
lv_obj_t * start_lab14;
lv_obj_t * start_lab15;
lv_obj_t * screen_task;
lv_obj_t * task_lab1;
lv_obj_t * Tic_bar;
lv_obj_t * task_lab2;
lv_obj_t * task_lab3;
lv_obj_t * task_lab4;
lv_obj_t * task_lab5;
lv_obj_t * task_lab6;
lv_obj_t * task_lab7;
lv_obj_t * c_bar;
lv_obj_t * task_lab8;
lv_obj_t * task_lab9;
lv_obj_t * task_lab10;
lv_obj_t * F_bar;
lv_obj_t * task_lab11;
lv_obj_t * task_lab12;
lv_obj_t * task_lab13;
lv_obj_t * P_bar;
lv_obj_t * task_lab14;
lv_obj_t * task_lab15;
lv_obj_t * task_lab16;
lv_obj_t * task_lab35;
lv_obj_t * task_lab17;
lv_obj_t * TO2_bar;
lv_obj_t * task_lab18;
lv_obj_t * task_lab19;
lv_obj_t * task_lab20;
lv_obj_t * task_lab21;
lv_obj_t * task_lab22;
lv_obj_t * task_lab23;
lv_obj_t * task_lab24;
lv_obj_t * task_lab36;
lv_obj_t * task_lab25;
lv_obj_t * task_lab26;
lv_obj_t * task_lab27;
lv_obj_t * task_lab28;
lv_obj_t * task_lab29;
lv_obj_t * task_lab30;
lv_obj_t * task_lab31;
lv_obj_t * task_lab32;
lv_obj_t * task_lab33;
lv_obj_t * task_lab34;
lv_obj_t * screen_filters_inform;
lv_obj_t * filters_inform_lab1;
lv_obj_t * filters_inform_lab2;
lv_obj_t * filters_inform_lab3;
lv_obj_t * filters_inform_lab4;
lv_obj_t * filters_inform_lab5;
lv_obj_t * filters_inform_lab6;
lv_obj_t * filters_inform_lab7;
lv_obj_t * filters_inform_lab8;
lv_obj_t * filters_inform_lab9;
lv_obj_t * filters_inform_lab10;
lv_obj_t * filters_inform_lab11;
lv_obj_t * filters_inform_lab12;
lv_obj_t * filters_inform_lab13;
lv_obj_t * filters_inform_lab14;
lv_obj_t * filters_inform_lab15;
lv_obj_t * filters_inform_lab16;
lv_obj_t * filters_inform_lab17;
lv_obj_t * filters_inform_lab18;
lv_obj_t * filters_inform_lab19;
lv_obj_t * screen_filters_new;
lv_obj_t * filters_new_lab1;
lv_obj_t * filters_new_lab2;
lv_obj_t * filters_new_lab3;
lv_obj_t * filters_new_lab4;
lv_obj_t * screen_filters_coarse;
lv_obj_t * filters_coarse_lab1;
lv_obj_t * filters_coarse_lab2;
lv_obj_t * filters_coarse_lab3;
lv_obj_t * filters_coarse_lab4;
lv_obj_t * screen_question_cleared_coarse;
lv_obj_t * filters_question_cleared_coarse_lab1;
lv_obj_t * filters_question_cleared_coarse_lab2;
lv_obj_t * filters_question_cleared_coarse_lab3;
lv_obj_t * filters_question_cleared_coarse_lab4;
lv_obj_t * screen_question_new_coarse;
lv_obj_t * filters_question_new_coarse_lab1;
lv_obj_t * filters_question_new_coarse_lab2;
lv_obj_t * filters_question_new_coarse_lab3;
lv_obj_t * filters_question_new_coarse_lab4;
lv_obj_t * screen_filters_fine;
lv_obj_t * filters_fine_lab1;
lv_obj_t * filters_fine_lab2;
lv_obj_t * filters_fine_lab3;
lv_obj_t * filters_fine_lab4;
lv_obj_t * screen_question_cleared_fine;
lv_obj_t * filters_question_cleared_fine_lab1;
lv_obj_t * filters_question_cleared_fine_lab2;
lv_obj_t * filters_question_cleared_fine_lab3;
lv_obj_t * filters_question_cleared_fine_lab4;
lv_obj_t * screen_question_new_fine;
lv_obj_t * filters_question_new_fine_lab1;
lv_obj_t * filters_question_new_fine_lab2;
lv_obj_t * filters_question_new_fine_lab3;
lv_obj_t * filters_question_new_fine_lab4;


//Сигналы для графического интерфейса
#define SERVICE_DISP 1         //Сервисный режим
#define MOVE_TO_T1_DISP 2      //Сдвинуться на T1 Выбирается опредленнный параметр, изменение которого затем производится
#define MOVE_TO_T2_DISP 3      //Сдвинуться на T2
#define MOVE_TO_T3_DISP 4      //Сдвинуться на T3
#define MOVE_TO_T4_DISP 5      //Сдвинуться на T4
#define MOVE_TO_OFFSET_DISP 6  //Сдвинуться на OFFSET
#define INCREASE_PARAMETER_DISP 7
#define DECREASE_PARAMETER_DISP 8

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    return ESP_OK;
}

void app_main(void)
{
    gpio_pad_select_gpio(LED1);
    gpio_pad_select_gpio(LED2);
    gpio_pad_select_gpio(BEEP);
    gpio_pad_select_gpio(PWR_PUMP);
    gpio_pad_select_gpio(CLAP1);
    gpio_pad_select_gpio(CLAP2);
    gpio_config_t conf_gpio;
    conf_gpio.pin_bit_mask=LED1|LED2|BEEP|PWR_PUMP|CLAP1|CLAP2;
    conf_gpio.mode=GPIO_MODE_OUTPUT;
    conf_gpio.pull_up_en=GPIO_PULLUP_DISABLE;
    conf_gpio.pull_down_en=GPIO_PULLDOWN_DISABLE;
    gpio_config(&conf_gpio);
    //Initialization
    printf("Start of the program!\n");

    printf("Creating tasks!\n");
    printf("Creating an ADC task\n");

    res_Creating=xTaskCreatePinnedToCore(ADC_Task, "ADC", 100, NULL, 4, &adc_desc, 1); //Создаем задачу АЦП
    check_res_creating_task(res_Creating); //Проверка статуса вызова фунции создания задачи
    printf("Creating an Buttons task\n");
    res_Creating=xTaskCreatePinnedToCore(Buttons_Task, "Buttons", 100, NULL, 4, &button_desc, 1); //Создаем задачу сенсорных кнопок
    check_res_creating_task(res_Creating); //Проверка статуса вызова фунции создания задачи
    printf("Creating an Clapan1 task\n");
    res_Creating=xTaskCreatePinnedToCore(Clapan1_Task, "Clapan1", 100, NULL, 4, &clapan1_desc, 1); //Создаем задачу входа управления клапана 1
    check_res_creating_task(res_Creating); //Проверка статуса вызова фунции создания задачи
    printf("Creating an Clapan2 task\n");
    res_Creating=xTaskCreatePinnedToCore(Clapan2_Task, "Clapan2", 100, NULL, 4, &clapan2_desc, 1); //Создаем задачу входа управления клапана 2
    check_res_creating_task(res_Creating); //Проверка статуса вызова фунции создания задачи
    printf("Creating an Сoncentration sensor task\n");
    res_Creating=xTaskCreatePinnedToCore(CS_Task, "CS", 100, NULL, 4, &cs_desc, 1); //Создаем задачу опроса датчика концентрации
    check_res_creating_task(res_Creating); //Проверка статуса вызова фунции создания задачи
    printf("Creating an Service task\n");
    res_Creating=xTaskCreatePinnedToCore(Service_Task, "Service", 100, NULL, 4, &service_desc, 1); //Создаем задачу настройки аппарата
    check_res_creating_task(res_Creating); //Проверка статуса вызова фунции создания задачи
    printf("Creating an Time task\n");
    res_Creating=xTaskCreatePinnedToCore(Time_Task, "Time", 100, NULL, 4, &time_desc, 1); //Создаем задачу подсчета времени работы аппарата
    check_res_creating_task(res_Creating); //Проверка статуса вызова фунции создания задачи
    res_Creating=xTaskCreatePinnedToCore(Checking_Power_Loss_Task, "CHECKPWR", 100, NULL, 4, &power_loss_desc, 1); //Создаем задачу проверки присутствия питания
    check_res_creating_task(res_Creating); //Проверка статуса вызова фунции создания задачи
    res_Creating=xTaskCreatePinnedToCore(guiTask, "gui", 4096*2, NULL, 4, &gui_desc, 0);  //Создаем задачу графического интерфейса
    check_res_creating_task(res_Creating); //Проверка статуса вызова фунции создания задачи
    res_Creating=xTaskCreatePinnedToCore(lv_tick_task, "lv_tick", 100, NULL, 4, &lv_tick_desc, 0);  //Создаем задачу графического интерфейса
    check_res_creating_task(res_Creating); //Проверка статуса вызова фунции создания задачи

    printf("Creating queues\n");
    printf("Creating pressure queue\n");
    xQueue_press=xQueueCreate(PRESSURE_QUEUE_LENGTH, SIZE_DATA_OF_THE_PRESSURE_QUEUE); //Создание очереди значений давления
    check_res_creating_queue(xQueue_press);
    printf("Creating Button_queue\n");
    xQueue_touch=xQueueCreate(TOUCH_QUEUE_LENGTH, SIZE_DATA_OF_THE_TOUCH_QUEUE); //Создание очереди значений сенсорных кнопок
    check_res_creating_queue(xQueue_touch);
    printf("Creating concentration queue\n");
    xQueue_CS=xQueueCreate(CS_QUEUE_LENGTH, SIZE_DATA_OF_THE_CS_QUEUE); //Создание очереди значений датчика концентрации
    check_res_creating_queue(xQueue_CS);
    printf("Creating LVGL signals queue\n");
	xQueue_signals_LVGL=xQueueCreate(SIGNALS_LVGL_QUEUE_LENGTH, SIZE_DATA_OF_THE_SIGNALS_LVGL_QUEUE); //Создание очереди сигналов LVGL
	check_res_creating_queue(xQueue_signals_LVGL);
	printf("Creating LVGL data queue\n");
	xQueue_signals_LVGL=xQueueCreate(DATA_SERVICE_LVGL_QUEUE_LENGTH, SIZE_DATA_SERVICE_OF_THE_DATA_LVGL_QUEUE); //Создание очереди данных LVGL от потока настройки
	check_res_creating_queue(xQueue_signals_LVGL);

    printf("Creating semaphores\n");
    clap2_semph = xSemaphoreCreateBinary();
    check_res_creating_semph(clap2_semph);

    //Инициализация Flash
    printf("Opening Non-Volatile Storage (NVS) handle... ");
    err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		 // NVS partition was truncated and needs to be erased
		 // Retry nvs_flash_init
		 ESP_ERROR_CHECK(nvs_flash_erase());
		 err = nvs_flash_init();
	 }
	 ESP_ERROR_CHECK(err);
	 err = nvs_open("storage", NVS_READWRITE, &flash_handle);
	 if (err != ESP_OK) {
		 printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
	 }
	 else{
		 printf("Done\n");
	 }
	 //Загрузка T1-T4 и OFFSET из Flash
	 err=nvs_get_u64(flash_handle, "T1", &T1);
	 if (err==ESP_ERR_NVS_NOT_FOUND){           //В случае если настройка аппарата не проводилась, то загрузить значения по умолчанию
		 printf("The value is not initialized yet!\n");
		 T1=DEFAULT_VALUE_T1;
		 T2=DEFAULT_VALUE_T2;
		 T3=DEFAULT_VALUE_T3;
		 T4=DEFAULT_VALUE_T4;
		 OFFSET=DEFAULT_VALUE_OFFSET;
	 }
	 else{                                     //После настройки загружать значения из Flash
		 nvs_get_u64(flash_handle, "T2", &T2);
		 nvs_get_u64(flash_handle, "T3", &T3);
		 nvs_get_u64(flash_handle, "T4", &T4);
		 nvs_get_u64(flash_handle, "OFFSET", &OFFSET);
	 }
	 vTaskStartScheduler();
	 for( ;; );
}
//Вычисление контрольной суммы датчика концентрации
uint8_t getCheckSum(uint8_t *getbuff){
	static int8_t i, checksum;
	for( i = 0; i < 11; i++)
	{
			checksum += getbuff [i];
	}
	checksum = 0 - checksum;
	return checksum;
}
//Проверка статуса вызова фунции создания задачи
void check_res_creating_task(BaseType_t res_Creating){
	if (res_Creating==errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY){
		printf("COULD NOT ALLOCATE REQUIRED MEMORY!\n The program is frozen\n");
		Infinit_loop1:
			goto Infinit_loop1;
	}
	else{
		printf("Successfully\n");
	}
}
//Проверка статуса создания очереди
void check_res_creating_queue(QueueHandle_t xQueue_descr){
	if (xQueue_descr==NULL){
		printf("The queue could not be created!\n The program is frozen\n");
		Infinit_loop2:
			goto Infinit_loop2;
	}
    else{
    	printf("Successfully\n");
    }
	return;
}
static void Time_Task(void *pvParameter){
	uint32_t tmp_m, tmp_h;
	//Загрузка значения времени из Flash
		 err = nvs_get_u64(flash_handle, "time_sec", &time_s);
		 if (err==ESP_ERR_NVS_NOT_FOUND){
			 printf("The value is not initialized yet!\n");
			 time_s=0;
		 }
	while (1){
		vTaskDelay(pdMS_TO_TICKS(1000));  //Каждую секунду увеличиваем значение времени
		time_s++;
		tmp_m++;
		tmp_h++;
		if (tmp_m==60){
			time_min_filter_coarse_clear++;
			time_min_filter_fine_clear++;
			tmp_m=0;
		}
		if (tmp_h==3600){
			time_h_filter_coarse_clear++;
			time_h_filter_fine_clear++;
			tmp_h=0;
		}
	}
}
//Считывание данных с датчика давления
static void ADC_Task(void *pvParameter){
	adc1_config_width(ADC_WIDTH_12Bit); //Разрядность данных АЦП ADC_WIDTH_BIT_12
   	adc1_config_channel_atten(ADC1_CHANNEL_0,ADC_ATTEN_DB_11); //Выбор канала и ослабления
   	while (1){
   		val=0;
   		for (uint8_t i=0; i<NUMBER_OF_VALUES; i++){
   			val += (int32_t)adc1_get_raw(ADC1_CHANNEL_0);        //Считывание NUMBER_OF_VALUES раз
   		}
   		val/=NUMBER_OF_VALUES; //Усреднение данных
   		//Сюда добавить калибровку
   		if (xQueueSendToBack( xQueue_press, &val, 10 ) != pdPASS){
   			printf("Warning! ADC Task: The pressure queue is overflowing\n");
   		}
   	}
}
   //Опрос сенсорных кнопок
static void Buttons_Task(void *pvParameter){
	static uint32_t touch_values[4];
	touch_pad_init();
	for (int i = 0; i < TOUCH_BUTTON_NUM; i++) {
		touch_pad_config(button[i]);
	}
	touch_pad_denoise_t denoise = {
			.grade = TOUCH_PAD_DENOISE_BIT4,
			.cap_level = TOUCH_PAD_DENOISE_CAP_L4,
	};
	touch_pad_denoise_set_config(&denoise);
	touch_pad_denoise_enable();
	touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
	touch_pad_fsm_start();
	while (1){
		for (uint8_t i = 0; i < TOUCH_BUTTON_NUM; i++) {
			touch_pad_read_raw_data(button[i], &touch_values[i]);    // Чтение значений
			xQueueSendToBack(xQueue_touch, &touch_values[i], portMAX_DELAY);          //Отправка значений в очередь
		}
		vTaskDelay(pdMS_TO_TICKS(20));
	}
}
static void Clapan1_Task(void *pvParameter){
	static uint8_t tmp;
	gpio_set_level(PWR_PUMP, TRUE);  //Включение компрессора
	tmp=pdMS_TO_TICKS(T1);
	Timer1=xTimerCreate("Timer1", tmp, pdTRUE, (void*) 0, T1_handler); //Создать таймер 1
	tmp=pdMS_TO_TICKS(T2);
	Timer2=xTimerCreate("Timer2", tmp, pdTRUE, (void*) 0, T2_handler); //Создать таймер 2
	tmp=pdMS_TO_TICKS(OFFSET);
	Timer_of_offset=xTimerCreate("Timer_of_offset", tmp, pdTRUE, (void*) 0, Timer_of_offset_handler); //Создать таймер 3
	xTimerStart(Timer3, portMAX_DELAY); //Запуск таймера 3
	start_T1:
	xTimerStart(Timer1, portMAX_DELAY); //Запуск таймера 1
	t1_end=FALSE;
	gpio_set_level(CLAP1, HL);
	check_t1_end:
	if (t1_end!=TRUE){
	   goto check_t1_end;    //Ожидание окончания высокого уровня вывода клапана 1
	}
	xTimerStop(Timer1, portMAX_DELAY); //Остановка таймера 1
	t1_end=FALSE;
	xTimerStart(Timer2, portMAX_DELAY); //Запуск таймера 2
	check_t2_end:
	if (t1_end!=TRUE){
	   goto check_t2_end;    //Ожидание окончания высокого уровня вывода клапана 2
	}
	xTimerStop(Timer2, portMAX_DELAY); //Остановка таймера 2
	t1_end=FALSE;
	goto start_T1;
}
static void Clapan2_Task(void *pvParameter){
   static uint8_t tmp;
   tmp=pdMS_TO_TICKS(T3);
   Timer3=xTimerCreate("Timer3", tmp, pdTRUE, (void*) 0, T3_handler); //Создать таймер 3
   tmp=pdMS_TO_TICKS(T4);
   Timer4=xTimerCreate("Timer4", tmp, pdTRUE, (void*) 0, T4_handler); //Создать таймер 4
   xSemaphoreTake(clap2_semph, portMAX_DELAY);
   start_T3:
   xTimerStart(Timer3, portMAX_DELAY); //Запуск таймера 3
   gpio_set_level(CLAP2, HL);
   t2_end=FALSE;
   check_t3_end:
   if (t2_end!=TRUE){
	   goto check_t3_end;    //Ожидание окончания высокого уровня вывода клапана 3
   }
   xTimerStop(Timer3, portMAX_DELAY); //Запуск таймера 3
   t2_end=FALSE;
   xTimerStart(Timer4, portMAX_DELAY); //Запуск таймера 4
   check_t4_end:
   if (t2_end!=TRUE){
	   goto check_t4_end;    //Ожидание окончания высокого уровня вывода клапана 4
   }
   xTimerStop(Timer4, portMAX_DELAY); //Запуск таймера 4
   t2_end=FALSE;
   goto start_T3;
}
   //Считывание данных с датчика концентрации
static void CS_Task(void *pvParameter){
static const int uart_num = UART_NUM_1;
static uint8_t data[12];
static uint8_t useful_data[6];
//static float converted_values[3];
//static uint16_t convert_value;
static int length = 0;
static uint8_t checksum;
//static uint16_t tmp;
//static uint8_t b;
//static uint8_t c;
	while (1){
		uart_get_buffered_data_len(uart_num, (size_t*)&length);
		if (length==12){
			uart_read_bytes(uart_num, data, length, 100);
			checksum=getCheckSum(data);
			if (data[11]!=checksum){
				printf("Danger! The checksums don't match\n The program is frozen\n Problems with communication with the concentration sensor. Line noise or poor contact");
				gpio_set_level(BEEP, HL);     //Запуск звукового сигнала
				vTaskSuspendAll(); //Остановка работы планировщика
				Infinit_loop3:
						goto Infinit_loop3;
			}
			for (uint8_t a=0; a<12; a++){
				if ((a>2)&&(a<9)){
					useful_data[a-3]=data[a];
				}
	//   			b=0;                //Перенести обработку в место после очереди
	//   			c=0;
	//   			transform:
	//				tmp=(uint16_t)useful_data[b];
	//				tmp<<8;
	//				b++;
	//				convert_value=(uint16_t)useful_data[b];
	//				b++;
	//				convert_value |= tmp;
	//				converted_values[c]=(float)convert_value;
	//				converted_values[c] /= 10;
	//				c++;
	//				if (c!=3){
	//				goto transform;
	//				}

			}
			if (xQueueSendToBack( xQueue_CS, useful_data, 10 ) != pdPASS){
				printf("Warning! CS Task: The CS queue is overflowing\n");
			}
		}
	}
}
static void Service_Task(void *pvParameter){
	static uint32_t value_touch[4];
	static uint8_t signal;
	while (1){
		if (!gpio_get_level(SERVICE_BUTTON)){  //Значения проинвертировано с помощью !
			waiting_for_the_squeeze:
			if (!gpio_get_level(SERVICE_BUTTON)){
				goto waiting_for_the_squeeze;
			}
			vTaskDelay(pdMS_TO_TICKS(RATTLE_RELAXATION_TIME_MS));
			if (gpio_get_level(SERVICE_BUTTON)){
				//Остановка всех потоков кроме LVGL
				nvs_set_u64(flash_handle, "time_sec", time_s); //Запись значения времени во Flash
				nvs_commit(flash_handle);
				vTaskSuspend(time_desc);
				vTaskSuspend(clapan1_desc);
				vTaskSuspend(clapan2_desc);
				vTaskSuspend(adc_desc);
				vTaskSuspend(cs_desc);
				signal=SERVICE_DISP;
				xQueueSendToBack(xQueue_signals_LVGL, signal, portMAX_DELAY);
////////
				check_button_change_parameter_T1:
				READ_TOUCH(value_touch);
				vTaskDelay(pdMS_TO_TICKS(LIMITING_RATE_OF_PARAMETER_CHANGE_MS));
				if (value_touch[PARAMETER_SELECTION]>CLICK_THRESHOLD){
					check_release_T1:
					READ_TOUCH(value_touch);
					if(!(value_touch[PARAMETER_SELECTION]<=CLICK_THRESHOLD)){goto check_release_T1;}
					signal=MOVE_TO_T1_DISP;
					xQueueSendToBack(xQueue_signals_LVGL, signal, portMAX_DELAY);
					check_button_change_T1:
					READ_TOUCH(value_touch);
					vTaskDelay(pdMS_TO_TICKS(LIMITING_RATE_OF_PARAMETER_CHANGE_MS));
					if(value_touch[INCREASE]>CLICK_THRESHOLD){
						T1++;
						signal=INCREASE_PARAMETER_DISP;
						xQueueSendToBack(xQueue_signals_LVGL, signal, portMAX_DELAY);
						goto check_button_change_T1;
					}
					READ_TOUCH(value_touch);
					if(value_touch[DECREASE]>CLICK_THRESHOLD){
						T1--;
						signal=DECREASE_PARAMETER_DISP;
						xQueueSendToBack(xQueue_signals_LVGL, signal, portMAX_DELAY);
						goto check_button_change_T1;
					}
					READ_TOUCH(value_touch);
					if(!(value_touch[PARAMETER_SELECTION]>CLICK_THRESHOLD)){
						goto check_button_change_parameter_T1;
					}
					loc_lab1:
					READ_TOUCH(value_touch);
					if (value_touch[PARAMETER_SELECTION]>CLICK_THRESHOLD){
						goto  loc_lab1;
					}

/////////
					check_button_change_parameter_T2:
					vTaskDelay(pdMS_TO_TICKS(LIMITING_RATE_OF_PARAMETER_CHANGE_MS));
					READ_TOUCH(value_touch);
					if (value_touch[PARAMETER_SELECTION]>CLICK_THRESHOLD){
						check_release_T2:
							READ_TOUCH(value_touch);
							if(!(value_touch[PARAMETER_SELECTION]<=CLICK_THRESHOLD)){goto check_release_T2;}
						signal=MOVE_TO_T2_DISP;
						xQueueSendToBack(xQueue_signals_LVGL, signal, portMAX_DELAY);
						check_button_change_T2:
						READ_TOUCH(value_touch);
						if (value_touch[INCREASE]>CLICK_THRESHOLD){
							T2++;
							signal=INCREASE_PARAMETER_DISP;
							xQueueSendToBack(xQueue_signals_LVGL, signal, portMAX_DELAY);
							goto check_button_change_T2;
						}
						READ_TOUCH(value_touch);
						if (value_touch[DECREASE]>CLICK_THRESHOLD){
							T2--;
							signal=DECREASE_PARAMETER_DISP;
							xQueueSendToBack(xQueue_signals_LVGL, signal, portMAX_DELAY);
							goto check_button_change_T2;
						}
						READ_TOUCH(value_touch);
						if (!(value_touch[PARAMETER_SELECTION]>CLICK_THRESHOLD)){
							goto check_button_change_parameter_T2;
						}
						loc_lab4:
						READ_TOUCH(value_touch);
						if(!(value_touch[PARAMETER_SELECTION]<=CLICK_THRESHOLD)){
							goto loc_lab4;
						}
/////////
						check_button_change_parameter_T3:
						vTaskDelay(pdMS_TO_TICKS(LIMITING_RATE_OF_PARAMETER_CHANGE_MS));
						READ_TOUCH(value_touch);
						if (value_touch[PARAMETER_SELECTION]>CLICK_THRESHOLD){
							check_release_T3:
							READ_TOUCH(value_touch);
							if(!(value_touch[PARAMETER_SELECTION]<=CLICK_THRESHOLD)){goto check_release_T3;}
							signal=MOVE_TO_T3_DISP;
							xQueueSendToBack(xQueue_signals_LVGL, signal, portMAX_DELAY);
							check_button_change_T3:
							READ_TOUCH(value_touch);
							if (value_touch[INCREASE]>CLICK_THRESHOLD){
								T3++;
								signal=INCREASE_PARAMETER_DISP;
								xQueueSendToBack(xQueue_signals_LVGL, signal, portMAX_DELAY);
								goto check_button_change_T3;
							}
							READ_TOUCH(value_touch);
							if (value_touch[DECREASE]>CLICK_THRESHOLD){
								T3--;
								signal=DECREASE_PARAMETER_DISP;
								xQueueSendToBack(xQueue_signals_LVGL, signal, portMAX_DELAY);
								goto check_button_change_T3;
							}
							READ_TOUCH(value_touch);
							if (!(value_touch[PARAMETER_SELECTION]>CLICK_THRESHOLD)){
								goto check_button_change_T3;
							}
							READ_TOUCH(value_touch);
							if (!(value_touch[PARAMETER_SELECTION]>CLICK_THRESHOLD)){
								goto check_button_change_parameter_T3;
							}
							loc_lab5:
							READ_TOUCH(value_touch);
							if(!(value_touch[PARAMETER_SELECTION]<=CLICK_THRESHOLD)){
								goto loc_lab5;
							}
////////
							check_button_change_parameter_T4:
							vTaskDelay(pdMS_TO_TICKS(LIMITING_RATE_OF_PARAMETER_CHANGE_MS));
							READ_TOUCH(value_touch);
							if (value_touch[PARAMETER_SELECTION]>CLICK_THRESHOLD){
								check_release_T4:
								READ_TOUCH(value_touch);
								if(!(value_touch[PARAMETER_SELECTION]<=CLICK_THRESHOLD)){goto check_release_T4;}
								signal=MOVE_TO_T4_DISP;
								xQueueSendToBack(xQueue_signals_LVGL, signal, portMAX_DELAY);
								check_button_change_T4:
								READ_TOUCH(value_touch);
								if (value_touch[INCREASE]>CLICK_THRESHOLD){
									T4++;
									signal=INCREASE_PARAMETER_DISP;
									xQueueSendToBack(xQueue_signals_LVGL, signal, portMAX_DELAY);
									goto check_button_change_T4;
								}
								READ_TOUCH(value_touch);
								if (value_touch[DECREASE]>CLICK_THRESHOLD){
									T4--;
									signal=DECREASE_PARAMETER_DISP;
									xQueueSendToBack(xQueue_signals_LVGL, signal, portMAX_DELAY);
									goto check_button_change_T4;
								}
								READ_TOUCH(value_touch);
								if (!(value_touch[PARAMETER_SELECTION]>CLICK_THRESHOLD)){
									goto check_button_change_T4;
								}
								READ_TOUCH(value_touch);
								if (!(value_touch[PARAMETER_SELECTION]>CLICK_THRESHOLD)){
									goto check_button_change_parameter_T4;
								}
								loc_lab6:
								READ_TOUCH(value_touch);
								if(!(value_touch[PARAMETER_SELECTION]<=CLICK_THRESHOLD)){
									goto loc_lab6;
								}
///////
								check_button_change_parameter_offset:
								vTaskDelay(pdMS_TO_TICKS(LIMITING_RATE_OF_PARAMETER_CHANGE_MS));
								READ_TOUCH(value_touch);
								if (value_touch[PARAMETER_SELECTION]>CLICK_THRESHOLD){
									check_release_offset:
									READ_TOUCH(value_touch);
									if(!(value_touch[PARAMETER_SELECTION]<=CLICK_THRESHOLD)){goto check_release_offset;}
									signal=MOVE_TO_OFFSET_DISP;
									xQueueSendToBack(xQueue_signals_LVGL, signal, portMAX_DELAY);
									check_button_change_offset:
									READ_TOUCH(value_touch);
									if (value_touch[INCREASE]>CLICK_THRESHOLD){
										OFFSET++;
										signal=INCREASE_PARAMETER_DISP;
										xQueueSendToBack(xQueue_signals_LVGL, signal, portMAX_DELAY);
										goto check_button_change_offset;
									}
									READ_TOUCH(value_touch);
									if (value_touch[DECREASE]>CLICK_THRESHOLD){
										OFFSET--;
										signal=DECREASE_PARAMETER_DISP;
										xQueueSendToBack(xQueue_signals_LVGL, signal, portMAX_DELAY);
										goto check_button_change_offset;
									}
									READ_TOUCH(value_touch);
									if (!(value_touch[PARAMETER_SELECTION]>CLICK_THRESHOLD)){
										goto check_button_change_offset;
									}
									READ_TOUCH(value_touch);
									if (!(value_touch[PARAMETER_SELECTION]>CLICK_THRESHOLD)){
										goto check_button_change_parameter_offset;
									}
									loc_lab7:
									READ_TOUCH(value_touch);
									if(!(value_touch[PARAMETER_SELECTION]<=CLICK_THRESHOLD)){
										goto loc_lab7;
									}
									goto check_button_change_T1;
								}
								else{
									if (!(gpio_get_level(SERVICE_BUTTON))){
										goto check_button_change_offset;
									}
									goto loc_lab2;
								}
							}
							else{
								if (!(gpio_get_level(SERVICE_BUTTON))){
									goto check_button_change_T4;
								}
								goto loc_lab2;
							}
						}
						else{
							if (!(gpio_get_level(SERVICE_BUTTON))){
								goto check_button_change_T3;
							}
							goto loc_lab2;
						}
					}//T2
					else{
						if (!(gpio_get_level(SERVICE_BUTTON))){
							goto check_button_change_T2;
						}
						goto loc_lab2;
					}
				}
				else{
					if (!(gpio_get_level(SERVICE_BUTTON))){
						goto check_button_change_T1;
					}
					goto loc_lab2;
				}
			}
		}
	}
	loc_lab2:
	if (gpio_get_level(SERVICE_BUTTON)){
		goto loc_lab2;
	}
	//Запись параметров во Flash
	nvs_set_u64(flash_handle, "T1", T1);
	nvs_set_u64(flash_handle, "T2", T2);
	nvs_set_u64(flash_handle, "T3", T3);
	nvs_set_u64(flash_handle, "T4", T4);
	nvs_set_u64(flash_handle, "OFFSET", OFFSET);
	nvs_commit(flash_handle);
	esp_restart();
}
void check_res_creating_semph(SemaphoreHandle_t xSemph){
   if(xSemph == NULL){
	   printf("It is impossible to create a semaphore!\n The program is frozen\n");
	   Infinit_loop3:
		   goto Infinit_loop3;
   }
   else{
	   printf("Successfully\n");
   }
   return;
}
void T3_handler(TimerHandle_t xTimer){
  gpio_set_level(CLAP2, LL);
  t2_end=TRUE;
}
void T4_handler(TimerHandle_t xTimer){
  t2_end=TRUE;
}
void T1_handler(TimerHandle_t xTimer){
  gpio_set_level(CLAP1, LL);
  t1_end=TRUE;
}
void T2_handler(TimerHandle_t xTimer){
  t1_end=TRUE;
}
void Timer_of_offset_handler(TimerHandle_t xTimer){
	xSemaphoreTake(clap2_semph, portMAX_DELAY); //Взять семафор перед первым использованием
	xSemaphoreGive(clap2_semph); //Дать семафор вход упралвения клапана 2 запускается
}
static void Checking_Power_Loss_Task(void *pvParameter){
	static int abs_PWR;
	static uint32_t tmp[4];
	while (1){
		check_PWR:
		abs_PWR=gpio_get_level(PWR);
		if ((abs_PWR!=1)){
			gpio_set_level(BEEP, HL);  //Постоянное излучение звука после потери питания
			nvs_set_u64(flash_handle, "time_sec", time_s); //Запись значения времени во Flash
			nvs_commit(flash_handle);
			vTaskSuspend(service_desc); //Переводим все задачи, ненужные в момент потери питания, в приостановленное состояние
			vTaskSuspend(time_desc);
			vTaskSuspend(clapan1_desc);
			vTaskSuspend(clapan2_desc);
			vTaskSuspend(adc_desc);
			vTaskSuspend(gui_desc);
			vTaskSuspend(cs_desc);
			gpio_set_level(PWR_DISP, LL);      //Отключить дисплей
			gpio_set_level(PWR_LED_DISP, LL);  //Отключить подсветку дисплея
			check_touches_Checking_Power_Loss_Task:
			READ_TOUCH(tmp);
			abs_PWR=gpio_get_level(PWR);
			if ((tmp[RESUME_AFTER_POWER_IS_RESTORED]>=CLICK_THRESHOLD)&&(abs_PWR==1)){     //Если питание вернулось и нажата кнопка продолжения, то возобновляем работу
				vTaskResume(service_desc);
				vTaskResume(time_desc);
				vTaskResume(clapan1_desc);
				vTaskResume(clapan2_desc);
				vTaskResume(adc_desc);
				vTaskResume(gui_desc);
				vTaskResume(cs_desc);
				nvs_get_u64(flash_handle, "time_sec", &time_s); //Восстанавливаем время
				gpio_set_level(BEEP, LL);  //Отключаем звук
				gpio_set_level(PWR_DISP, HL);      //Включить дисплей
				gpio_set_level(PWR_LED_DISP, HL);  //Включить подсветку дисплея
				goto check_PWR;
			}
			if (tmp[RESETTING_THE_MACHINE]>=CLICK_THRESHOLD){      //Если нажата кнопка сброса, то переводим МК в режим глубокого сна
				esp_sleep_enable_ext0_wakeup(PWR, HL);        //Условие пробуждения МК - высокий уровень на выводе PWR - питание вернулось
				esp_deep_sleep_start();
				esp_restart();  //После возвращения питания перезапускаем микроконтроллер
			}
			goto check_touches_Checking_Power_Loss_Task;
		}
	}
}
static void guiTask(void *pvParameter){
	(void) pvParameter;
	xGuiSemaphore = xSemaphoreCreateMutex();

	lv_init();

	static lv_disp_drv_t disp_drv;
	lvgl_driver_init();

	lv_color_t* buf1 = heap_caps_malloc(lv_disp_get_hor_res(NULL) * sizeof(lv_color_t), MALLOC_CAP_DMA);
	assert(buf1 != NULL);

	lv_color_t* buf2 = heap_caps_malloc(lv_disp_get_hor_res(NULL) * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2 != NULL);


    static lv_disp_draw_buf_t disp_buf;

    uint32_t size_in_px = lv_disp_get_hor_res(NULL);

    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, size_in_px);

    lv_disp_drv_init(&disp_drv);
    disp_drv.flush_cb = disp_driver_flush;

    disp_drv.draw_buf = &disp_buf;
    lv_disp_drv_register(&disp_drv);

	const esp_timer_create_args_t periodic_timer_args = {
		.callback = &lv_tick_task,
		.name = "periodic_gui"
	};

	 esp_timer_handle_t periodic_timer;
	 ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
	 ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, LV_TICK_PERIOD_MS * 1000));

	 create_application_gui();

	 while (1) {
		/* Delay 1 tick (assumes FreeRTOS tick is 10ms */
		vTaskDelay(pdMS_TO_TICKS(10));

		/* Try to take the semaphore, call lvgl related function on success */
		if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
			lv_task_handler();
			xSemaphoreGive(xGuiSemaphore);
		}
	 }
	 free(buf1);
#ifndef CONFIG_LV_TFT_DISPLAY_MONOCHROME
	 free(buf2);
#endif
    vTaskDelete(NULL);
}
static void create_application_gui(void){
	//Стили
	//Стиль основного текста
//	static lv_style_t style_main_text;
	lv_style_init(&style_main_text);
	lv_style_set_text_font(&style_main_text, LV_FONT_MONTSERRAT_16);
	//Стиль индексов
//	static lv_style_t style_index_text;
	lv_style_init(&style_index_text);
	lv_style_set_text_font(&style_index_text, LV_FONT_MONTSERRAT_10);
	//Стиль полей ввода
//	static lv_style_t style_input_field_text;
	lv_style_init(&style_input_field_text);
	lv_style_set_text_font(&style_input_field_text, LV_FONT_MONTSERRAT_16);
	lv_style_set_text_color(&style_input_field_text, lv_color_black());
	lv_style_set_border_color(&style_input_field_text, lv_color_black());
	lv_style_set_border_side(&style_input_field_text, LV_BORDER_SIDE_BOTTOM);
	lv_style_set_border_side(&style_input_field_text, LV_BORDER_SIDE_NONE);
	lv_style_set_border_width(&style_input_field_text, 2);
	//Стиль выбираемого текста
//	static lv_style_t style_selectable_text;
	lv_style_init(&style_selectable_text);
	lv_style_set_text_font(&style_selectable_text, LV_FONT_MONTSERRAT_16);
	lv_style_set_border_color(&style_selectable_text, lv_color_black());
//	lv_style_set_border_side(&style_selectable_text, LV_STATE_FOCUSED, LV_BORDER_SIDE_BOTTOM);
//	lv_style_set_border_side(&style_selectable_text, LV_STATE_DEFAULT, LV_BORDER_SIDE_NONE);
	lv_style_set_border_width(&style_selectable_text, 2);
	//Стиль Кнопок
//	static lv_style_t style_button_text;
	lv_style_init(&style_button_text);
	lv_style_set_text_font(&style_button_text, LV_FONT_MONTSERRAT_16);
	//Стиль индексов низшего уровня
//	static lv_style_t style_sub_index_text;
	lv_style_init(&style_sub_index_text);
	lv_style_set_text_font(&style_sub_index_text, LV_FONT_MONTSERRAT_8);

	//Экраны
	//Стартовый экран
	/*lv_obj_t **/ screen_start=lv_obj_create(NULL);
	//Создаем приглашение экрана
	/*lv_obj_t **/ start_lab1=lv_label_create(screen_start);
	lv_label_set_text(start_lab1, "Configure the parameters\n" "Enter the time of the procedure");
	lv_obj_align(start_lab1, LV_ALIGN_CENTER, -1, -66);
	lv_obj_add_style(start_lab1, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Обозначение переменной
	//t
	/*lv_obj_t*/ * start_lab2=lv_label_create(screen_start);
	lv_label_set_text(start_lab2, "t");
	lv_obj_align(start_lab2, LV_ALIGN_CENTER, -19, -24);
	lv_obj_add_style(start_lab2, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Индекс
	/*lv_obj_t **/ start_lab3=lv_label_create(screen_start);
	lv_label_set_text(start_lab3, "ep");
	lv_obj_align(start_lab3, LV_ALIGN_CENTER, -11, -18);
	lv_obj_add_style(start_lab3, &style_index_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Поле ввода минут
	/*lv_obj_t **/ start_lab4=lv_label_create(screen_start);
	text_start1=start_lab4;
	lv_label_set_text(start_lab4, "00");
	lv_obj_align(start_lab4, LV_ALIGN_CENTER, -77, 5);
	lv_obj_add_style(start_lab4, &style_input_field_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//min
	/*lv_obj_t **/ start_lab5=lv_label_create(screen_start);
	lv_label_set_text(start_lab5, "min");
	lv_obj_align(start_lab5, LV_ALIGN_CENTER, -49, 6);
	lv_obj_add_style(start_lab5, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//:
	/*lv_obj_t **/ start_lab6=lv_label_create(screen_start);
	lv_label_set_text(start_lab6, ":");
	lv_obj_align(start_lab6, LV_ALIGN_CENTER, -17, 7);
	lv_obj_add_style(start_lab6, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Поле ввода секунд
	/*lv_obj_t **/ start_lab7=lv_label_create(screen_start);
	text_start2=start_lab7;
	lv_label_set_text(start_lab7, "00");
	lv_obj_align(start_lab7, LV_ALIGN_CENTER, 7, 6);
	lv_obj_add_style(start_lab7, &style_input_field_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//sec
	/*lv_obj_t **/ start_lab8=lv_label_create(screen_start);
	lv_label_set_text(start_lab8, "sec");
	lv_obj_align(start_lab8, LV_ALIGN_CENTER, 36, 6);
	lv_obj_add_style(start_lab8, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Поле выбора поятоянной работы
	/*lv_obj_t **/ start_lab9=lv_label_create(screen_start);
	lv_label_set_text(start_lab9, "A constant task");
	lv_obj_align(start_lab9, LV_ALIGN_CENTER, -10, 32);
	lv_obj_add_style(start_lab9, &style_selectable_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Поле выбора Additionally
	/*lv_obj_t **/ start_lab10=lv_label_create(screen_start);
	lv_label_set_text(start_lab10, "Additionally");
	lv_obj_align(start_lab10, LV_ALIGN_CENTER, -10, 52);
	lv_obj_add_style(start_lab10, &style_selectable_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Поле выбора Start
	/*lv_obj_t **/ start_lab11=lv_label_create(screen_start);
	lv_label_set_text(start_lab11, "Additionally");
	lv_obj_align(start_lab11, LV_ALIGN_CENTER, -16, 70);
	lv_obj_add_style(start_lab11, &style_selectable_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Кнопка Enter
	/*lv_obj_t **/ start_lab12=lv_label_create(screen_start);
	lv_label_set_text(start_lab12, "Enter");
	lv_obj_align(start_lab12, LV_ALIGN_CENTER, -82, 109);
	lv_obj_add_style(start_lab12, &style_button_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Кнопка Switching
	/*lv_obj_t **/ start_lab13=lv_label_create(screen_start);
	lv_label_set_text(start_lab13, "Switching");
	lv_obj_align(start_lab13, LV_ALIGN_CENTER, -70, 109);
	lv_obj_add_style(start_lab13, &style_button_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Кнопка Increase
	/*lv_obj_t **/ start_lab14=lv_label_create(screen_start);
	lv_label_set_text(start_lab14, "Increase");
	lv_obj_align(start_lab14, LV_ALIGN_CENTER, 126, -29);
	lv_obj_add_style(start_lab14, &style_button_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Кнопка Decrease
	/*lv_obj_t **/ start_lab15=lv_label_create(screen_start);
	lv_label_set_text(start_lab15, "Decrease");
	lv_obj_align(start_lab15, LV_ALIGN_CENTER, 123, 38);
	lv_obj_add_style(start_lab15, &style_button_text, LV_STATE_DEFAULT | LV_PART_MAIN);

	//Экран работы
	/*lv_obj_t **/ screen_task=lv_obj_create(NULL);
	//Tic
	/*lv_obj_t */* task_lab1=lv_label_create(screen_task);
	lv_label_set_text(task_lab1, "Tic:");
	lv_obj_align(task_lab1, LV_ALIGN_CENTER, -142, -108);
	lv_obj_add_style(task_lab1, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Индикатор температуры внутри корпуса
	/*lv_obj_t **/ Tic_bar=lv_bar_create(screen_task);
	lv_bar_set_range(Tic_bar, 0, 100);
	lv_obj_set_size(Tic_bar, 150, 10);
	lv_obj_align(Tic_bar, LV_ALIGN_CENTER, -27, -109);
	lv_bar_set_value(Tic_bar, 0, LV_ANIM_ON);
	//Значение температуры внутри корпуса
	/*lv_obj_t **/ task_lab2=lv_label_create(screen_task);
	lv_label_set_text(task_lab2, "0");
	lv_obj_align(task_lab2, LV_ALIGN_CENTER, 66, -111);
	lv_obj_add_style(task_lab2, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Градусы
	/*lv_obj_t **/ task_lab3=lv_label_create(screen_task);
	lv_label_set_text(task_lab3, "0");
	lv_obj_align(task_lab3, LV_ALIGN_CENTER, 87, -116);
	lv_obj_add_style(task_lab3, &style_index_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//C Цельсии
	/*lv_obj_t **/ task_lab4=lv_label_create(screen_task);
	lv_label_set_text(task_lab4, "C");
	lv_obj_align(task_lab4, LV_ALIGN_CENTER, 97, -109);
	lv_obj_add_style(task_lab4, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//C
	/*lv_obj_t **/ task_lab5=lv_label_create(screen_task);
	lv_label_set_text(task_lab5, "C  :");
	lv_obj_align(task_lab5, LV_ALIGN_CENTER, -139, -96);
	lv_obj_add_style(task_lab5, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//O
	/*lv_obj_t **/ task_lab6=lv_label_create(screen_task);
	lv_label_set_text(task_lab6, "O");
	lv_obj_align(task_lab6, LV_ALIGN_CENTER, -136, -93);
	lv_obj_add_style(task_lab6, &style_index_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//2
	/*lv_obj_t **/ task_lab7=lv_label_create(screen_task);
	lv_label_set_text(task_lab7, "2");
	lv_obj_align(task_lab7, LV_ALIGN_CENTER, -129, -90);
	lv_obj_add_style(task_lab7, &style_sub_index_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Индикатор концентрации
	/*lv_obj_t **/ c_bar=lv_bar_create(screen_task);
	lv_bar_set_range(c_bar, 0, 100);
	lv_obj_set_size(c_bar, 150, 10);
	lv_obj_align(c_bar, LV_ALIGN_CENTER, -27, -94);
	lv_bar_set_value(c_bar, 0, LV_ANIM_ON);
	//Значение концентрации
	/*lv_obj_t **/ task_lab8=lv_label_create(screen_task);
	lv_label_set_text(task_lab8, "0");
	lv_obj_align(task_lab8, LV_ALIGN_CENTER, 68, -94);
	lv_obj_add_style(task_lab8, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//%
	/*lv_obj_t **/ task_lab9=lv_label_create(screen_task);
	lv_label_set_text(task_lab9, "%");
	lv_obj_align(task_lab9, LV_ALIGN_CENTER, 94, -94);
	lv_obj_add_style(task_lab9, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//F
	/*lv_obj_t **/ task_lab10=lv_label_create(screen_task);
	lv_label_set_text(task_lab10, "F:");
	lv_obj_align(task_lab10, LV_ALIGN_CENTER, -147, -80);
	lv_obj_add_style(task_lab10, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Индикатор потока
	/*lv_obj_t **/ F_bar=lv_bar_create(screen_task);
	lv_bar_set_range(F_bar, 0, 100);
	lv_obj_set_size(F_bar, 150, 10);
	lv_obj_align(F_bar, LV_ALIGN_CENTER, -26, -78);
	lv_bar_set_value(F_bar, 0, LV_ANIM_ON);
	//Значение потока
	/*lv_obj_t **/ task_lab11=lv_label_create(screen_task);
	lv_label_set_text(task_lab11, "0");
	lv_obj_align(task_lab11, LV_ALIGN_CENTER, 68, -79);
	lv_obj_add_style(task_lab11, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//L/min
	/*lv_obj_t **/ task_lab12=lv_label_create(screen_task);
	lv_label_set_text(task_lab12, "L/min");
	lv_obj_align(task_lab12, LV_ALIGN_CENTER, 106, -79);
	lv_obj_add_style(task_lab12, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//P
	/*lv_obj_t **/ task_lab13=lv_label_create(screen_task);
	lv_label_set_text(task_lab13, "P:");
	lv_obj_align(task_lab13, LV_ALIGN_CENTER, -147, -67);
	lv_obj_add_style(task_lab13, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Индикатор давления
	/*lv_obj_t **/ P_bar=lv_bar_create(screen_task);
	lv_bar_set_range(P_bar, 0, 100);
	lv_obj_set_size(P_bar, 150, 10);
	lv_obj_align(P_bar, LV_ALIGN_CENTER, -25, -65);
	lv_bar_set_value(P_bar, 0, LV_ANIM_ON);
	//Значение давления
	/*lv_obj_t **/ task_lab14=lv_label_create(screen_task);
	lv_label_set_text(task_lab14, "0");
	lv_obj_align(task_lab14, LV_ALIGN_CENTER, 68, -65);
	lv_obj_add_style(task_lab14, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//MPa
	/*lv_obj_t **/ task_lab15=lv_label_create(screen_task);
	lv_label_set_text(task_lab15, "MPa");
	lv_obj_align(task_lab15, LV_ALIGN_CENTER, 101, -65);
	lv_obj_add_style(task_lab15, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//T
	/*lv_obj_t **/ task_lab16=lv_label_create(screen_task);
	lv_label_set_text(task_lab16, "T  :");
	lv_obj_align(task_lab16, LV_ALIGN_CENTER, -136, -53);
	lv_obj_add_style(task_lab16, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//O
	/*lv_obj_t **/ task_lab35=lv_label_create(screen_task);
	lv_label_set_text(task_lab35, "O");
	lv_obj_align(task_lab35, LV_ALIGN_CENTER, -139, -50);
	lv_obj_add_style(task_lab35, &style_index_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//2
	/*lv_obj_t **/ task_lab17=lv_label_create(screen_task);
	lv_label_set_text(task_lab17, "2");
	lv_obj_align(task_lab17, LV_ALIGN_CENTER, -129, -48);
	lv_obj_add_style(task_lab17, &style_sub_index_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Индикатор температуры кислорода
	/*lv_obj_t **/ TO2_bar=lv_bar_create(screen_task);
	lv_bar_set_range(TO2_bar, 0, 100);
	lv_obj_set_size(TO2_bar, 150, 10);
	lv_obj_align(TO2_bar, LV_ALIGN_CENTER, -25, -50);
	lv_bar_set_value(TO2_bar, 0, LV_ANIM_ON);
	//Значение температуры кислорода
	/*lv_obj_t **/ task_lab18=lv_label_create(screen_task);
	lv_label_set_text(task_lab18, "0");
	lv_obj_align(task_lab18,LV_ALIGN_CENTER, -69, -50);
	lv_obj_add_style(task_lab18, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//0
	/*lv_obj_t **/ task_lab19=lv_label_create(screen_task);
	lv_label_set_text(task_lab19, "0");
	lv_obj_align(task_lab19, LV_ALIGN_CENTER, 89, -56);
	lv_obj_add_style(task_lab19, &style_index_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//C
	/*lv_obj_t **/ task_lab20=lv_label_create(screen_task);
	lv_label_set_text(task_lab20, "C");
	lv_obj_align(task_lab20, LV_ALIGN_CENTER, 97, -49);
	lv_obj_add_style(task_lab20, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//t
	/*lv_obj_t **/ task_lab21=lv_label_create(screen_task);
	lv_label_set_text(task_lab21, "t  :");
	lv_obj_align(task_lab21, LV_ALIGN_CENTER, -138, -37);
	lv_obj_add_style(task_lab21, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//ep
	/*lv_obj_t **/ task_lab22=lv_label_create(screen_task);
	lv_label_set_text(task_lab22, "ep");
	lv_obj_align(task_lab22, LV_ALIGN_CENTER, -138, -37);
	lv_obj_add_style(task_lab22, &style_index_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Значение минут до окончания процедуры
	/*lv_obj_t **/ task_lab23=lv_label_create(screen_task);
	lv_label_set_text(task_lab23, "0");
	lv_obj_align(task_lab23, LV_ALIGN_CENTER, -90, -34);
	lv_obj_add_style(task_lab23, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//:
	/*lv_obj_t **/ task_lab24=lv_label_create(screen_task);
	lv_label_set_text(task_lab24, ":");
	lv_obj_align(task_lab24, LV_ALIGN_CENTER, -79, -34);
	lv_obj_add_style(task_lab24, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Значения секунд до окончания процедуры
	/*lv_obj_t **/ task_lab36=lv_label_create(screen_task);
	lv_label_set_text(task_lab36, "0");
	lv_obj_align(task_lab36, LV_ALIGN_CENTER, -68, -34);
	lv_obj_add_style(task_lab36, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//min:sec
	/*lv_obj_t **/ task_lab25=lv_label_create(screen_task);
	lv_label_set_text(task_lab25, "min:sec");
	lv_obj_align(task_lab25, LV_ALIGN_CENTER, -56, -16);
	lv_obj_add_style(task_lab25,  &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//t
	/*lv_obj_t **/ task_lab26=lv_label_create(screen_task);
	lv_label_set_text(task_lab26, "t  :");
	lv_obj_align(task_lab26, LV_ALIGN_CENTER, -138, -17);
	lv_obj_add_style(task_lab26, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//bp
	/*lv_obj_t **/ task_lab27=lv_label_create(screen_task);
	lv_label_set_text(task_lab27, "bp");
	lv_obj_align(task_lab27, LV_ALIGN_CENTER, -136, -15);
	lv_obj_add_style(task_lab27, &style_index_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Значения минут от начала процедуры
	/*lv_obj_t **/ task_lab28=lv_label_create(screen_task);
	lv_label_set_text(task_lab28, "0");
	lv_obj_align(task_lab28, LV_ALIGN_CENTER, -90, -17);
	lv_obj_add_style(task_lab28, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//:
	/*lv_obj_t **/ task_lab29=lv_label_create(screen_task);
	lv_label_set_text(task_lab29, ":");
	lv_obj_align(task_lab29, LV_ALIGN_CENTER, -78, -18);
	lv_obj_add_style(task_lab29, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Значения секунд до окончания процедуры
	/*lv_obj_t **/ task_lab30=lv_label_create(screen_task);
	lv_label_set_text(task_lab30, "0");
	lv_obj_align(task_lab30, LV_ALIGN_CENTER, -67, -17);
	lv_obj_add_style(task_lab30, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//min:sec
	/*lv_obj_t **/ task_lab31=lv_label_create(screen_task);
	lv_label_set_text(task_lab31, "min:sec");
	lv_obj_align(task_lab31, LV_ALIGN_CENTER, -30, -17);
	lv_obj_add_style(task_lab31, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//A task is in progress
	/*lv_obj_t **/ task_lab32=lv_label_create(screen_task);
	lv_label_set_text(task_lab32, "A task is in progress");
	lv_obj_align(task_lab32, LV_ALIGN_CENTER, -4, 8);
	lv_obj_add_style(task_lab32, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Stop
	/*lv_obj_t **/ task_lab33=lv_label_create(screen_task);
	lv_label_set_text(task_lab33, "Stop");
	lv_obj_align(task_lab33, LV_ALIGN_CENTER, -84, 110);
	lv_obj_add_style(task_lab33, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Pause
	/*lv_obj_t **/ task_lab34=lv_label_create(screen_task);
	lv_label_set_text(task_lab34, "Stop");
	lv_obj_align(task_lab34, LV_ALIGN_CENTER, 71, 106);
	lv_obj_add_style(task_lab34, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);

	//Экран информации
	/*lv_obj_t **/ screen_filters_inform=lv_obj_create(NULL);
	//Время до мойки фго
	/*lv_obj_t **/ filters_inform_lab1=lv_label_create(screen_filters_inform);
	lv_label_set_text(filters_inform_lab1, "Remaining time before cleaning/n the coarse filter:");
	lv_obj_align(filters_inform_lab1, LV_ALIGN_CENTER, 7, -94);
	lv_obj_add_style(filters_inform_lab1, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Время до мойки фильтра грубой очистки в часах
	/*lv_obj_t **/ filters_inform_lab2=lv_label_create(screen_filters_inform);
	lv_label_set_text(filters_inform_lab2, "0");
	lv_obj_align(filters_inform_lab2, LV_ALIGN_CENTER, -27, -62);
	lv_obj_add_style(filters_inform_lab2, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//:
	/*lv_obj_t **/ filters_inform_lab3=lv_label_create(screen_filters_inform);
	lv_label_set_text(filters_inform_lab3, "0");
	lv_obj_align(filters_inform_lab3, LV_ALIGN_CENTER, -4, -62);
	lv_obj_add_style(filters_inform_lab3, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Время до мойки фильтра грубой очистки в минутах
	/*lv_obj_t **/ filters_inform_lab4=lv_label_create(screen_filters_inform);
	lv_label_set_text(filters_inform_lab4, "0");
	lv_obj_align(filters_inform_lab4, LV_ALIGN_CENTER, 15, -62);
	lv_obj_add_style(filters_inform_lab4, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//h:min
	/*lv_obj_t **/ filters_inform_lab5=lv_label_create(screen_filters_inform);
	lv_label_set_text(filters_inform_lab5, "h:min");
	lv_obj_align(filters_inform_lab5, LV_ALIGN_CENTER, 48, -62);
	lv_obj_add_style(filters_inform_lab5, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Время до мойки фто
	/*lv_obj_t **/ filters_inform_lab6=lv_label_create(screen_filters_inform);
	lv_label_set_text(filters_inform_lab6, "Remaining time before cleaning\n the coarse filter:");
	lv_obj_align(filters_inform_lab6, LV_ALIGN_CENTER, 0, 0);
	lv_obj_add_style(filters_inform_lab6, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Время до мойки фильтра тонкой очистки в часах
	/*lv_obj_t **/ filters_inform_lab7=lv_label_create(screen_filters_inform);
	lv_label_set_text(filters_inform_lab7, "0");
	lv_obj_align(filters_inform_lab7, LV_ALIGN_CENTER, -23, 30);
	lv_obj_add_style(filters_inform_lab7, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//:
	/*lv_obj_t **/ filters_inform_lab8=lv_label_create(screen_filters_inform);
	lv_label_set_text(filters_inform_lab8, ":");
	lv_obj_align(filters_inform_lab8, LV_ALIGN_CENTER, 1, 30);
	lv_obj_add_style(filters_inform_lab8, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Время до мойки фильтра тонкой очистки в минутах
	/*lv_obj_t **/ filters_inform_lab9=lv_label_create(screen_filters_inform);
	lv_label_set_text(filters_inform_lab9, "0");
	lv_obj_align(filters_inform_lab9, LV_ALIGN_CENTER, 17, 30);
	lv_obj_add_style(filters_inform_lab9, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//h:min
	/*lv_obj_t **/ filters_inform_lab10=lv_label_create(screen_filters_inform);
	lv_label_set_text(filters_inform_lab10, "h:min");
	lv_obj_align(filters_inform_lab10, LV_ALIGN_CENTER, 51, 31);
	lv_obj_add_style(filters_inform_lab10, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Оставшееся врямя до замены фильтров
	/*lv_obj_t **/ filters_inform_lab11=lv_label_create(screen_filters_inform);
	lv_label_set_text(filters_inform_lab11, "Full replasment time:");
	lv_obj_align(filters_inform_lab11, LV_ALIGN_CENTER, 56, 56);
	lv_obj_add_style(filters_inform_lab11, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Грубой очистки
	/*lv_obj_t **/ filters_inform_lab12=lv_label_create(screen_filters_inform);
	lv_label_set_text(filters_inform_lab12, "Coarse:");
	lv_obj_align(filters_inform_lab12, LV_ALIGN_CENTER, -4, 79);
	lv_obj_add_style(filters_inform_lab12, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Значение времени до замены фильтра грубой очистки в часах
	/*lv_obj_t **/ filters_inform_lab13=lv_label_create(screen_filters_inform);
	lv_label_set_text(filters_inform_lab13, "0");
	lv_obj_align(filters_inform_lab13, LV_ALIGN_CENTER, 58, 79);
	lv_obj_add_style(filters_inform_lab13, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//h
	/*lv_obj_t **/ filters_inform_lab14=lv_label_create(screen_filters_inform);
	lv_label_set_text(filters_inform_lab14, "h");
	lv_obj_align(filters_inform_lab14, LV_ALIGN_CENTER, 101, 80);
	lv_obj_add_style(filters_inform_lab14, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Тонкой очистки
	/*lv_obj_t **/ filters_inform_lab15=lv_label_create(screen_filters_inform);
	lv_label_set_text(filters_inform_lab15, "Fine:");
	lv_obj_align(filters_inform_lab15, LV_ALIGN_CENTER, -13, 99);
	lv_obj_add_style(filters_inform_lab15, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Значение времени до замены фильтра грубой очистки в часах
	/*lv_obj_t **/ filters_inform_lab16=lv_label_create(screen_filters_inform);
	lv_label_set_text(filters_inform_lab16, "0");
	lv_obj_align(filters_inform_lab16, LV_ALIGN_CENTER, 57, 99);
	lv_obj_add_style(filters_inform_lab16, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//h
	/*lv_obj_t **/ filters_inform_lab17=lv_label_create(screen_filters_inform);
	lv_label_set_text(filters_inform_lab17, "h");
	lv_obj_align(filters_inform_lab17, LV_ALIGN_CENTER, 101, 99);
	lv_obj_add_style(filters_inform_lab17,  &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Кнопка обновления информации о фильтрах
	/*lv_obj_t **/ filters_inform_lab18=lv_label_create(screen_filters_inform);
	lv_label_set_text(filters_inform_lab18, "Updating\n information\n about filters");
	lv_obj_align(filters_inform_lab18, LV_ALIGN_CENTER, -88, 96);
	lv_obj_add_style(filters_inform_lab18, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Кнопка возврата
	/*lv_obj_t **/ filters_inform_lab19=lv_label_create(screen_filters_inform);
	lv_label_set_text(filters_inform_lab19, "Updating\n information\n about filters");
	lv_obj_align(filters_inform_lab19, LV_ALIGN_CENTER, 134, -39);
	lv_obj_add_style(filters_inform_lab19, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);

	//Экран обновления информации о фильтрах
	/*lv_obj_t **/ screen_filters_new=lv_obj_create(NULL);
	//Приглащение обновить информацию о фильтрах
	/*lv_obj_t **/ filters_new_lab1=lv_label_create(screen_filters_new);
	lv_label_set_text(filters_new_lab1, "Update information\n about filters");
	lv_obj_align(filters_new_lab1, LV_ALIGN_CENTER, 0, 0);
	lv_obj_add_style(filters_new_lab1, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Кнопка фго
	/*lv_obj_t **/ filters_new_lab2=lv_label_create(screen_filters_new);
	lv_label_set_text(filters_new_lab2, "Coarse\n filter");
	lv_obj_align(filters_new_lab2, LV_ALIGN_CENTER, -80, 102);
	lv_obj_add_style(filters_new_lab2, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Кнопка фто
	/*lv_obj_t **/ filters_new_lab3=lv_label_create(screen_filters_new);
	lv_label_set_text(filters_new_lab3, "Fine\n filter");
	lv_obj_align(filters_new_lab3, LV_ALIGN_CENTER, 55, 102);
	lv_obj_add_style(filters_new_lab3, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Кнопка возвращения
	/*lv_obj_t **/ filters_new_lab4=lv_label_create(screen_filters_new);
	lv_label_set_text(filters_new_lab4, "Back");
	lv_obj_align(filters_new_lab4, LV_ALIGN_CENTER, 132, -17);
	lv_obj_add_style(filters_new_lab4, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);

	//Экран обновления информации о фильтре грубой очистки
	/*lv_obj_t **/ screen_filters_coarse=lv_obj_create(NULL);
	//Приглащение фильтр грубой очистки
	/*lv_obj_t **/ filters_coarse_lab1=lv_label_create(screen_filters_coarse);
	lv_label_set_text(filters_coarse_lab1, "Coarse filter");
	lv_obj_align(filters_coarse_lab1, LV_ALIGN_CENTER, 0, 0);
	lv_obj_add_style(filters_coarse_lab1, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Кнопка Cleared
	/*lv_obj_t **/ filters_coarse_lab2=lv_label_create(screen_filters_new);
	lv_label_set_text(filters_coarse_lab2, "Cleared");
	lv_obj_align(filters_coarse_lab2, LV_ALIGN_CENTER, -80, 102);
	lv_obj_add_style(filters_coarse_lab2, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Кнопка New
	/*lv_obj_t **/ filters_coarse_lab3=lv_label_create(screen_filters_new);
	lv_label_set_text(filters_coarse_lab3, "New");
	lv_obj_align(filters_coarse_lab3, LV_ALIGN_CENTER, 55, 102);
	lv_obj_add_style(filters_coarse_lab3, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Кнопка возвращения
	/*lv_obj_t **/ filters_coarse_lab4=lv_label_create(screen_filters_new);
	lv_label_set_text(filters_coarse_lab4, "Back");
	lv_obj_align(filters_coarse_lab4, LV_ALIGN_CENTER, 132, -17);
	lv_obj_add_style(filters_coarse_lab4, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);

	//Экран вопроса о фильтре грубой очистки
	/*lv_obj_t **/ screen_question_cleared_coarse=lv_obj_create(NULL);
	//Приглащение фильтр грубой очистки
	/*lv_obj_t **/ filters_question_cleared_coarse_lab1=lv_label_create(screen_question_cleared_coarse);
	lv_label_set_text(filters_question_cleared_coarse_lab1, "Is the filter/n cleared?");
	lv_obj_align(filters_question_cleared_coarse_lab1, LV_ALIGN_CENTER, 0, 0);
	lv_obj_add_style(filters_question_cleared_coarse_lab1, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Кнопка Cleared
	/*lv_obj_t **/ filters_question_cleared_coarse_lab2=lv_label_create(screen_question_cleared_coarse);
	lv_label_set_text(filters_question_cleared_coarse_lab2, "Yes");
	lv_obj_align(filters_question_cleared_coarse_lab2, LV_ALIGN_CENTER, -80, 102);
	lv_obj_add_style(filters_question_cleared_coarse_lab2, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Кнопка New
	/*lv_obj_t **/ filters_question_cleared_coarse_lab3=lv_label_create(screen_question_cleared_coarse);
	lv_label_set_text(filters_question_cleared_coarse_lab3, "No");
	lv_obj_align(filters_question_cleared_coarse_lab3, LV_ALIGN_CENTER, 55, 102);
	lv_obj_add_style(filters_question_cleared_coarse_lab3, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Кнопка возвращения
	/*lv_obj_t **/ filters_question_cleared_coarse_lab4=lv_label_create(screen_question_cleared_coarse);
	lv_label_set_text(filters_question_cleared_coarse_lab4, "Back");
	lv_obj_align(filters_question_cleared_coarse_lab4, LV_ALIGN_CENTER, 132, -17);
	lv_obj_add_style(filters_question_cleared_coarse_lab4, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);

	//Экран вопроса о замене грубой очистки
	/*lv_obj_t **/ screen_question_new_coarse=lv_obj_create(NULL);
	//Приглащение фильтр грубой очистки
	/*lv_obj_t **/ filters_question_new_coarse_lab1=lv_label_create(screen_question_new_coarse);
	lv_label_set_text(filters_question_new_coarse_lab1, "A new filter is installed?");
	lv_obj_align(filters_question_new_coarse_lab1, LV_ALIGN_CENTER, 0, 0);
	lv_obj_add_style(filters_question_new_coarse_lab1, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Кнопка Cleared
	/*lv_obj_t **/ filters_question_new_coarse_lab2=lv_label_create(screen_question_new_coarse);
	lv_label_set_text(filters_question_new_coarse_lab2, "Yes");
	lv_obj_align(filters_question_new_coarse_lab2, LV_ALIGN_CENTER, -80, 102);
	lv_obj_add_style(filters_question_new_coarse_lab2, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Кнопка New
	/*lv_obj_t **/ filters_question_new_coarse_lab3=lv_label_create(screen_question_new_coarse);
	lv_label_set_text(filters_question_new_coarse_lab3, "No");
	lv_obj_align(filters_question_new_coarse_lab3, LV_ALIGN_CENTER, 55, 102);
	lv_obj_add_style(filters_question_new_coarse_lab3, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Кнопка возвращения
	/*lv_obj_t **/ filters_question_new_coarse_lab4=lv_label_create(screen_question_new_coarse);
	lv_label_set_text(filters_question_new_coarse_lab4, "Back");
	lv_obj_align(filters_question_new_coarse_lab4, LV_ALIGN_CENTER, 132, -17);
	lv_obj_add_style(filters_question_new_coarse_lab4, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);

	//Экран обновления информации о фильтре тонкой очистки
	/*lv_obj_t **/ screen_filters_fine=lv_obj_create(NULL);
	//Приглащение фильтр тонкой очистки
	/*lv_obj_t **/ filters_fine_lab1=lv_label_create(screen_filters_fine);
	lv_label_set_text(filters_fine_lab1, "Fine filter");
	lv_obj_align(filters_fine_lab1, LV_ALIGN_CENTER, 0, 0);
	lv_obj_add_style(filters_coarse_lab1, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Кнопка Cleared
	/*lv_obj_t **/ filters_fine_lab2=lv_label_create(screen_filters_fine);
	lv_label_set_text(filters_fine_lab2, "Cleared");
	lv_obj_align(filters_fine_lab2, LV_ALIGN_CENTER, -80, 102);
	lv_obj_add_style(filters_fine_lab2, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Кнопка New
	/*lv_obj_t **/ filters_fine_lab3=lv_label_create(screen_filters_fine);
	lv_label_set_text(filters_fine_lab3, "New");
	lv_obj_align(filters_fine_lab3, LV_ALIGN_CENTER, 55, 102);
	lv_obj_add_style(filters_fine_lab3, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Кнопка возвращения
	/*lv_obj_t **/ filters_fine_lab4=lv_label_create(screen_filters_fine);
	lv_label_set_text(filters_fine_lab4, "Back");
	lv_obj_align(filters_fine_lab4, LV_ALIGN_CENTER, 132, -17);
	lv_obj_add_style(filters_fine_lab4, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);

	//Экран вопроса о промывке фильтре тонкой очистки
	/*lv_obj_t **/ screen_question_cleared_fine=lv_obj_create(NULL);
	//Приглащение
	/*lv_obj_t **/ filters_question_cleared_fine_lab1=lv_label_create(screen_question_cleared_fine);
	lv_label_set_text(filters_question_cleared_fine_lab1, "Is the filter/n cleared?");
	lv_obj_align(filters_question_cleared_fine_lab1, LV_ALIGN_CENTER, 0, 0);
	lv_obj_add_style(filters_question_cleared_fine_lab1, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Кнопка Yes
	/*lv_obj_t **/ filters_question_cleared_fine_lab2=lv_label_create(screen_question_cleared_fine);
	lv_label_set_text(filters_question_cleared_fine_lab2, "Yes");
	lv_obj_align(filters_question_cleared_fine_lab2, LV_ALIGN_CENTER, -80, 102);
	lv_obj_add_style(filters_question_cleared_fine_lab2, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Кнопка No
	/*lv_obj_t **/ filters_question_cleared_fine_lab3=lv_label_create(screen_question_cleared_fine);
	lv_label_set_text(filters_question_cleared_fine_lab3, "No");
	lv_obj_align(filters_question_cleared_fine_lab3, LV_ALIGN_CENTER, 55, 102);
	lv_obj_add_style(filters_question_cleared_fine_lab3, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Кнопка возвращения
	/*lv_obj_t **/ filters_question_cleared_fine_lab4=lv_label_create(screen_question_cleared_fine);
	lv_label_set_text(filters_question_cleared_fine_lab4, "Back");
	lv_obj_align(filters_question_cleared_fine_lab4, LV_ALIGN_CENTER, 132, -17);
	lv_obj_add_style(filters_question_cleared_fine_lab4, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);

	//Экран вопроса о замене фильра тонкой очистки
	/*lv_obj_t **/ screen_question_new_fine=lv_obj_create(NULL);
	//Приглащение фильтр тонкой очистки
	/*lv_obj_t **/ filters_question_new_fine_lab1=lv_label_create(screen_question_new_fine);
	lv_label_set_text(filters_question_new_fine_lab1, "A new filter is installed?");
	lv_obj_align(filters_question_new_fine_lab1, LV_ALIGN_CENTER, 0, 0);
	lv_obj_add_style(filters_question_new_fine_lab1, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Кнопка Yes
	/*lv_obj_t **/ filters_question_new_fine_lab2=lv_label_create(screen_question_new_fine);
	lv_label_set_text(filters_question_new_fine_lab2, "Yes");
	lv_obj_align(filters_question_new_fine_lab2, LV_ALIGN_CENTER, -80, 102);
	lv_obj_add_style(filters_question_new_fine_lab2, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Кнопка No
	/*lv_obj_t **/ filters_question_new_fine_lab3=lv_label_create(screen_question_new_fine);
	lv_label_set_text(filters_question_new_fine_lab3, "No");
	lv_obj_align(filters_question_new_fine_lab3, LV_ALIGN_CENTER, 55, 102);
	lv_obj_add_style(filters_question_new_fine_lab3, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);
	//Кнопка возвращения
	/*lv_obj_t **/ filters_question_new_fine_lab4=lv_label_create(screen_question_new_fine);
	lv_label_set_text(filters_question_new_fine_lab4, "Back");
	lv_obj_align(filters_question_new_fine_lab4, LV_ALIGN_CENTER, 132, -17);
	lv_obj_add_style(filters_question_new_fine_lab4, &style_main_text, LV_STATE_DEFAULT | LV_PART_MAIN);

}
static void LVGL_Task(void * arg){
	static uint8_t screen=0;
	static uint32_t value_touch[4];
	uint8_t pointer_start;
	pointer_start=0;
	READ_TOUCH(value_touch);
	if (value_touch[INCREASE]>=CLICK_THRESHOLD){
		h_increase_button_release_check:
		READ_TOUCH(value_touch);
		if (value_touch[INCREASE]>=CLICK_THRESHOLD){
			goto h_increase_button_release_check;
		}
		pt_m++;
		if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {

			xSemaphoreGive(xGuiSemaphore);
		}
	}
	if ()
}
static void lv_tick_task(void *arg) {
    (void) arg;
    while (1){
    	lv_tick_inc(LV_TICK_PERIOD_MS);
    }
}
void READ_TOUCH(uint32_t * tmp){   //Чтение сенсорных кнопок
	xQueueReset(xQueue_touch); 	   //Очистить очередь сенсорных кнопок
	for (uint8_t i=0; i<TOUCH_QUEUE_LENGTH; i++){
		xQueueReceive(xQueue_touch, &tmp[i], portMAX_DELAY);
	}
}
void check_filters(void * arg){
	static const uint32_t time_before_cleaning_coarse_filter_m=100*60;  //Фильтр грубой очистки очищается раз в 100 часов
	static const uint32_t time_before_cleaning_fine_filter_m=300*60;  //Фильтр тонкой очистки очищается раз в 300 часов
	static uint32_t tmp;
	while (1){
		tmp=(uint32_t)time_h_filter_coarse_clear*60;
		tmp+=time_min_filter_coarse_clear;
		if (tmp>=time_before_cleaning_coarse_filter_m){
			coarse_filter_cleaning_is_required=TRUE;  //Установить флаг требования очистки ФГО
		}
		tmp=(uint32_t)time_h_filter_fine_clear*60;
		tmp+=time_min_filter_fine_clear;
		if (tmp>=time_before_cleaning_fine_filter_m){
			fine_filter_cleaning_is_required=TRUE;  //Установить флаг требования очистки ФТО
		}
	}
}
