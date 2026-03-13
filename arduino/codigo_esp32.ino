#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>

// Configuración de Hardware
#define IN1 12
#define IN2 13
#define ENC_A 18
#define ENC_B 19
#define LED_PIN 2

// Configuración PWM
#define PWM_CH1 0
#define PWM_CH2 1
#define PWM_FREQ 980
// 8 bits 0 a 255
#define PWM_RES 8

// Parámetros de Encoder y Velocidad
volatile long pulsos = 0;
long pulsos_ant = 0;
unsigned long t_prev = 0;
// Pulsos por revolucion del motor 
const float PPR = 408.0;

// Filtro de Paso Bajo para suavizado de la señal de velocidad
static float w_filtrada = 0.0f;
const float alpha = 0.05f; 


// Objetos de comunicación micro-ROS
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;
rcl_node_t node;
rcl_timer_t timer;
rcl_subscription_t subscriber;
std_msgs__msg__Float32 msg_sub;
rcl_publisher_t publisher;
std_msgs__msg__Float32 msg_pub;

//Manejo de errores critico o no critico
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// En caso de error el led permanecera encendido 
void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
  }
}

// Interrupción para conteo de pulsos del encoder donde encA dispara y encB determinara el sentido + o -
void IRAM_ATTR encoder_callback() {
  if (digitalRead(ENC_B) == HIGH) pulsos++;
  else pulsos--;
}

//Timer de pulso del led 
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void) last_call_time;
  if (timer != NULL) digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}

// Recibe la acción de control (de -1 a 1) y acciona los drivers del motor
void subscription_callback(const void * msgin) {
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;

  // Por seguridad contenemos datos dentro de -1 y 1
  float u = constrain(msg->data, -1.0f, 1.0f);
  // Mediante el producto convertimos el valor absoluto de la ccion del control a un valor dentro del rango 0 - 255
  int pwm_val = (int)(fabsf(u) * 255.0f);

  // Logica para determinar direccion incluyendo una zona muerta teorica no calculada
  if (u > 0.05f) {
    ledcWrite(PWM_CH1, pwm_val);
    ledcWrite(PWM_CH2, 0);
  } else if (u < -0.05f) {
    ledcWrite(PWM_CH1, 0);
    ledcWrite(PWM_CH2, pwm_val);
  } else {
    ledcWrite(PWM_CH1, 0);
    ledcWrite(PWM_CH2, 0);
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  
  // Configuración de canales PWM
  ledcSetup(PWM_CH1, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH2, PWM_FREQ, PWM_RES);
  ledcAttachPin(IN1, PWM_CH1);
  ledcAttachPin(IN2, PWM_CH2);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoder_callback, RISING);

  set_microros_transports();
  allocator = rcl_get_default_allocator();

  //Creacion del nodo para comunicacion 
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_motor_node", "", &support));

  //Declaramos publisher y subscriber a la accion del control y la salida del motor
  RCCHECK(rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/motor_input_u"));
  RCCHECK(rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/motor_output"));

  //Timer del led en 2 segundos 
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(2000), timer_callback));

  //Declaramos dos tareas que serán el timer del led y la subscripcion del control 
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg_sub, &subscription_callback, ON_NEW_DATA));

  t_prev = millis();
}

void loop() {
  //Tareas pendientes del micro ros 
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

  unsigned long t_now = millis();
  if (t_now - t_prev >= 100) { // Frecuencia de muestreo --> Tiempo de sampleo debe coincidir con el determinado en el proyecto
    float dt = (t_now - t_prev) / 1000.0f;

    // Aseguramos que la lectura de pulsos sea correcta
    noInterrupts();
    long p_actual = pulsos;
    interrupts();

    //DIferencia de pulsos 
    long delta_p = p_actual - pulsos_ant;

    // Cálculo de velocidad angular: (pulsos / PPR) --> vueltas --> (2PI) --> rad --> (/dt) --> rad/s
    float w_cruda = ((float)delta_p / PPR) * (2.0f * PI) / dt;

    // Aplicación de filtro de suavizado EMA
    w_filtrada = alpha * w_cruda + (1.0f - alpha) * w_filtrada;
    //Si reducimos alpha obtendremos mayor suavizado pero podria ocasionar retraso al graficar por observar datos pasados

    //PUblicamos la velocidad filtrada al topico motor_output
    msg_pub.data = w_filtrada;
    RCSOFTCHECK(rcl_publish(&publisher, &msg_pub, NULL));

    pulsos_ant = p_actual;
    t_prev = t_now;
  }
}
