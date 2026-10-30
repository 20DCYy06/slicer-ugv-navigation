/* C30D board-level references: local F407VET6 resource tables + AT8236 datasheet.
 * HSI 16MHz avoids depending on external crystal startup. No auto motor test.
 * Protocol: A5 5A v1 type len seqLE payload CRC16-CCITT. SLC2 handshake.
 */
#include "stm32f4xx.h"
#include "generated.h"
#include <stdint.h>
#include "BALANCE/balance.h"
#include "HARDWARE/ICM20948/ICM20948.h"
#include "HARDWARE/usartx.h"

#ifdef SLC2_NATIVE_TEST
static volatile uint32_t trace[8];
#else
/* Persistent SRAM breadcrumbs, outside BSS and the reserved 8KiB stack. */
#define trace ((volatile uint32_t *)0x2001d000u)
#endif

static volatile uint32_t ms;
static volatile uint8_t rx[512], fault;
static volatile uint16_t rw,rr;
static volatile uint8_t serial_resync;
static uint8_t frame[73],used, configured, have_command, hello_ok,cal_index=255;
static uint8_t direct_mode,test_blocked;
static int16_t direct_pwm[4];
static uint32_t test_started;
static uint32_t nonce,last_command,last_control,last_report,cal_end;
static uint16_t seq,ack_seq;
static int16_t targets[4],actual[4],duty[4];
static int8_t pwm_sign[4],enc_sign[4];
static uint16_t previous[4],battery_mv;
static uint32_t raw_total[4],sum_counts[4];
static float integral[4],filtered[4];
static int16_t previous_target[4];
static uint16_t stalled[4],opposite[4];
static ControlSources sources;
static uint8_t active_source;
static uint8_t tx[512];
static uint16_t tx_write,tx_read;
#ifdef SLC2_NATIVE_TEST
static uint8_t last_tx[73];
static unsigned last_tx_size;
static uint8_t previous_tx[73];
#endif
/* Diagnostic-only snapshot. Existing stop conditions and thresholds unchanged.
 * 0x14 + session nonce requests one 0x91 reply; no extra periodic UART traffic. */
static int16_t cycle_delta[4];
static uint32_t control_interval_ms;
static uint8_t first_fault, fault_source, fault_mode, fault_inputs;
static uint32_t fault_ms, fault_interval, fault_uart_sr;
static int16_t fault_delta[4],fault_command[4],fault_pwm[4];
static USART_TypeDef *port=USART1;
static TIM_TypeDef *const enc[4]={TIM2,TIM3,TIM4,TIM5};
static uint16_t rd16(const uint8_t*p){return p[0]|((uint16_t)p[1]<<8);}
static void p16(uint8_t*p,uint16_t n){p[0]=n;p[1]=n>>8;}
static void p32(uint8_t*p,uint32_t n){p16(p,n);p16(p+2,n>>16);}
static uint32_t rd32(const uint8_t*p){return rd16(p)|((uint32_t)rd16(p+2)<<16);}
static float clamp(float x,float cap){return x>cap?cap:(x<-cap?-cap:x);}
static int absi(int n){return n<0?-n:n;}
static uint16_t crc(const uint8_t*p,unsigned n){
 uint16_t c=0xffff;while(n--){c^=(uint16_t)*p++<<8;for(unsigned i=0;i<8;i++)c=(c&0x8000)?(c<<1)^0x1021:c<<1;}return c;
}
static void zero(void){
 TIM10->CCR1=TIM11->CCR1=TIM9->CCR1=TIM9->CCR2=0;
 TIM1->CCR1=TIM1->CCR2=TIM1->CCR3=TIM1->CCR4=0;
 for(unsigned i=0;i<4;i++){duty[i]=0;integral[i]=0;previous_target[i]=0;}
 body_reset();
}
static void stop(void){zero();for(unsigned i=0;i<4;i++){targets[i]=direct_pwm[i]=0;stalled[i]=opposite[i]=0;}cal_index=255;have_command=direct_mode=0;sources=(ControlSources){0};active_source=SOURCE_STOP;}
static void latch_fault(uint8_t mask,uint8_t source,uint32_t uart_sr){
 /* Protect the first-cause snapshot against a UART IRQ interrupting main. */
 uint32_t irq_state=__get_PRIMASK();__disable_irq();
 if(!fault){
  first_fault=mask;fault_source=source;fault_ms=ms;
  fault_interval=control_interval_ms;fault_uart_sr=uart_sr;
  fault_mode=direct_mode?1:(cal_index!=255?3:(configured?2:0));
  fault_inputs=((GPIOD->IDR&8u)?1:0)|(battery_mv>=MIN_BATTERY_MV?2:0);
  for(unsigned i=0;i<4;i++){
   fault_delta[i]=cycle_delta[i];fault_command[i]=direct_mode?direct_pwm[i]:targets[i];
   fault_pwm[i]=duty[i];
  }
 }
 fault|=mask;__set_PRIMASK(irq_state);
}
static void clear_fault(void){
 uint32_t irq_state=__get_PRIMASK();__disable_irq();
 fault=first_fault=fault_source=fault_mode=fault_inputs=0;
 fault_ms=fault_interval=fault_uart_sr=0;
 for(unsigned i=0;i<4;i++)fault_delta[i]=fault_command[i]=fault_pwm[i]=0;
 __set_PRIMASK(irq_state);
}
void SysTick_Handler(void){ms++;trace[6]=ms;}
void Default_Handler(void){trace[2]=0xdead;trace[3]=SCB->CFSR;trace[4]=SCB->HFSR;zero();for(;;){} /* IWDG resets */}
void USART2_IRQHandler(void){(void)USART2->DR;}
static void receive(USART_TypeDef*u){
 uint32_t s=u->SR;uint8_t b=(uint8_t)u->DR;
 if(s&(USART_SR_ORE|USART_SR_FE|USART_SR_NE|USART_SR_PE)){
  /* A broken byte while stopped cannot have moved a motor. Discard the
   * partial frame; a subsequent valid CRC/nonce command may start motion.
   * During motion the original latched-fault stop remains mandatory. */
  if(have_command||direct_mode||cal_index!=255)latch_fault(1,1,s);
  else serial_resync=1;
  return;
 }
 if(!(s&USART_SR_RXNE)||u!=port)return;
 uint16_t next=(rw+1)&511;
 if(next==rr){latch_fault(1,2,s);return;}rx[rw]=b;rw=next;
}
void USART1_IRQHandler(void){receive(USART1);}
void USART3_IRQHandler(void){receive(USART3);}
static void emit(uint8_t kind,const uint8_t*p,unsigned n){
 uint8_t out[73];out[0]=0xa5;out[1]=0x5a;out[2]=1;out[3]=kind;out[4]=n;p16(out+5,seq++);
 for(unsigned i=0;i<n;i++)out[7+i]=p[i];p16(out+7+n,crc(out+2,n+5));
 /* 发送环形缓冲避免新增遥测阻塞100Hz控制；帧总量约3.6KB/s，低于串口带宽。 */
#ifdef SLC2_NATIVE_TEST
 for(unsigned i=0;i<last_tx_size;i++)previous_tx[i]=last_tx[i];
 for(unsigned i=0;i<n+9;i++)last_tx[i]=out[i];last_tx_size=n+9;
#else
 unsigned free=(tx_read-tx_write-1u)&511u;
 if(free<n+9)return;
 for(unsigned i=0;i<n+9;i++){tx[tx_write]=out[i];tx_write=(tx_write+1)&511;}
#endif
}
static void uart_pump(void){
 if(tx_read!=tx_write&&(port->SR&USART_SR_TXE)){
  port->DR=tx[tx_read];tx_read=(tx_read+1)&511;
 }
}
static int enabled(void){return (GPIOD->IDR&(1u<<3))!=0;}
static int powered(void){return battery_mv>=MIN_BATTERY_MV;}
static void command(void){
 uint8_t reply[14];const uint8_t*p=frame+7;unsigned n=frame[4];
 if(frame[3]==1&&n==4){
  stop();test_blocked=0;configured=0;hello_ok=1;nonce=rd32(p);if(!enabled())clear_fault();
  reply[0]='S';reply[1]='L';reply[2]='C';reply[3]='2';reply[4]=2;reply[5]=4;
  p16(reply+6,COUNTS_REV);p16(reply+8,WATCHDOG_MS);p32(reply+10,nonce);emit(0x81,reply,14);
 }else if(frame[3]==0x14&&n==4&&hello_ok&&rd32(p)==nonce){
  uint8_t diagnostic[42];uint32_t irq_state=__get_PRIMASK();__disable_irq();
  diagnostic[0]=1;diagnostic[1]=fault;diagnostic[2]=first_fault;diagnostic[3]=fault_source;
  p32(diagnostic+4,fault_ms);p32(diagnostic+8,fault_interval);p32(diagnostic+12,fault_uart_sr);
  for(unsigned i=0;i<4;i++){
   p16(diagnostic+16+2*i,fault_delta[i]);p16(diagnostic+24+2*i,fault_command[i]);
   p16(diagnostic+32+2*i,fault_pwm[i]);
  }
  diagnostic[40]=fault_mode;diagnostic[41]=fault_inputs;__set_PRIMASK(irq_state);
  emit(0x91,diagnostic,42);
 }else if(frame[3]==0x15&&n==4&&hello_ok&&rd32(p)==nonce){
  /* Read-only output-register snapshot; confirms timer programming without
   * treating a requested PWM value as proof that a wheel actually moved. */
  uint8_t registers[60];
  uint32_t values[15]={1,TIM1->CR1,TIM1->CCER,TIM1->BDTR,
   TIM1->CCR1,TIM1->CCR2,TIM1->CCR3,TIM1->CCR4,
   TIM9->CCR1,TIM9->CCR2,TIM10->CCR1,TIM11->CCR1,
   GPIOD->IDR,GPIOB->AFR[1],GPIOE->AFR[1]};
  for(unsigned i=0;i<15;i++)p32(registers+4*i,values[i]);
  emit(0x92,registers,60);
 }else if(frame[3]==0x13&&n==4&&hello_ok&&rd32(p)==nonce){
  p32(reply,nonce);reply[4]=1;p16(reply+5,TEST_PWM_CAP*1000u/PWM_PERIOD);p16(reply+7,TEST_MAX_MS);
  emit(0x83,reply,9);
 }else if(frame[3]==BODY_INFO_REQUEST&&n==4&&hello_ok&&rd32(p)==nonce){
  p32(reply,nonce);reply[4]=1;reply[5]=imu.who;
  reply[6]=(imu.present?1:0)|(imu.calibrated?2:0)|(icm20948_ready(ms)?4:0)|(imu.failed?8:0);
  reply[7]=active_source;emit(BODY_INFO_REPLY,reply,8);
 }else if(frame[3]==BODY_COMMAND&&n==8&&hello_ok&&rd32(p)==nonce){
  BodyCommand request;if(!body_decode_command(p,n,nonce,&request))return;
  if(!configured||fault||!enabled()||!powered())return;
  sources.vx_mm_s=request.vx_mm_s;sources.wz_mrad_s=request.wz_mrad_s;sources.nav_mode=SOURCE_BODY;
  sources.nav_valid=1;sources.nav_ms=ms;ack_seq=rd16(frame+5);
 }else if(frame[3]==0x11&&n==12&&hello_ok&&rd32(p)==nonce){
  int all_zero=1;
  for(unsigned i=0;i<4;i++){
   int v=(int16_t)rd16(p+4+2*i);if((unsigned)absi(v)>TEST_PWM_CAP*1000u/PWM_PERIOD)return;
   if(v)all_zero=0;
  }
  if(all_zero){sources.manual_valid=0;test_blocked=0;ack_seq=rd16(frame+5);return;}
  if(test_blocked||fault||!enabled()||!powered())return;
  if(!sources.manual_valid)test_started=ms;
  for(unsigned i=0;i<4;i++)sources.manual[i]=(int16_t)rd16(p+4+2*i);
  sources.manual_valid=1;sources.manual_ms=ms;ack_seq=rd16(frame+5);
 }else if(frame[3]==0x12&&n==12&&hello_ok&&rd32(p)==nonce){
  for(unsigned i=0;i<4;i++)if((p[4+i]!=1&&p[4+i]!=255)||(p[8+i]!=1&&p[8+i]!=255))return;
  stop();for(unsigned i=0;i<4;i++){pwm_sign[i]=(int8_t)p[4+i];enc_sign[i]=(int8_t)p[8+i];}
  configured=1;p32(reply,nonce);reply[4]=1;emit(0x82,reply,5);
 }else if(frame[3]==0x10&&n==8&&hello_ok){
  int all_zero=1;for(unsigned i=0;i<4;i++){
   int16_t v=(int16_t)rd16(p+2*i);if((unsigned)absi(v)>MAX_MM_S)return;if(v)all_zero=0;
  }
  if(all_zero){stop();test_blocked=0;last_command=ms;ack_seq=rd16(frame+5);return;}
  if(!configured||fault||!enabled()||!powered()||cal_index!=255)return;
  for(unsigned i=0;i<4;i++)sources.wheel[i]=(int16_t)rd16(p+2*i);
  sources.nav_mode=SOURCE_WHEEL;sources.nav_valid=1;sources.nav_ms=ms;ack_seq=rd16(frame+5);
 }else if(frame[3]==0x20&&n==9&&hello_ok&&rd32(p)==nonce){
  unsigned i=p[4];int16_t v=(int16_t)rd16(p+5);unsigned duration=rd16(p+7);
  if(i>3||v<=0||(unsigned)v>CAL_PERMILLE||duration==0||duration>CAL_MS||fault||!enabled()||!powered())return;
  stop();configured=0;cal_index=i;duty[i]=(int16_t)(v*PWM_PERIOD/1000);cal_end=ms+duration;
  last_command=ms;have_command=1;
 }
}
static void parse(uint8_t b){
 if(used==0){if(b==0xa5)frame[used++]=b;return;}
 if(used==1&&b!=0x5a){used=0;if(b==0xa5)frame[used++]=b;return;}
 frame[used++]=b;
 if(used>=5&&(frame[2]!=1||frame[4]>64)){used=0;return;}
 if(used>=7&&used==frame[4]+9u){
  if(crc(frame+2,used-4)==rd16(frame+used-2))command();used=0;
 }
}
static void pin(GPIO_TypeDef*g,unsigned p,unsigned mode,unsigned af,unsigned pull){
 g->MODER=(g->MODER&~(3u<<(p*2)))|(mode<<(p*2));
 g->OTYPER&=~(1u<<p);g->OSPEEDR|=3u<<(p*2);
 g->PUPDR=(g->PUPDR&~(3u<<(p*2)))|(pull<<(p*2));
 if(mode==2)g->AFR[p/8]=(g->AFR[p/8]&~(15u<<((p%8)*4)))|(af<<((p%8)*4));
}
static void pwm_init(TIM_TypeDef*t,unsigned channels){
 t->CR1=0;t->PSC=0;t->ARR=PWM_PERIOD-1;t->CCR1=0;
 if(channels>1)t->CCR2=0;
 if(channels==4)t->CCR3=t->CCR4=0;
 t->CCMR1=channels>1?0x6868:0x68;if(channels==4)t->CCMR2=0x6868;t->CCER=0;
 for(unsigned i=0;i<channels;i++)t->CCER|=1u<<(i*4);
 if(t==TIM1)t->BDTR=TIM_BDTR_MOE;
 t->EGR=1;t->CR1=TIM_CR1_ARPE|TIM_CR1_CEN;
}
static void pwm(unsigned i,int signed_value){
 unsigned v=absi(signed_value);if(v>PWM_CAP)v=PWM_CAP;
 /* 与实车已验证的test_pwm使用同一电桥调制：驱动侧恒高、另一侧互补PWM。
  * 只统一调制，不改变闭环pwm_signs所定义的电气方向；零仍为00。
  * 如A前进：闭环负号 -> TIM10恒高、TIM11=周期-v，与开环正号完全一致。 */
 unsigned in1=0,in2=0;
 if(v){in1=signed_value>0?PWM_PERIOD:PWM_PERIOD-v;
       in2=signed_value<0?PWM_PERIOD:PWM_PERIOD-v;}
 if(i==0){TIM11->CCR1=in1;TIM10->CCR1=in2;}
 else if(i==1){TIM9->CCR1=in1;TIM9->CCR2=in2;}
 else if(i==2){TIM1->CCR2=in1;TIM1->CCR1=in2;}
 else{TIM1->CCR4=in1;TIM1->CCR3=in2;}
}
static void encoder_init(TIM_TypeDef*t){
 t->CR1=0;t->PSC=0;t->ARR=65535;t->CCMR1=0x3131; /* TI1/TI2, filter=3 */
 t->CCER=0;t->SMCR=3;t->CNT=0;t->EGR=1;t->CR1=1;
}
static void test_pwm(unsigned i,int value){
 /* Vendor Set_Pwm uses high/high slow-decay PWM. Stop remains 00/coast.
  * A1=TIM10, A2=TIM11; B1/2=TIM9 CH1/2; C1/2=TIM1 CH2/1;
  * D1/2=TIM1 CH4/3. Positive A/C and B/D use opposite wiring conventions. */
 volatile uint32_t *a,*b;
 if(i==0){a=&TIM10->CCR1;b=&TIM11->CCR1;}
 else if(i==1){a=&TIM9->CCR1;b=&TIM9->CCR2;}
 else if(i==2){a=&TIM1->CCR2;b=&TIM1->CCR1;}
 else{a=&TIM1->CCR4;b=&TIM1->CCR3;}
 unsigned v=(unsigned)absi(value);if(v>TEST_PWM_CAP)v=TEST_PWM_CAP;
 if(!v){*a=*b=0;return;}
 int forward=(value>0)^((i==1)||(i==3));
 if(forward){*a=PWM_PERIOD;*b=PWM_PERIOD-v;}
 else{*b=PWM_PERIOD;*a=PWM_PERIOD-v;}
}
static uint16_t voltage(void){
 ADC1->CR2|=ADC_CR2_SWSTART;unsigned timeout=40000;
 while(!(ADC1->SR&ADC_SR_EOC)&&--timeout){}
 if(!timeout)return 0;
 return (uint16_t)((ADC1->DR*36300u)/4095u);
}
static void hardware(void){
 trace[0]=0x534c4332;trace[1]=1;trace[5]=RCC->CSR;
 trace[2]=trace[3]=trace[4]=trace[6]=trace[7]=0;
 /* Reset bootloader clock/peripheral state explicitly before application IRQs. */
 SysTick->CTRL=0;SCB->VTOR=0x08000000;SCB->CPACR|=0xfu<<20;__DSB();__ISB();
 for(unsigned i=0;i<8;i++){NVIC->ICER[i]=0xffffffffu;NVIC->ICPR[i]=0xffffffffu;}
 RCC->CR|=RCC_CR_HSION;while(!(RCC->CR&RCC_CR_HSIRDY)){}
 RCC->CFGR=0;while(RCC->CFGR&RCC_CFGR_SWS){}
 RCC->CR&=~RCC_CR_PLLON;
 RCC->AHB1ENR|=0x1f;RCC->APB1ENR|=0x0f|(1u<<18); /* TIM2..5 USART3 */
 RCC->APB2ENR|=(1u<<0)|(1u<<4)|(1u<<8)|(1u<<16)|(1u<<17)|(1u<<18);
 (void)RCC->AHB1ENR;
 trace[1]=2;
 /* Force all H-bridge pins low BEFORE selecting timer alternate functions. */
 unsigned bp[2]={8,9},ep[6]={5,6,9,11,13,14};
 GPIOB->BSRRH=(1u<<8)|(1u<<9);
 for(unsigned i=0;i<6;i++){GPIOE->BSRRH=1u<<ep[i];pin(GPIOE,ep[i],1,0,2);}
 for(unsigned i=0;i<2;i++)pin(GPIOB,bp[i],1,0,2);
 pwm_init(TIM1,4);pwm_init(TIM9,2);pwm_init(TIM10,1);pwm_init(TIM11,1);
 pin(GPIOB,8,2,3,2);pin(GPIOB,9,2,3,2);
 pin(GPIOE,5,2,3,2);pin(GPIOE,6,2,3,2);
 pin(GPIOE,9,2,1,2);pin(GPIOE,11,2,1,2);pin(GPIOE,13,2,1,2);pin(GPIOE,14,2,1,2);
 trace[1]=3;
 pin(GPIOA,15,2,1,1);pin(GPIOB,3,2,1,1);
 pin(GPIOB,4,2,2,1);pin(GPIOB,5,2,2,1);pin(GPIOB,6,2,2,1);pin(GPIOB,7,2,2,1);
 pin(GPIOA,0,2,2,1);pin(GPIOA,1,2,2,1);
 for(unsigned i=0;i<4;i++)encoder_init(enc[i]);
 /* Match vendor Enable_Pin(): switch grounds PD3 in OFF, released/up in ON. */
 pin(GPIOD,3,0,0,1);pin(GPIOB,0,3,0,0);
 ADC->CCR=1u<<16;ADC1->SMPR2=7u<<24;ADC1->SQR3=8;ADC1->CR2=ADC_CR2_ADON;
 trace[1]=4;
 pin(GPIOA,9,2,7,1);pin(GPIOA,10,2,7,1);
 USART1->CR1=0;USART1->CR2=0;USART1->CR3=0;USART1->BRR=(16000000u+BAUD/2)/BAUD;
 USART1->CR1=USART_CR1_UE|USART_CR1_TE|USART_CR1_RE|USART_CR1_RXNEIE;
 pin(GPIOD,8,2,7,1);pin(GPIOD,9,2,7,1);
 USART3->CR1=0;USART3->CR2=0;USART3->CR3=0;USART3->BRR=(16000000u+BAUD/2)/BAUD;
 USART3->CR1=USART_CR1_UE|USART_CR1_TE|USART_CR1_RE|USART_CR1_RXNEIE;
 NVIC_EnableIRQ(USART1_IRQn); /* USART1 default. USART3 optional future build. */
 trace[1]=5;
 SysTick_Config(16000);
 /* LSI ~32kHz /64 /250 ~=500ms: independent reset if main loop hangs. */
 RCC->CSR|=RCC_CSR_LSION;while(!(RCC->CSR&RCC_CSR_LSIRDY)){}
 /* Start before waiting for PR/RLR synchronization: the IWDG kernel clock
  * is gated until start. Matches ST HAL_IWDG_Init initialization order. */
 IWDG->KR=0xcccc;
 IWDG->KR=0x5555;IWDG->PR=4;IWDG->RLR=249;while(IWDG->SR&3u){}
 IWDG->KR=0xaaaa;
 trace[1]=6;
 __enable_irq();
}
static void select_source(void){
 uint8_t next=SOURCE_STOP;
 if(cal_index!=255)next=SOURCE_CAL;
 else if(sources.manual_valid&&(uint32_t)(ms-sources.manual_ms)<WATCHDOG_MS)next=SOURCE_PWM;
 else if(sources.nav_valid&&(uint32_t)(ms-sources.nav_ms)<WATCHDOG_MS)next=sources.nav_mode;
 if(next!=active_source){
  if(next!=SOURCE_CAL)zero(); /* 校准指令已先stop，再写入本次单轮PWM。 */
  for(unsigned i=0;i<4;i++)stalled[i]=opposite[i]=0;
 }
 active_source=next;direct_mode=next==SOURCE_PWM;have_command=next!=SOURCE_STOP;
 if(next==SOURCE_PWM){
  last_command=sources.manual_ms;
  for(unsigned i=0;i<4;i++)direct_pwm[i]=sources.manual[i];
 }else if(next==SOURCE_WHEEL){
  last_command=sources.nav_ms;
  for(unsigned i=0;i<4;i++)targets[i]=sources.wheel[i];
 }else if(next==SOURCE_BODY)last_command=sources.nav_ms;
}
static void wheel_control(float dt){
 float effort[4],saved[4],error[4],peak=PWM_CAP;
 for(unsigned i=0;i<4;i++){
  if(!targets[i]){
   duty[i]=0;integral[i]=0;previous_target[i]=0;
   stalled[i]=opposite[i]=0;effort[i]=saved[i]=error[i]=0;continue;
  }
  /* 换向不带入上一方向的积分，否则小幅反向目标仍会被旧积分顶住。 */
  if((int32_t)targets[i]*previous_target[i]<0)integral[i]=0;
  previous_target[i]=targets[i];saved[i]=integral[i];
  error[i]=targets[i]-filtered[i];
  integral[i]=clamp(integral[i]+error[i]*KI*dt,PWM_CAP);
  effort[i]=KP*error[i]+integral[i]+FEEDFORWARD*targets[i];
  if(effort[i]*targets[i]<0)effort[i]=0;
  float magnitude=effort[i]<0?-effort[i]:effort[i];
  if(magnitude>peak)peak=magnitude;
 }
 /* 四轮使用一个缩放系数，保留饱和前的差速比例；禁止分别截成相同满输出。
  * 饱和时冻结仍在推高输出的积分，反向误差仍可释放积分。 */
 if(peak>PWM_CAP){
  peak=PWM_CAP;
  for(unsigned i=0;i<4;i++){
   if(error[i]*effort[i]>0)integral[i]=saved[i];
   effort[i]=targets[i]?KP*error[i]+integral[i]+FEEDFORWARD*targets[i]:0;
   if(effort[i]*targets[i]<0)effort[i]=0;
   float magnitude=effort[i]<0?-effort[i]:effort[i];
   if(magnitude>peak)peak=magnitude;
  }
 }
 float scale=PWM_CAP/peak;
 for(unsigned i=0;i<4;i++){
  duty[i]=(int16_t)(effort[i]*scale)*pwm_sign[i];
  if(!targets[i]){pwm(i,0);continue;}
  if(absi(actual[i])<3&&(unsigned)absi(duty[i])>PWM_PERIOD/25)stalled[i]+=CONTROL_MS;else stalled[i]=0;
  if(actual[i]*targets[i]<0&&absi(actual[i])>15)opposite[i]+=CONTROL_MS;else opposite[i]=0;
  if(stalled[i]>STALL_TIMEOUT_MS||opposite[i]>DIRECTION_TIMEOUT_MS){latch_fault(4,5,0);stop();return;}
  pwm(i,duty[i]);
 }
}
static void control(void){
 int encoder_jump=0;
 unsigned dt=control_interval_ms?control_interval_ms:CONTROL_MS;
 /* Main's independent >30ms fault remains. No extra delay allowance here. */
 unsigned limit=(ENCODER_DELTA_LIMIT*dt+CONTROL_MS-1)/CONTROL_MS;
 for(unsigned i=0;i<4;i++){
  uint16_t count=(uint16_t)enc[i]->CNT;int16_t delta=(int16_t)(count-previous[i]);previous[i]=count;
  raw_total[i]+=(uint32_t)(int32_t)delta;
  sum_counts[i]+=(uint32_t)(int32_t)(delta*(configured?enc_sign[i]:1));
  cycle_delta[i]=delta;if((unsigned)absi(delta)>limit)encoder_jump=1;
  float speed=delta*MM_PER_COUNT*(1000.0f/dt)*enc_sign[i];
  filtered[i]=filtered[i]*0.5f+speed*0.5f;actual[i]=(int16_t)clamp(filtered[i],32760);
 }
 if(encoder_jump)latch_fault(2,4,0);
 icm20948_sample(ms,actual,!have_command);
 body.measured_vx=(filtered[0]+filtered[1]+filtered[2]+filtered[3])*.00025f;
 body.measured_yaw=imu.yaw_rate;
 battery_mv=voltage();
 select_source();
 if(fault||!enabled()||!powered()||!have_command||(uint32_t)(ms-last_command)>=WATCHDOG_MS){stop();return;}
 if(direct_mode){
  /* Zero means continuous commands; the independent stale-command watchdog
   * above still stops outputs after 500ms without valid command refresh. */
#if TEST_MAX_MS > 0
  if((uint32_t)(ms-test_started)>=TEST_MAX_MS){stop();test_blocked=1;return;}
#endif
  for(unsigned i=0;i<4;i++){
   duty[i]=(int16_t)(direct_pwm[i]*(int)PWM_PERIOD/1000);test_pwm(i,duty[i]);
  }
  return;
 }
 if(cal_index!=255){
  if((int32_t)(ms-cal_end)>=0){stop();return;}
  for(unsigned i=0;i<4;i++)pwm(i,i==cal_index?duty[i]:0);
  return;
 }
 if(!configured){stop();return;}
 if(active_source==SOURCE_BODY){
  if(!icm20948_ready(ms)){
   /* 无新鲜IMU只暂停车身导航，不锁存整板故障；0x11手柄仍可用。 */
   zero();have_command=0;for(unsigned i=0;i<4;i++)targets[i]=0;return;
  }
  body_update(sources.vx_mm_s*.001f,sources.wz_mrad_s*.001f,
              body.measured_vx,imu.yaw_rate,dt*.001f,targets);
 }
 wheel_control(dt*.001f);
}
static void telemetry(void){
 uint8_t p[61];p32(p,ms*1000u);p32(p+4,(ms-last_report)*1000u);
 for(unsigned i=0;i<4;i++){p32(p+8+4*i,sum_counts[i]);sum_counts[i]=0;}
 p[24]=(have_command?1:2)|(fault?4:0)|(configured?8:0)|(enabled()?16:0)|(powered()?32:0)|(cal_index!=255?64:0)|(direct_mode?128:0);
 p16(p+25,ack_seq);
 for(unsigned i=0;i<4;i++){p32(p+27+4*i,raw_total[i]);p16(p+43+2*i,actual[i]);p16(p+51+2*i,duty[i]);}
 p16(p+59,battery_mv);last_report=ms;emit(0x90,p,61);
 /* 旧0x90逐字节保持兼容；新增0x93承载最新10ms采样和IMU数据。 */
 uint8_t extra[64];BodyTelemetry snapshot;
 snapshot.imu_flags=(imu.present?1:0)|(imu.calibrated?2:0)|(icm20948_ready(ms)?4:0)|(imu.failed?8:0);
 snapshot.who=imu.who;snapshot.source=active_source;snapshot.mcu_us=ms*1000u;
 for(unsigned i=0;i<4;i++){snapshot.delta[i]=cycle_delta[i];snapshot.wheel_mm_s[i]=actual[i];}
 for(unsigned i=0;i<3;i++){
  snapshot.gyro_raw[i]=imu.gyro_raw[i];snapshot.accel_raw[i]=imu.accel_raw[i];
  snapshot.gyro_mrad_s[i]=imu.gyro_mrad_s[i];snapshot.accel_mm_s2[i]=imu.accel_mm_s2[i];
 }
 snapshot.measured_vx=(int16_t)(body.measured_vx*1000);snapshot.measured_wz=(int16_t)(imu.yaw_rate*1000);
 snapshot.target_vx=sources.vx_mm_s;snapshot.target_wz=sources.wz_mrad_s;
 snapshot.battery_mv=battery_mv;
 snapshot.status_flags=(enabled()?1:0)|(powered()?2:0)|(configured?4:0)|(have_command?8:0)|((unsigned)fault<<8);
 uint32_t age=imu.have_sample?(uint32_t)(ms-imu.sample_ms):65535;
 snapshot.sample_age_ms=age>65535?65535:age;snapshot.ack_seq=ack_seq;
 body_pack_telemetry(extra,&snapshot);
 emit(BODY_TELEMETRY,extra,64);
}
int main(void){
 hardware();stop();
 const int8_t ps[4]=DEFAULT_PWM_SIGNS,es[4]=DEFAULT_ENCODER_SIGNS;
 for(unsigned i=0;i<4;i++){pwm_sign[i]=ps[i];enc_sign[i]=es[i];}
 icm20948_init();last_control=last_report=ms;trace[1]=7;
 for(;;){
  trace[7]++;
  uart_pump();
  if(serial_resync){rr=rw;used=0;serial_resync=0;}
  while(rr!=rw){uint8_t b=rx[rr];rr=(rr+1)&511;parse(b);}
  if((uint32_t)(ms-last_control)>=CONTROL_MS){
   control_interval_ms=ms-last_control;
   if(control_interval_ms>CONTROL_MS*3)latch_fault(8,6,0);
   last_control=ms;control();
   /* Feed only when the timed control loop progresses. A stopped SysTick
    * must not be hidden by a busy main loop repeatedly feeding IWDG. */
   IWDG->KR=0xaaaa;
  }
  if((uint32_t)(ms-last_report)>=TELEMETRY_MS)telemetry();
 }
}
