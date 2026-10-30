.syntax unified
.cpu cortex-m4
.thumb
.section .isr_vector,"a",%progbits
.global vectors
vectors:
.word _estack, Reset_Handler
.word Default_Handler, Default_Handler, Default_Handler, Default_Handler, Default_Handler
.word 0,0,0,0,Default_Handler,Default_Handler,0,Default_Handler,SysTick_Handler
.rept 37
.word Default_Handler
.endr
.word USART1_IRQHandler,USART2_IRQHandler,USART3_IRQHandler
.rept 42
.word Default_Handler
.endr
.section .text.Reset_Handler,"ax",%progbits
.thumb_func
.global Reset_Handler
Reset_Handler:
 cpsid i
 /* 必须在任何C函数之前使能FPU：硬浮点ABI的main入口可能先执行vpush。
  * 若等到main内hardware()再使能，函数序言已因NOCP触发HardFault。 */
 ldr r0, =0xe000ed88
 ldr r1, [r0]
 orr r1, r1, #0x00f00000
 str r1, [r0]
 dsb
 isb
 ldr r0, =_sidata
 ldr r1, =_sdata
 ldr r2, =_edata
1:
 cmp r1,r2
 bcs 2f
 ldr r3,[r0],#4
 str r3,[r1],#4
 b 1b
2:
 ldr r1,=_sbss
 ldr r2,=_ebss
 movs r3,#0
3:
 cmp r1,r2
 bcs 4f
 str r3,[r1],#4
 b 3b
4:
 bl main
 b .
