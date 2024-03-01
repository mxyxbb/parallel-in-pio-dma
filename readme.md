# 【树莓派Pico】16位100MHz并口高速输入数据|读取并口数据|PIO|DMA

![pic](https://github.com/mxyxbb/parallel-in-pio-dma/assets/53026754/9aa75906-340e-4c0f-accf-4628bbaae57d)
![pic1](https://github.com/mxyxbb/parallel-in-pio-dma/assets/53026754/0c733df9-b818-46f0-b546-9fe2cce0a06d)


# 主要内容

本文实现的功能实际上十分类似于example中的逻辑分析仪logic_analyser例程。

## pio程序内容

设置1个side引脚用于输出时钟信号
通过in pins,16指令来读取16个引脚上输入的信号

```
.program parallel_in
.side_set 1

; This is just a simple clocked parallel TX. At 200 MHz system clock we can
; sustain up to 100 * 32 Mbps (16 used here).
; Data on IN pins 0~15
; Clock output on GPIO pin 16

.wrap_target
    in pins, 16   side 0
    nop           side 1
.wrap
```

## pio初始化函数

提供了两个函数，parallel_in_program_init函数用于初始化
pin_base和pin_count表示设置从标号pin_base开始的pin_count个引脚为输入引脚。本例中输入引脚为pin0~15.
初始化函数内部依序进行了
- 1.使用pio_gpio_init对用到的引脚进行**初始化**
- 2.使用pio_sm_set_consecutive_pindirs对用到的引脚设置**方向**
- 3.设置state machine的基本参数: sideset_pins、in_pins、clkdiv、in_shift、fifo_join
- 其中in_shift的参数表示：false输入移位寄存器左移, true自动装载, pin_count移位个数
- 4.使用pio_sm_init进行状态机初始化
函数parallel_in_get用于测试，功能为堵塞式读取rx fifo（实际中采用dma方式，不使用该函数）
- 使用pio_sm_is_rx_fifo_empty获取rx fifo状态
- 空时返回true，等待autopush。
- 非空时返回false，后面就将rx fifo数据读取到x内。
```
% c-sdk {

// clock_pin should be a real pin number
static inline void parallel_in_program_init(PIO pio, uint sm, uint offset, uint pin_base, uint pin_count, uint clock_pin, float div) {
    for (uint16_t i = 0; i < pin_count; i++)
        pio_gpio_init(pio, pin_base+i);
    pio_gpio_init(pio, clock_pin); //wait gpio should be inited. 
    pio_sm_set_consecutive_pindirs(pio, sm, pin_base, pin_count, false);
    pio_sm_set_consecutive_pindirs(pio, sm, clock_pin, 1, true);
    
    pio_sm_config c = parallel_in_program_get_default_config(offset);
    sm_config_set_sideset_pins(&c, clock_pin);
    sm_config_set_in_pins(&c, pin_base);
    sm_config_set_clkdiv(&c, div);
    sm_config_set_in_shift(&c, false, true, pin_count);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    pio_sm_init(pio, sm, offset, &c);
    //pio_sm_set_enabled(pio, sm, true);
}

static inline void parallel_in_get(PIO pio, uint sm, uint16_t *x) {
    while (pio_sm_is_rx_fifo_empty(pio, sm))
        ;
    *x = *(volatile uint16_t*)&pio->rxf[sm];
}

%}
```

## 主程序内容

在main.c中，在开头使用下面两个函数将rp2040的主频超频到200MHz。

```
    vreg_set_voltage(VREG_VOLTAGE_MAX);
    set_sys_clock_khz(200*1000, true);
```

**关键程序**如下。主要行为依序为
- 1.获取没用到的dma通道
- 2.向pio指令空间中加载pio程序
- 3.初始化pio程序与相关引脚，本例中数据输入引脚为0~15，时钟输出引脚为16.
- 4.初始化dma传输的源，目的，和数量。
- 5.启动dma传输
- 6.启动pio程序

```
    PIO pio = pio0;
    uint sm = 0;
    dma_chan = dma_claim_unused_channel(true);
    //dma_chan2 = dma_claim_unused_channel(true);

    uint offset = pio_add_program(pio, &parallel_in_program);
    parallel_in_program_init(pio, sm, offset, CAPTURE_PIN_BASE, CAPTURE_PIN_COUNT, CLOCK_OUTPUT_PIN, 1.f);

    // inter chain the two channel
    parallel_in_get_dma_init(pio, sm, dma_chan, capture_buf, buf_size_halfwords);

    //start one channel
    dma_channel_start(dma_chan);
    //start the pio sm
    pio_sm_set_enabled(pio, sm, true);
```

相关函数如下。该函数用于初始化dma参数
- 1.设置传输宽度为16位
- 2.read_increment设置为否（固定从sm的rxfifo中读取）
- 3.write_increment设置为是（向capture_buf为首地址的一系列空间中写入）
- 4.set_dreq表示设置dma请求源，pio_get_dreq(pio, sm, false)表示pio的rx请求。
- 就是说rx到位了就立马叫dma来搬。
- 5.最后是简单设置目的地址、源地址、传输数量，并设置是否立马开始传输（否）。

```
static inline void parallel_in_get_dma_init(PIO pio, uint sm, uint dma_chan, uint16_t *capture_buf, size_t capture_size_halfwords) {
    pio_sm_set_enabled(pio, sm, false);
    // Need to clear _input shift counter_, as well as FIFO, because there may be
    // partial ISR contents left over from a previous run. sm_restart does this.
    pio_sm_clear_fifos(pio, sm);
    pio_sm_restart(pio, sm);

    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));
    //channel_config_set_chain_to(&c, dma_chan2); //trigger the dma_chan2 after trans finished
    dma_channel_configure(dma_chan, &c,
        capture_buf,        // Destination pointer
        &pio->rxf[sm],      // Source pointer
        capture_size_halfwords, // Number of transfers
        false                // Start immediately
    );
    
    //pio_sm_set_enabled(pio, sm, true);
}
```

## 测试程序

下面是接续在**关键程序**之后的测试程序，主要功能为
- 等待dma传输完成
- 串口打印出接收缓冲数组内的数据
- 闪个灯指示一下

```
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    while(1)
    {
        //wait for dma finish
        dma_channel_wait_for_finish_blocking(dma_chan);
        for (uint i = 0; i <= buf_size_halfwords; i+=8)
        {
            printf("%4x,%4x,%4x,%4x,%4x,%4x,%4x,%4x,delta=%d\n",
            capture_buf[i],capture_buf[i+1],capture_buf[i+2],capture_buf[i+3],
            capture_buf[i+4],capture_buf[i+5],capture_buf[i+6],capture_buf[i+7],
            capture_buf[i+7]-capture_buf[i]);
            sleep_ms(1);
        }
        gpio_put(LED_PIN, 1);
        sleep_ms(100);
        gpio_put(LED_PIN, 0);
        sleep_ms(100);
        dma_channel_set_write_addr(dma_chan, capture_buf, true);

    }
```
