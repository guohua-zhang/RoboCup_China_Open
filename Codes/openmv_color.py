import sensor, image, time,pyb
from pyb import UART

thresholds = ((9, 53, 25, 59, 12, 36))

# 颜色阈值的设定可以暂停mv工作，让右边有图片 然后点击左上角工具(Tools) -> 机器视觉(Machine Vision) -> 阈值编辑器(Threshold Editor) 中调试

# 颜色代码是find_blobs返回的blob对象中的一个成分， 用于标识，该色块是由在哪个阈值下选择的
# 颜色1: 红色的颜色代码
pink_color_code = 1 # code = 2^0 = 1  若颜色列表中下继有其他颜色，则可用2^x来代表code值


sensor.reset() # 初始化摄像头
sensor.set_pixformat(sensor.RGB565) # 选择像素模式 RGB565.
sensor.set_framesize(sensor.QQVGA) # use QQVGA for speed.
sensor.skip_frames(time = 2000) # Let new settings take affect.
sensor.set_auto_whitebal(False) #关闭白平衡。白平衡是默认开启的，在颜色识别中，需要关闭白平衡。
sensor.set_auto_exposure(False, 250000)


#初始化串口
uart = UART(3, 115200)
uart.init(115200, bits=8, parity=None, stop=1)
led = pyb.LED(3)

clock = time.clock() # Tracks FPS.

while(True):
    clock.tick() # Track elapsed milliseconds between snapshots().
    img = sensor.snapshot() # 拍照，返回图像
    # 在图像中寻找满足颜色阈值约束(color_threshold, 数组格式), 像素阈值pixel_threshold， 色块面积大小阈值(area_threshold)的色块
    blobs = img.find_blobs([thresholds], area_threshold=100)
    if blobs:
    #如果找到了目标颜色
        for blob in blobs:
        #迭代找到的目标颜色区域
            x = blob[0]
            y = blob[1]
            width = blob[2] # 色块矩形的宽度
            height = blob[3] # 色块矩形的高度
            color_code = blob[8] # 颜色代码

            # 添加颜色说明
            if color_code == pink_color_code:
                red = 5
                led.on()
                time.sleep_ms(100)
                led.off()
                #data = bytearray([0xb3,0xb3,red,0x5b])
                data = bytearray([0x2C,0x12,red,0x5B])  #对所需传输的数据进行封装，帧头帧尾可自定，协议写好就可以了
                uart.write(data)
                #print(data)
            #用矩形标记出目标颜色区域
            img.draw_rectangle([x, y, width, height])
    else:
        #zero = bytearray([0xb3,0xb3,0,0x5b])
        zero = bytearray([0x2C,0x12,0,0x5B])
        uart.write(zero)
        #print(zero)
    #print(clock.fps())
