#从imgae模块引入SEARCH_EX和SEARCH_DS。使用from import仅仅引入SEARCH_EX,
#SEARCH_DS两个需要的部分，而不把image模块全部引入。
import time#引入延时函数
import time, sensor, image , pyb
from image import SEARCH_EX, SEARCH_DS
from pyb import UART

# Reset sensor
sensor.reset()
sensor.set_hmirror(True)  # 水平翻转
sensor.set_vflip(True)    # 竖直翻转

# Set sensor settings
sensor.set_contrast(1)
sensor.set_gainceiling(16)
# Max resolution for template matching with SEARCH_EX is QVGA
sensor.set_framesize(sensor.VGA)
# You can set windowing to reduce the search image.
#sensor.set_windowing(((640-80)//2, (480-60)//2, 80, 60))
sensor.set_pixformat(sensor.GRAYSCALE)

# init uart
uart = UART(3, 115200)
uart.init(115200, bits=8, parity=None, stop=1)
led = pyb.LED(3) # Red LED = 1, Green LED = 2, Blue LED = 3, IR LEDs = 4.

# Load template.
# Template should be a small (eg. 32x32 pixels) grayscale image.
#templates = ["/11.pgm", "/12.pgm"] #保存数字1的模板
#templates1 = ["/21.pgm", "/22.pgm", "/23.pgm", "/24.pgm"] #保存数字2的模板
#templates2 = ["/305.pgm", "/304.pgm", "/309.pgm", "/329.pgm", "/330.pgm","/311.pgm","/312.pgm"] #保存数字3的模板
#templates3 = ["/41.pgm", "/42.pgm"] #保存数字4的模板
#templates4 = ["/51.pgm", "/52.pgm","/53.pgm", "/54.pgm","/55.pgm", "/56.pgm","/57.pgm", "/58.pgm","/59.pgm", "/510.pgm","/511.pgm","/512.pgm","/513.pgm", "/514.pgm","/515.pgm", "/516.pgm","/517.pgm"] #保存数字5的模板
templates5 = ["/61.pgm", "/62.pgm", "/63.pgm","/64.pgm","/65.pgm"] #保存数字6的模板
# templates6 = ["/71.pgm", "/72.pgm"] #保存数字7的模板
# templates7 = ["/81.pgm", "/82.pgm", "/83.pgm","/84.pgm","/85.pgm"] #保存数字8的模板
#加载模板图片

clock = time.clock()


# 步进值 stp 决定我当下要走哪两个循环
stp = 1

# Run template matching
while (True):
    clock.tick()
    #img = sensor.snapshot()
    #if stp == 1:
        ## 识别数字 3
        #img2 = sensor.snapshot().lens_corr(1.5)
        #for t2 in templates2:
            #template2 = image.Image(t2)
            #r2 = img2.find_template(template2, 0.70, roi=(0,0,160,120),step=6, search=SEARCH_DS) #, roi=(10, 0, 60, 60))
            #if r2:
                #img2.draw_rectangle(r2)
                #stp = stp + 1
                #data = bytearray([0x2C,0x12,0x33,0x5B])  #对所需传输的数据进行封装，帧头帧尾可自定，协议写好就可以了
                #uart.write(data)
                #print(data)
                #break


        # 识别数字 4
        #img3 = sensor.snapshot().lens_corr(1.5)
        #for t3 in templates3:
            #template3 = image.Image(t3)
            ##对每个模板遍历进行模板匹配
            #r3 = img3.find_template(template3, 0.70, roi=(0,0,160,120),step=6, search=SEARCH_DS) #, roi=(10, 0, 60, 60))
            #if r3:
                #img3.draw_rectangle(r3)
                #stp = stp + 1
                #data = bytearray([0x2C,0x12,0x34,0x5B])  #对所需传输的数据进行封装，帧头帧尾可自定，协议写好就可以了
                #uart.write(data)
                #print(data)
                #break


    #else if stp == 2:
    if stp == 1:
        ## 识别数字5
        #img4 = sensor.snapshot().lens_corr(1.5)
        #for t4 in templates4:
            #template4 = image.Image(t4)
            #r4 = img4.find_template(template4, 0.70, roi=(0,0,160,120),step=6, search=SEARCH_DS) #, roi=(10, 0, 60, 60))
            #if r4:
                #led.on()
                #img4.draw_rectangle(r4)
                #data = bytearray([0x2C,0x12,0x35,0x5B])  #对所需传输的数据进行封装，帧头帧尾可自定，协议写好就可以了
                #uart.write(data)
                #print(data)
                #break


        ## 识别数字6
        img5 = sensor.snapshot().lens_corr(1.5)
        for t5 in templates5:
            template5 = image.Image(t5)
            r5 = img5.find_template(template5, 0.70,roi=(0,0,160,120), step=6, search=SEARCH_DS) #, roi=(10, 0, 60, 60))
            if r5:
                img5.draw_rectangle(r5)
                led.on()
                #stp = stp + 1
                #data = bytearray([0x2C,0x12,0x36,0x5B])  #对所需传输的数据进行封装，帧头帧尾可自定，协议写好就可以了
                uart.write('6')
                #print(data)
                break

    #else if stp == 3:
        ## 识别数字 7
        #img6 = sensor.snapshot().lens_corr(1.5)
        #img6 = sensor.snapshot()
        #for t6 in templates6:
            #template6 = image.Image(t6)
            #r6 = img6.find_template(template6, 0.70,roi=(0,0,160,120), step=6, search=SEARCH_DS) #, roi=(10, 0, 60, 60))
            #if r6:
                #img6.draw_rectangle(r6)
                #stp = stp + 1
                #data = bytearray([0x2C,0x12,0x37,0x5B])  #对所需传输的数据进行封装，帧头帧尾可自定，协议写好就可以了
                #uart.write(data)
                #print(data)
                #break

        ## 识别数字 8
        #img7 = sensor.snapshot().lens_corr(1.5)
        ##img7 = sensor.snapshot()
        #for t7 in templates7:
            #template7 = image.Image(t7)
            ##对每个模板遍历进行模板匹配
            #r7 = img7.find_template(template7, 0.70, roi=(0,0,160,120),step=6, search=SEARCH_DS) #, roi=(10, 0, 60, 60))
        ##find_template(template, threshold, [roi, step, search]),threshold中
        ##的0.7是相似度阈值,roi是进行匹配的区域（左上顶点为（10，0），长80宽60的矩形），
        ##注意roi的大小要比模板图片大，比frambuffer小。
        ##把匹配到的图像标记出来
            #if r7:
                #img7.draw_rectangle(r7)
                #stp = stp + 1
                #data = bytearray([0x2C,0x12,0x38,0x5B])  #对所需传输的数据进行封装，帧头帧尾可自定，协议写好就可以了
                #uart.write(data)
                #print(data)
                #break
        ##time.sleep(0.3)
        ##print(clock.fps())
    #else:
        #break
