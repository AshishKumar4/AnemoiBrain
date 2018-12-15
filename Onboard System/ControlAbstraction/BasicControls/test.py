import spidev 
spi = spidev.SpiDev()
spi.open(0, 0)
def sim():
    try:
        while True:
            r=spi.xfer2([251])
            time.sleep(0.1)
            r = spi.xfer2([101])
            time.sleep(0.1)
            for i in range(0, 18):
                r = spi.xfer2([10])
                time.sleep(0.1)
            r = spi.xfer2([251])
            time.sleep(0.1)
    except KeyboardInterrupt:
        spi.close()