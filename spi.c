/*
 * sunxi a64 spi slave mode monitor
 * 2017 Tong Zhang<ztong@vt.edu>
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/input/mt.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/fs.h>

#include <asm/io.h>
#include <asm/uaccess.h> /* for put_user */


MODULE_LICENSE("Dual BSD/GPL");

/*
 * Port Controller
 */
#define PIO_BASE_ADDRESS 0x01C20800
#define PIO_SIZE 0x258

#define PC_CFG0_REG 0x48
#define PC_PULL0_REG 0x64

/*
 * PC_CFG0_REG config bits
 */
#define PC3_SELECT 12
#define PC2_SELECT 8
#define PC1_SELECT 4
#define PC0_SELECT 0
/*
 * PC_PULL0_REG register offset?
 */

/*
 * CCU stuff
 */
#define CCU_BASE_ADDRESS 0x01C20000
#define CCU_SIZE 0x324
/*
 * CCU register offset
 */
#define PLL_PERIPH0_CTRL_REG 0x0028
#define PLL_PERIPH1_CTRL_REG 0x002C
#define BUS_CLK_GATING_REG0 0x0060
#define BUS_CLK_GATING_REG1 0x0064
#define BUS_CLK_GATING_REG2 0x0068
#define BUS_CLK_GATING_REG3 0x006C
#define BUS_CLK_GATING_REG4 0x0070
#define SPI0_CLK_REG 0x00A0
#define SPI1_CLK_REG 0x00A4
#define BUS_SOFT_RST_REG0 0x02C0

/*
 * Bus Clock Gating Register0 
 */
#define SPI1_GATING 21
#define SPI0_GATING 20

/*
 * SPI CLK REG
 */
#define SCLK_GATING 31
#define CLK_SRC_SEL 24
#define CLK_DIV_RATIO_N 16
#define CLK_DIV_RATIO_M 0

/*
 * BUS_SOFT_RST_REG0
 */
#define SPI1_RST 21
#define SPI0_RST 20


/*
 * IRQ
 */
#define SPI_0_IRQ 97
#define SPI_1_IRQ 98

#define SPI_0_BASE_ADDRESS 0x01C68000
#define SPI_1_BASE_ADDRESS 0x01C69000

#define SPI_REGISTER_MEMORY_SIZE 0x400
/*
 * register offset
 */
#define SPI_GCR 0x04
#define SPI_TCR 0x08
#define SPI_IER 0x10
#define SPI_ISR 0x14
#define SPI_FCR 0x18
#define SPI_FSR 0x1C
#define SPI_WCR 0x20
#define SPI_CCR 0x24
#define SPI_MBC 0x30
#define SPI_MTC 0x34
#define SPI_BCC 0x38
#define SPI_NDMA_MODE_CTL 0x88
#define SPI_TXD 0x200
#define SPI_RXD 0x300

/*
 * GCR Control bits
 */
#define SRST (31)
#define TP_EN (7)
#define MODE (1)
#define EN (0)

/*
 * TCR Control bits
 */

#define XCH (31)
#define SDDM (14)
#define SDM (13)
#define FBS (12)
#define SDC (11)
#define RPSM (10)
#define DDB (9)
#define DHB (8)
#define SS_LEVEL (7)
#define SS_OWNER (6)
#define SS_SEL (4)
#define SSCTL (3)
#define SPOL (2)
#define CPOL (1)
#define CPHA (0)

/*
 * IER Control bits
 */
#define SS_INT_EN 13
#define TC_INT_EN 12
#define TF_UDR_INT_EN 11
#define TFOVF_INT_EN 10
#define RF_UDR_INT_EN 9
#define RF_OVF_INT_EN 8
#define TF_FUL_INT_EN 6
#define TF_EMP_INT_EN 5
#define TF_ERQ_INT_EN 4
#define RF_FUL_INT_EN 2
#define RX_EMP_INT_EN 1
#define RF_RDY_INT_EN 0
/*
 * ISR Control bits
 */
#define SSI 13
#define TC 12
#define TF_UDF 11
#define TF_OVF 10
#define RX_UDF 9
#define RX_OVF 8
#define TX_FULL 6
#define TX_EMP 5
#define TX_READY 4
#define RX_FULL 2
#define RX_EMP 1
#define RX_READY 0

/*
 * FCR control bits
 */
#define TX_FIFO_RST 31
#define TF_TEST_ENB 30
#define TF_DRQ_EN 24
#define TX_TRIG_LEVEL 16
#define RF_RST 15
#define RF_TEST 14
#define RX_DMA_MODE 9
#define RF_DRQ_EN 8
#define RX_TRIG_LEVEL 0

/*
 * SPI_FSR bits
 */
#define TB_WR 31
#define TB_CNT 28
#define TF_CNT 16
#define RB_WR 15
#define RB_CNT 12
#define RF_CNT 0

#define SPI_BASE_ADDRESS SPI_0_BASE_ADDRESS
#define SPI_IRQ SPI_0_IRQ

static void __iomem *spi_base;
#define RX_BUFFER_SIZE 0x1000
static u8 rx_buffer[RX_BUFFER_SIZE];
int rx_buf_ptr_head;
int rx_buf_ptr_tail;

void spi_mon_dump_data_tl(unsigned long);
DECLARE_TASKLET(spi_mon_tasklet, spi_mon_dump_data_tl, 0);

void spi_mon_dump_data_tl (unsigned long unused)
{
    while (rx_buf_ptr_head!=rx_buf_ptr_tail)
    {
        printk("spi_mon:0x%02x\n", rx_buffer[rx_buf_ptr_tail]);
        rx_buf_ptr_tail = (rx_buf_ptr_tail+1) % RX_BUFFER_SIZE;
    }
}

static irqreturn_t spi_mon_irq_handler(int irq, void *dev_id)
{
    u32 irq_register;
    irq_register = readl(spi_base + SPI_ISR);

    if ((irq_register & (1<<SSI)) ||
        (irq_register & (1<<RX_READY)) ||
        (irq_register & (1<<RX_FULL)))
    {
        //read out things from rx fifo
        u8 rf_cnt = (u8)readl(spi_base + SPI_FSR) & 0xFF;
        int i;
        for (i=0;i<rf_cnt;i++)
        {
            rx_buffer[rx_buf_ptr_head] = readb(spi_base+SPI_RXD);
            rx_buf_ptr_head = (rx_buf_ptr_head+1) % RX_BUFFER_SIZE;
        }
    }
    //clear all bits
    writel(irq_register, spi_base + SPI_ISR);
    /*
    if (rx_buf_ptr_head != rx_buf_ptr_tail)
    {
        tasklet_schedule(&spi_mon_tasklet);
    }*/

    return IRQ_HANDLED;
}

static void setup_spi_pin(void)
{
    static void __iomem* pcr_base;
    u32 tmp;
    pcr_base = ioremap(PIO_BASE_ADDRESS, PIO_SIZE);
    tmp = readl(pcr_base + PC_CFG0_REG);
    printk("SPI PIO PC_CFG0_REG : 0x%x\n", tmp);
    tmp &= ~((7<<PC3_SELECT) | (7<<PC2_SELECT) | (7<<PC1_SELECT) | (7<<PC0_SELECT));
    tmp |= (4<<PC3_SELECT) | (4<<PC2_SELECT) | (4<<PC1_SELECT) | (4<<PC0_SELECT);
    printk(" -> set to 0x%x\n", tmp);
    writel(tmp, pcr_base + PC_CFG0_REG);
    //internal pull register
    tmp = readl(pcr_base + PC_PULL0_REG);
    printk("SPI PIO PC_PULL0_REG : 0x%x\n", tmp);
    //PC3->CS
    tmp &= ~(3<<(2*3));
    //tmp |= (1<<6);
    //clear PC2-PC0(MOSI,MISO,SCLK) they are tri state
    tmp &= ~((3<<(2*2))|(3<<(2*1))|(3<<(2*0)));
    printk(" -> set to 0x%x\n", tmp);
    writel(tmp, pcr_base + PC_PULL0_REG);
    
    iounmap(pcr_base);
}

static void setup_spi_clk(void)
{
    static void __iomem* ccu_base;
    u32 tmp;
    ccu_base = ioremap(CCU_BASE_ADDRESS, CCU_SIZE);

    printk("SPI CCU RESET SPI BUS\n");
    tmp = readl(ccu_base+BUS_SOFT_RST_REG0);
    printk("SPI CCU BUS_SOFT_RST_REG0:0x%x\n", tmp);
    //toggle rst?
    tmp &= ~(1<<SPI0_RST);
    writel(tmp, ccu_base+BUS_SOFT_RST_REG0);
    tmp |= (1<<SPI0_RST);
    writel(tmp, ccu_base+BUS_SOFT_RST_REG0);
    tmp = readl(ccu_base+BUS_SOFT_RST_REG0);
    printk("SPI CCU BUS_SOFT_RST_REG0:0x%x\n", tmp);

    printk("SPI CCU setup pll\n");
    tmp = readl(ccu_base+PLL_PERIPH0_CTRL_REG);
    printk("SPI CCU PLL_PERIPH0_CTRL_REG : 0x%x\n", tmp);

    tmp = readl(ccu_base+PLL_PERIPH1_CTRL_REG);
    printk("SPI CCU PLL_PERIPH1_CTRL_REG : 0x%x\n", tmp);

    printk("SPI CCU setup SPI CLK REG\n");
    tmp = readl(ccu_base+SPI0_CLK_REG);

    printk("SPI CCU SPI CLK REG : 0x%x\n", tmp);
    writel((1<<CLK_SRC_SEL) | (3<<CLK_DIV_RATIO_N) | (2<<CLK_DIV_RATIO_M),
            ccu_base+SPI0_CLK_REG);

    tmp = readl(ccu_base+SPI0_CLK_REG);
    printk("SPI CCU SPI CLK REG : 0x%x\n", tmp);
    tmp |= (1<<SCLK_GATING);
    writel(tmp, ccu_base+SPI0_CLK_REG);
    tmp = readl(ccu_base+SPI0_CLK_REG);
    printk("SPI CCU SPI CLK REG : 0x%x\n", tmp);

    printk("SPI CCU setup SPI GATING\n");

    tmp = readl(ccu_base+BUS_CLK_GATING_REG0);
    printk("SPI CCU GATING REG0 : 0x%x\n", tmp);
    writel(tmp | (1<<SPI0_GATING) , ccu_base+BUS_CLK_GATING_REG0);
    tmp = readl(ccu_base+BUS_CLK_GATING_REG0);
    printk("SPI CCU GATING REG : 0x%x\n", tmp);


    iounmap(ccu_base);
}

static int device_open(struct inode *, struct file *);
static int device_release(struct inode *, struct file *);
static ssize_t device_read(struct file *, char *, size_t, loff_t *);
static ssize_t device_write(struct file *, const char *, size_t, loff_t *);

static struct file_operations fops = {
    .read = device_read,
    .write = device_write,
    .open = device_open,
    .release = device_release
};

static int major;

static int spi_mon_init(void)
{
    int ret;
    u32 tmp;

    printk(KERN_ALERT "Hello, world, from SPI\n");

    setup_spi_pin();

    setup_spi_clk();

    rx_buf_ptr_head = 0;
    rx_buf_ptr_tail = 0;

    spi_base = ioremap(SPI_BASE_ADDRESS, SPI_REGISTER_MEMORY_SIZE);

    //reset spi
    tmp = readl(spi_base + SPI_GCR);
    printk("SPI GCR : 0x%x resetting...\n", tmp);
    writel((1<<SRST)|(1<<TP_EN), spi_base + SPI_GCR);
    while(1)
    {
        tmp = readl(spi_base + SPI_GCR);
        if (!(tmp & (1<<SRST)))
        {
            break;
        }
    }
    printk("SPI reset complete GCR : 0x%x\n", tmp);

    //enable SPI
    writel((1<<TP_EN)|(1<<EN), spi_base + SPI_GCR);
    tmp = readl(spi_base + SPI_GCR);
    printk("SPI GCR : 0x%x\n", tmp);

    //TCR
    tmp = readl(spi_base + SPI_TCR);
    printk("SPI TCR : 0x%x\n", tmp);
    tmp |= (1<<RPSM)|(1<<SPOL)|(1<<CPOL)|(1<<CPHA);
    writel(tmp, spi_base + SPI_TCR);

    tmp = readl(spi_base + SPI_TCR);
    printk("SPI TCR : 0x%x\n", tmp);

    ret = request_irq(SPI_IRQ, spi_mon_irq_handler, IRQF_TRIGGER_HIGH, "spi_mon_0", 0);
    if (ret)
    {
        printk(KERN_ALERT "%s: request_irq failed with %d\n",  __func__, ret);
        goto out;
    }
    //setup FCR
    tmp = readl(spi_base + SPI_FCR);
    printk("SPI FCR resetting FIFO : 0x%x\n", tmp);

    writel((1<<TX_FIFO_RST)|(1<<RF_RST), spi_base + SPI_FCR);

    while(1)
    {
        tmp = readl(spi_base + SPI_FCR);
        if ((tmp & ((1<<TX_FIFO_RST) | (1<<RF_RST)))==0)
        {
            break;
        }
    }
    tmp = readl(spi_base + SPI_FCR);
    tmp |= (0x40<<TX_TRIG_LEVEL)|(0x10<<RX_TRIG_LEVEL);
    printk("SPI FCR trigger level: 0x%x\n", tmp);
    writel(tmp, spi_base + SPI_FCR );

    //enable irq
    tmp = readl(spi_base + SPI_IER);
    printk("SPI IER : 0x%x\n", tmp);
    writel((1<<RF_RDY_INT_EN)|(1<<RF_FUL_INT_EN)|
            (1<<SS_INT_EN)|(1<<TC_INT_EN),
            spi_base + SPI_IER);
    tmp = readl(spi_base + SPI_IER);
    printk("SPI IER : 0x%x\n", tmp);

//register char device
    major = register_chrdev(0, "spi_mon", &fops);
    printk("created spi_mon with major %d\n", major);
    printk("creat device using mknod /dev/spi_mon c %d 0\n", major);
    return 0;

out:
    return -1;
}

static void spi_mon_exit(void)
{
    printk(KERN_ALERT "Goodbye, cruel world, from SPI\n");
    //disable irq
    writel(0, spi_base + SPI_IER);
    free_irq(SPI_IRQ, 0);
    writel((1<<SRST)|(1<<TP_EN), spi_base + SPI_GCR);
    iounmap(spi_base);
    //spi_mon_dump_data_tl(0);
    unregister_chrdev(major, "spi_mon");
}

static int device_open(struct inode *inode, struct file *filp)
{
    try_module_get(THIS_MODULE);
    return 0;
}

static int device_release(struct inode *inode, struct file *filp)
{
    module_put(THIS_MODULE);
    return 0;
}

static ssize_t device_read(struct file *filp, /* see include/linux/fs.h   */
                           char *buffer,      /* buffer to fill with data */
                           size_t length,     /* length of the buffer     */
                           loff_t *offset)
{
	int bytes_read = 0;
	while (length && (rx_buf_ptr_head!=rx_buf_ptr_tail))
	{
		put_user(rx_buffer[rx_buf_ptr_tail], buffer++);
		length--;
		bytes_read++;
		rx_buf_ptr_tail = (rx_buf_ptr_tail+1) % RX_BUFFER_SIZE;
	}
	return bytes_read;
}



static ssize_t
device_write(struct file *filp, const char *buf, size_t len, loff_t *off)
{
    printk(KERN_ALERT "Sorry, this operation isn't supported.\n");
    return -EINVAL;
}


module_init(spi_mon_init);
module_exit(spi_mon_exit);

